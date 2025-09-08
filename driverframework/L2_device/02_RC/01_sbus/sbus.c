#include "sbus.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "rtconfig.h"
#include "uartConfig.h"
#include "sbus_proto.h"
#include "hal/rc/rc.h"
#include "ipc/ringbuffer.h"

#ifdef RC_SBUS_DEBUGPIN_EN
#include "debugPin.h"
#endif

/* SBUS protocol constants and flags are defined in sbus_proto.h */

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

struct uart_rx_msg {
    rt_device_t dev;
    rt_size_t size;
};

static rt_device_t sbus_uart = RT_NULL;
static struct serial_configure sbus_uart_cfg;

static struct rt_messagequeue sbus_rx_mq;
static char sbus_msg_pool[256];

/* Ringbuffer for UART RX accumulation */
static struct rt_ringbuffer sbus_rb;
static uint8_t sbus_rb_pool[SBUS_FRAME_SIZE * 4];
/* carry buffer for sliding-window parse when a frame is invalid */
static uint8_t sbus_carry_buf[SBUS_FRAME_SIZE];
static size_t sbus_carry_len = 0;

static sbus_rc_data_t g_sbus_data;
static struct rt_mutex g_sbus_lock;

static struct rt_event g_sbus_event; /* EVENT: new frame ready */
static volatile rt_uint8_t g_sbus_has_data = 0;

/* forward decl */
static void sbus_thread_entry(void *param);
static rt_err_t sbus_rc_control(rc_dev_t rc, int cmd, void* arg);
static rt_uint16_t sbus_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val);
static rt_err_t sbus_init_uart(const char* uart_name);
static rt_err_t sbus_init_ipc(void);
static rt_err_t sbus_register_rc(void);
static rt_err_t sbus_start_thread(void);
static void sbus_on_uart_chunk(const uint8_t* data, rt_size_t len);
static void sbus_process_ringbuffer(void);

static const struct rc_ops sbus_rc_ops = {
    .rc_config = RT_NULL,
    .rc_control = sbus_rc_control,
    .rc_read = sbus_rc_read,
};

static struct rc_device sbus_rc_dev = {
    .config = {
        .protocol = RC_PROTOCOL_SBUS,
        .channel_num = 16,
        .sample_time = 0.02f,
        .rc_min_value = 1000,
        .rc_max_value = 2000,
    },
    .ops = &sbus_rc_ops,
};

static void sbus_update_data(const uint8_t *frame)
{
    uint16_t ch[16];
    uint8_t flags = 0;
    sbus_proto_decode_channels(frame, ch, &flags);
    bool failsafe = (flags & SBUS_FLAG_FAILSAFE) != 0;

    sbus_rc_data_t data;
    data.timestamp = rt_tick_get();

    /* Map channels to sticks: center 1024, scale to -500..+500 or 0..1000 for throttle */
    data.roll  = ((int32_t)ch[0] - 1024) * 0.48828125f;   /* approx 1000 span -> +-500 */
    data.pitch = ((int32_t)ch[1] - 1024) * 0.48828125f;
    data.thrust = (float)(ch[2] - 172) * (1000.0f / 1639.0f); /* map 172..1811 -> 0..1000 */
    data.yaw   = ((int32_t)ch[3] - 1024) * 0.48828125f;

    /* Arm switch on ch5 (index 4): >1500 -> armed */
    data.arm_status = ch[4] > 1500;

    /* Mode switch placeholder on ch6 (index 5): >1500 -> rate */
    data.ctrl_mode = ch[5] > 1500;

    if (failsafe)
    {
        data.roll = 0.0f;
        data.pitch = 0.0f;
        data.yaw = 0.0f;
        data.thrust = 0.0f;
        data.arm_status = false;
    }

    rt_mutex_take(&g_sbus_lock, RT_WAITING_FOREVER);
    g_sbus_data = data;
    rt_mutex_release(&g_sbus_lock);

    /* notify readers */
    rt_event_send(&g_sbus_event, 0x01);
    g_sbus_has_data = 1;
}

static rt_err_t sbus_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    struct uart_rx_msg msg;
    msg.dev = dev;
    msg.size = size;
    rt_err_t result = rt_mq_send(&sbus_rx_mq, &msg, sizeof(msg));
#ifdef RC_SBUS_DEBUGPIN_EN
    DEBUG_PIN_DEBUG1_HIGH();
#endif
    if (result == -RT_EFULL) {
        /* drop notification if queue full */
        return RT_EOK;
    }
    return result;
}

int sbus_init(void)
{
    rt_err_t ret;
    char uart_name[RT_NAME_MAX];

    rt_strncpy(uart_name, RC_SBUS_UART_NAME, RT_NAME_MAX);

    /* 1) IPC (mutex/event/mq) */
    ret = sbus_init_ipc();
    if (ret != RT_EOK) return ret;

    /* 2) UART (find/open/config/callback) */
    ret = sbus_init_uart(uart_name);
    if (ret != RT_EOK) return ret;

    /* 3) RC register */
    ret = sbus_register_rc();
    if (ret != RT_EOK) return ret;

    /* 4) Worker thread */
    ret = sbus_start_thread();
    if (ret != RT_EOK) return ret;

    return RT_EOK;
}

/* UART group */
static rt_err_t sbus_init_uart(const char* uart_name)
{
    rt_err_t ret;
    sbus_uart = rt_device_find(uart_name);
    if (!sbus_uart)
    {
        rt_kprintf("[SBUS] find %s failed\n", uart_name);
        return -RT_ERROR;
    }
    ret = rt_device_open(sbus_uart, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    if (ret != RT_EOK)
    {
        rt_kprintf("[SBUS] open %s failed\n", uart_name);
        return ret;
    }
    ret = uart_config_by_device_name(uart_name, RC_SBUS_BAUDRATE, &sbus_uart_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[SBUS] get uart cfg failed\n");
        return ret;
    }
    ret = rt_device_control(sbus_uart, RT_DEVICE_CTRL_CONFIG, &sbus_uart_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[SBUS] config uart failed\n");
        return ret;
    }
    rt_device_set_rx_indicate(sbus_uart, sbus_uart_rx_ind);
    return RT_EOK;
}

/* IPC group */
static rt_err_t sbus_init_ipc(void)
{
    rt_mutex_init(&g_sbus_lock, "sbuslk", RT_IPC_FLAG_FIFO);
    rt_event_init(&g_sbus_event, "sbus_evt", RT_IPC_FLAG_FIFO);
    rt_ringbuffer_init(&sbus_rb, sbus_rb_pool, sizeof(sbus_rb_pool));
    sbus_carry_len = 0;
    rt_err_t ret = rt_mq_init(&sbus_rx_mq, "sbus_mq", sbus_msg_pool, sizeof(struct uart_rx_msg), sizeof(sbus_msg_pool), RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK) {
        rt_kprintf("[SBUS] mq init failed\n");
        return ret;
    }
    return RT_EOK;
}

/* RC group */
static rt_err_t sbus_register_rc(void)
{
    rt_err_t result = hal_rc_register(&sbus_rc_dev, "rc_sbus", RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK) {
        rt_kprintf("[SBUS] rc register failed\n");
        return result;
    }
    return RT_EOK;
}

/* Thread group */
static rt_err_t sbus_start_thread(void)
{
    rt_thread_t th = rt_thread_create("sbus_task", sbus_thread_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (th == RT_NULL) {
        rt_kprintf("[SBUS] thread create failed\n");
        return -RT_ERROR;
    }
    rt_thread_startup(th);
    return RT_EOK;
}

sbus_rc_data_t sbus_get_data(void)
{
    sbus_rc_data_t out;
    rt_mutex_take(&g_sbus_lock, RT_WAITING_FOREVER);
    out = g_sbus_data;
    rt_mutex_release(&g_sbus_lock);
    return out;
}

int sbus_send_channels(const uint16_t ch[16], bool ch17, bool ch18)
{
    if (sbus_uart == RT_NULL) return -RT_ERROR;

    uint8_t frame[SBUS_FRAME_SIZE];
    if (sbus_proto_encode_frame(ch, ch17, ch18, frame) != SBUS_FRAME_SIZE)
        return -RT_ERROR;
    return rt_device_write(sbus_uart, 0, frame, sizeof(frame)) == sizeof(frame) ? RT_EOK : -RT_ERROR;
}

/* Thread: wait for rx notifications, then read and parse frames */
static void sbus_thread_entry(void *param)
{
    struct uart_rx_msg msg;
    uint8_t read_buf[64];

    while (1) {
        rt_memset(&msg, 0, sizeof(msg));
        if (rt_mq_recv(&sbus_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) != RT_EOK) {
            continue;
        }
#ifdef RC_SBUS_DEBUGPIN_EN
        DEBUG_PIN_DEBUG1_LOW();
#endif
        /* drain UART indicated bytes; keep reading until empty */
        while (1) {
            rt_size_t rx_len = rt_device_read(msg.dev, 0, read_buf, sizeof(read_buf));
            if (rx_len <= 0) break;
            sbus_on_uart_chunk(read_buf, rx_len);
            sbus_process_ringbuffer();
        }
    }
}

static void sbus_on_uart_chunk(const uint8_t* data, rt_size_t len)
{
    if (len == 0) return;
    rt_ringbuffer_put_force(&sbus_rb, data, (rt_uint32_t)len);
}

static void sbus_process_ringbuffer(void)
{
    /* process while enough data + carry to form a frame */
    while ((rt_ringbuffer_data_len(&sbus_rb) + sbus_carry_len) >= SBUS_FRAME_SIZE) {
        uint8_t frame[SBUS_FRAME_SIZE];
        rt_size_t need = SBUS_FRAME_SIZE - sbus_carry_len;
        /* fill from carry first */
        if (sbus_carry_len > 0) {
            rt_memcpy(frame, sbus_carry_buf, sbus_carry_len);
        }

        /* if no carry, skip bytes until header */
        if (sbus_carry_len == 0) {
            /* drop until header appears as first byte */
            rt_uint8_t ch;
            while (rt_ringbuffer_data_len(&sbus_rb) > 0) {
                if (rt_ringbuffer_getchar(&sbus_rb, &ch) == 1) {
                    if (ch == SBUS_HEADER) { frame[0] = ch; need = SBUS_FRAME_SIZE - 1; break; }
                }
            }
            if (need == SBUS_FRAME_SIZE) {
                /* no header found */
                return;
            }
        }

        /* read remaining bytes to complete a frame */
        if (need > 0) {
            rt_size_t got = rt_ringbuffer_get(&sbus_rb, frame + sbus_carry_len, (rt_uint32_t)need);
            if (got < need) {
                /* not enough, stash into carry and wait more */
                sbus_carry_len += got;
                rt_memcpy(sbus_carry_buf, frame, sbus_carry_len);
                return;
            }
        }

        /* we have SBUS_FRAME_SIZE bytes in frame */
        if (sbus_proto_is_valid_frame(frame, SBUS_FRAME_SIZE)) {
            sbus_update_data(frame);
            sbus_carry_len = 0;
        } else {
            /* find next header inside frame[1..] to keep as carry */
            size_t k = 1;
            for (; k < SBUS_FRAME_SIZE; k++) {
                if (frame[k] == SBUS_HEADER) break;
            }
            if (k < SBUS_FRAME_SIZE) {
                sbus_carry_len = SBUS_FRAME_SIZE - 1 - k;
                if (sbus_carry_len > 0) {
                    rt_memcpy(sbus_carry_buf, &frame[k + 1], sbus_carry_len);
                }
                /* keep header for next round by placing as first byte in frame on next loop */
                sbus_carry_buf[sbus_carry_len++] = 0; /* placeholder for header alignment */
                /* Move header to beginning by setting up so next loop will search header again */
                /* Simpler: discard carry and continue searching fresh */
                sbus_carry_len = 0;
            } else {
                sbus_carry_len = 0;
            }
        }
    }
}

int sbus_read(sbus_rc_data_t* out, int timeout_ms)
{
    if (!out) return -RT_ERROR;
    rt_uint32_t recved = 0;
    rt_err_t er = rt_event_recv(&g_sbus_event, 0x01, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                timeout_ms < 0 ? RT_WAITING_FOREVER : timeout_ms, &recved);
    if (er == RT_EOK) {
        rt_mutex_take(&g_sbus_lock, RT_WAITING_FOREVER);
        *out = g_sbus_data;
        rt_mutex_release(&g_sbus_lock);
        return RT_EOK;
    }
    /* timeout: failsafe neutral */
    out->timestamp = rt_tick_get();
    out->roll = 0.0f;
    out->pitch = 0.0f;
    out->yaw = 0.0f;
    out->thrust = 0.0f;
    out->arm_status = false;
    out->ctrl_mode = false;
    return -RT_ETIMEOUT;
}

/* RC OPS */
static rt_err_t sbus_rc_control(rc_dev_t rc, int cmd, void* arg)
{
    if (cmd == RC_CMD_CHECK_UPDATE && arg) {
        *(uint8_t*)arg = g_sbus_has_data ? 1 : 0;
        return RT_EOK;
    }
    return RT_EOK;
}

static rt_uint16_t sbus_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val)
{
    /* Map g_sbus_data to 4 primary channels: 1 roll, 2 pitch, 3 thrust, 4 yaw */
    sbus_rc_data_t snap;
    rt_mutex_take(&g_sbus_lock, RT_WAITING_FOREVER);
    snap = g_sbus_data;
    rt_mutex_release(&g_sbus_lock);

    rt_uint16_t written = 0;
    for (int i = 0; i < rc->config.channel_num; i++) {
        if (chan_mask & (1 << i)) {
            rt_int16_t val = 1500;
            switch (i) {
                case 0: val = (rt_int16_t)(1500 + snap.roll); break;
                case 1: val = (rt_int16_t)(1500 + snap.pitch); break;
                case 2: val = (rt_int16_t)(1000 + snap.thrust); break;
                case 3: val = (rt_int16_t)(1500 + snap.yaw); break;
                default: val = 1500; break;
            }
            chan_val[i] = val;
            written += 2;
        }
    }
    return written;
}

#ifdef RC_USING_SBUS
INIT_DEVICE_EXPORT(sbus_init);
#endif


