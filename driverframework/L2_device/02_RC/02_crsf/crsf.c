#include "crsf.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "rtconfig.h"
#include "uartConfig.h"

#ifdef RC_CRSF_DEBUGPIN_EN
#include "debugPin.h"
#endif

/* CRSF protocol basics */
#define CRSF_SYNC_BYTE              0xC8
#define CRSF_FRAME_TYPE_RC_CHANNELS 0x16

/* 16 channels x 11-bit -> 22 bytes payload; CRSF packs slightly different but still 11-bit each */

static rt_device_t crsf_uart = RT_NULL;
static struct serial_configure crsf_uart_cfg;

static uint8_t rx_buf[64];
static size_t rx_len = 0;

static crsf_rc_data_t g_crsf_data;
static struct rt_mutex g_crsf_lock;

static void crsf_update_data_from_channels(const uint8_t *payload, size_t len)
{
    /* Expect 22 bytes channel payload */
    if (len < 22) return;
    uint16_t ch[16] = {0};

    /* CRSF RC channels are packed 11-bit little-endian, same extraction approach */
    uint32_t bitpos = 0;
    for (int i = 0; i < 16; i++)
    {
        uint32_t byte_index = bitpos >> 3;
        uint8_t bit_offset = bitpos & 0x07;
        uint32_t v = payload[byte_index] | ((uint32_t)payload[byte_index + 1] << 8) | ((uint32_t)payload[byte_index + 2] << 16);
        v >>= bit_offset;
        ch[i] = (uint16_t)(v & 0x07FF);
        bitpos += 11;
    }

    crsf_rc_data_t data;
    data.timestamp = rt_tick_get();

    /* Map to sticks */
    data.roll  = ((int32_t)ch[0] - 992) * 0.5f;   /* CRSF mid around 992 */
    data.pitch = ((int32_t)ch[1] - 992) * 0.5f;
    data.thrust = (float)(ch[2] - 172) * (1000.0f / 1639.0f);
    data.yaw   = ((int32_t)ch[3] - 992) * 0.5f;

    data.arm_status = ch[4] > 1200;
    data.ctrl_mode  = ch[5] > 1200;

    rt_mutex_take(&g_crsf_lock, RT_WAITING_FOREVER);
    g_crsf_data = data;
    rt_mutex_release(&g_crsf_lock);
}

static rt_err_t crsf_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    while (size--)
    {
        uint8_t b = 0;
        if (rt_device_read(dev, 0, &b, 1) != 1) break;

        if (rx_len == 0)
        {
            if (b != CRSF_SYNC_BYTE) continue;
            rx_buf[rx_len++] = b;
            continue;
        }

        rx_buf[rx_len++] = b;

        /* We need at least header(2B), type(1B), payload, crc(1B) */
        if (rx_len >= 3)
        {
            uint8_t frame_len = rx_buf[1];
            if (rx_len >= (size_t)(frame_len + 2))
            {
                /* frame structure: [0]=sync, [1]=len, [2]=type, [3..]=payload, [end]=crc */
                uint8_t type = rx_buf[2];
                uint8_t crc_calc = 0; /* simple sum over [2..end-1] per CRSF v3 */
                for (int i = 2; i < 2 + frame_len - 1; i++) crc_calc += rx_buf[i];
                uint8_t crc_rx = rx_buf[2 + frame_len - 1];

                if (crc_calc == crc_rx)
                {
                    if (type == CRSF_FRAME_TYPE_RC_CHANNELS)
                    {
                        /* payload is channels, length usually 22 bytes */
                        crsf_update_data_from_channels(&rx_buf[3], frame_len - 2);
                    }
                }

                rx_len = 0; /* ready for next frame */
            }
            else if (rx_len >= sizeof(rx_buf))
            {
                rx_len = 0; /* overflow recovery */
            }
        }
    }

    return RT_EOK;
}

int crsf_init(void)
{
    rt_err_t ret;
    char uart_name[RT_NAME_MAX];

    rt_mutex_init(&g_crsf_lock, "crsflk", RT_IPC_FLAG_FIFO);

    rt_strncpy(uart_name, RC_CRSF_UART_NAME, RT_NAME_MAX);
    crsf_uart = rt_device_find(uart_name);
    if (!crsf_uart)
    {
        rt_kprintf("[CRSF] find %s failed\n", uart_name);
        return -RT_ERROR;
    }

    ret = rt_device_open(crsf_uart, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    if (ret != RT_EOK)
    {
        rt_kprintf("[CRSF] open %s failed\n", uart_name);
        return ret;
    }

    ret = uart_config_by_device_name(uart_name, RC_CRSF_BAUDRATE, &crsf_uart_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[CRSF] get uart cfg failed\n");
        return ret;
    }

    crsf_uart_cfg.baud_rate = RC_CRSF_BAUDRATE;
    crsf_uart_cfg.data_bits = DATA_BITS_8;
    crsf_uart_cfg.stop_bits = STOP_BITS_1;
    crsf_uart_cfg.parity    = PARITY_NONE;

    ret = rt_device_control(crsf_uart, RT_DEVICE_CTRL_CONFIG, &crsf_uart_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[CRSF] config uart failed\n");
        return ret;
    }

    rt_device_set_rx_indicate(crsf_uart, crsf_uart_rx_ind);

    return RT_EOK;
}

crsf_rc_data_t crsf_get_data(void)
{
    crsf_rc_data_t out;
    rt_mutex_take(&g_crsf_lock, RT_WAITING_FOREVER);
    out = g_crsf_data;
    rt_mutex_release(&g_crsf_lock);
    return out;
}

#ifdef RC_USING_CRSF
INIT_DEVICE_EXPORT(crsf_init);
#endif


