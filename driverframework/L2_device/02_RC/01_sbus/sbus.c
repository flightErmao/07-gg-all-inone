#include <rtthread.h>
#include <rtdevice.h>
#include "sbusProtocol.h"
#include "uartConfig.h"
#include "rc.h"

#ifdef RC_SBUS_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

static sbus_decoder_t g_sbus_decoder_;
static rt_device_t sbus_uart_ = RT_NULL;

static void sbus_thread_entry(void* param);
static rt_err_t sbus_rc_control(rc_dev_t rc, int cmd, void* arg);
static rt_uint16_t sbus_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val);
static rt_err_t sbus_init_uart(const char* uart_name);
static rt_err_t sbus_register_rc(void);
static rt_err_t sbus_start_thread(void);
static rt_err_t sbus_uart_rx_ind(rt_device_t dev, rt_size_t size);

static const struct rc_ops sbus_rc_ops = {
    .rc_config = RT_NULL,
    .rc_control = sbus_rc_control,
    .rc_read = sbus_rc_read,
};

static struct rc_device sbus_rc_dev = {
    .config =
        {
            .protocol = RC_PROTOCOL_SBUS,
            .channel_num = 16,
            .sample_time = 0.02f,
            .rc_min_value = 1000,
            .rc_max_value = 2000,
        },
    .ops = &sbus_rc_ops,
};

static rt_err_t sbus_uart_rx_ind(rt_device_t dev, rt_size_t size) {
  rt_event_send(g_sbus_decoder_.sbus_data_received_event, EVENT_SBUS_DATA_RECEIVED);
#ifdef RC_SBUS_DEBUGPIN_EN
  DEBUG_PIN_DEBUG1_HIGH();
#endif
  return RT_EOK;
}

static int sbus_thread_auto_start(void) {
  rt_err_t ret;
  char uart_name[RT_NAME_MAX];

  rt_strncpy(uart_name, RC_SBUS_UART_NAME, RT_NAME_MAX);

  ret = sbus_decoder_init(&g_sbus_decoder_);
  if (ret != RT_EOK) {
    rt_kprintf("[SBUS] decoder init failed\n");
    return ret;
  }

  ret = sbus_init_uart(uart_name);
  if (ret != RT_EOK) return ret;

  ret = sbus_register_rc();
  if (ret != RT_EOK) return ret;

  ret = sbus_start_thread();
  if (ret != RT_EOK) return ret;

  return RT_EOK;
}

static rt_err_t sbus_init_uart(const char* uart_name) {
  rt_err_t ret;
  static struct serial_configure sbus_uart_cfg;

  sbus_uart_ = rt_device_find(uart_name);
  if (!sbus_uart_) {
    rt_kprintf("[SBUS] find %s failed\n", uart_name);
    return -RT_ERROR;
  }
  ret = rt_device_open(sbus_uart_, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (ret != RT_EOK) {
    rt_kprintf("[SBUS] open %s failed\n", uart_name);
    return ret;
  }
  ret = uart_config_by_device_name(uart_name, RC_SBUS_BAUDRATE, &sbus_uart_cfg);
  if (ret != RT_EOK) {
    rt_kprintf("[SBUS] get uart cfg failed\n");
    return ret;
  }
  ret = rt_device_control(sbus_uart_, RT_DEVICE_CTRL_CONFIG, &sbus_uart_cfg);
  if (ret != RT_EOK) {
    rt_kprintf("[SBUS] config uart failed\n");
    return ret;
  }
  rt_device_set_rx_indicate(sbus_uart_, sbus_uart_rx_ind);
  return RT_EOK;
}

static rt_err_t sbus_register_rc(void) {
  rt_err_t result = hal_rc_register(&sbus_rc_dev, "rc_sbus", RT_DEVICE_FLAG_RDWR, RT_NULL);
  if (result != RT_EOK) {
    rt_kprintf("[SBUS] rc register failed\n");
    return result;
  }
  return RT_EOK;
}

static rt_err_t sbus_start_thread(void) {
  static struct rt_thread sbus_thread;
  static rt_uint8_t sbus_thread_stack[THREAD_STACK_SIZE];

  rt_thread_init(&sbus_thread, "sbus_task", sbus_thread_entry, RT_NULL, sbus_thread_stack, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&sbus_thread);
  return RT_EOK;
}

static void sbus_thread_entry(void* param) {
  rt_uint32_t recved = 0;
  uint8_t read_buf[64];

  while (1) {
    if (rt_event_recv(g_sbus_decoder_.sbus_data_received_event, EVENT_SBUS_DATA_RECEIVED,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved) == RT_EOK) {
#ifdef RC_SBUS_DEBUGPIN_EN
      DEBUG_PIN_DEBUG1_LOW();
#endif
      while (1) {
        rt_size_t rx_len = rt_device_read(sbus_uart_, 0, read_buf, sizeof(read_buf));
        if (rx_len <= 0) break;
        sbus_input(&g_sbus_decoder_, read_buf, rx_len);
      }

      if (!sbus_islock(&g_sbus_decoder_)) {
        uint32_t available_len = rt_ringbuffer_data_len(g_sbus_decoder_.sbus_rb);
        if (available_len >= SBUS_FRAME_SIZE) {
          sbus_update(&g_sbus_decoder_);
        }
      }
    }
  }
}

static rt_err_t sbus_rc_control(rc_dev_t rc, int cmd, void* arg) {
  if (cmd == RC_CMD_CHECK_UPDATE && arg) {
    *(uint8_t*)arg = sbus_data_ready(&g_sbus_decoder_) ? 1 : 0;
    return RT_EOK;
  }
  return RT_EOK;
}

static rt_uint16_t sbus_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val) {
  rt_uint16_t written = 0;
  rt_uint32_t recved = 0;
  rt_err_t event_ret;

  event_ret = rt_event_recv(g_sbus_decoder_.sbus_data_ready_event, EVENT_SBUS_DATA_READY,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, SBUS_READ_TIMEOUT_MS, &recved);

  if (event_ret != RT_EOK) {
    for (int i = 0; i < rc->config.channel_num; i++) {
      if (chan_mask & (1 << i)) {
        chan_val[i] = SBUS_INVALID_CHANNEL_VALUE;
        written += 2;
      }
    }
    return written;
  }

  uint16_t temp_channels[MAX_SBUS_CHANNEL];

  sbus_lock(&g_sbus_decoder_);
  rt_memcpy(temp_channels, g_sbus_decoder_.sbus_val, sizeof(g_sbus_decoder_.sbus_val));
  sbus_data_clear(&g_sbus_decoder_);
  sbus_unlock(&g_sbus_decoder_);

  for (int i = 0; i < rc->config.channel_num; i++) {
    if (chan_mask & (1 << i)) {
      chan_val[i] = temp_channels[i];
      written += 2;
    }
  }

  return written;
}

#ifdef RC_USING_SBUS
INIT_APP_EXPORT(sbus_thread_auto_start);
#endif
