#include "crsf.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "rtconfig.h"
#include "uartConfig.h"
#include "crsfProtocol.h"
#include "rc.h"

#ifdef RC_CRSF_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

static crsf_decoder_t g_crsf_decoder_;
static rt_device_t crsf_uart_ = RT_NULL;

static void crsf_thread_entry(void* param);
static rt_err_t crsf_rc_control(rc_dev_t rc, int cmd, void* arg);
static rt_uint16_t crsf_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val);
static rt_err_t crsf_init_uart(const char* uart_name);
static rt_err_t crsf_register_rc(void);
static rt_err_t crsf_start_thread(void);
static rt_err_t crsf_uart_rx_ind(rt_device_t dev, rt_size_t size);

static const struct rc_ops crsf_rc_ops = {
    .rc_config = RT_NULL,
    .rc_control = crsf_rc_control,
    .rc_read = crsf_rc_read,
};

static struct rc_device crsf_rc_dev = {
    .config =
        {
            .protocol = RC_PROTOCOL_CRSF,
            .channel_num = 16,
            .sample_time = 0.011f,
            .rc_min_value = 1000,
            .rc_max_value = 2000,
        },
    .ops = &crsf_rc_ops,
};

static rt_err_t crsf_uart_rx_ind(rt_device_t dev, rt_size_t size) {
  rt_event_send(g_crsf_decoder_.crsf_data_received_event, EVENT_CRSF_DATA_RECEIVED);
#ifdef RC_CRSF_DEBUGPIN_EN
  DEBUG_PIN_DEBUG0_HIGH();
#endif
  return RT_EOK;
}

static int crsf_thread_auto_start(void) {
  rt_err_t ret;
  char uart_name[RT_NAME_MAX];

  rt_strncpy(uart_name, RC_CRSF_UART_NAME, RT_NAME_MAX);

  ret = crsf_decoder_init(&g_crsf_decoder_);
  if (ret != RT_EOK) {
    rt_kprintf("[CRSF] decoder init failed\n");
    return ret;
  }

  ret = crsf_init_uart(uart_name);
  if (ret != RT_EOK) return ret;

  ret = crsf_register_rc();
  if (ret != RT_EOK) return ret;

  ret = crsf_start_thread();
  if (ret != RT_EOK) return ret;

  return RT_EOK;
}

static rt_err_t crsf_init_uart(const char* uart_name) {
  rt_err_t ret;
  static struct serial_configure crsf_uart_cfg;

  crsf_uart_ = rt_device_find(uart_name);
  if (!crsf_uart_) {
    rt_kprintf("[CRSF] find %s failed\n", uart_name);
    return -RT_ERROR;
  }
  ret = rt_device_open(crsf_uart_, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (ret != RT_EOK) {
    rt_kprintf("[CRSF] open %s failed\n", uart_name);
    return ret;
  }
  ret = uart_config_by_device_name(uart_name, RC_CRSF_BAUDRATE, &crsf_uart_cfg);
  if (ret != RT_EOK) {
    rt_kprintf("[CRSF] get uart cfg failed\n");
    return ret;
  }
  crsf_uart_cfg.baud_rate = RC_CRSF_BAUDRATE;
  crsf_uart_cfg.data_bits = DATA_BITS_8;
  crsf_uart_cfg.stop_bits = STOP_BITS_1;
  crsf_uart_cfg.parity = PARITY_NONE;
  ret = rt_device_control(crsf_uart_, RT_DEVICE_CTRL_CONFIG, &crsf_uart_cfg);
  if (ret != RT_EOK) {
    rt_kprintf("[CRSF] config uart failed\n");
    return ret;
  }
  /* disable UART RX DMA half/full transfer interrupts to reduce ISR load */
  ret = rt_device_control(crsf_uart_, RT_DEVICE_CTRL_UART_DMA_RX_DISABLE_HALF_FULL_INT, RT_NULL);
  if (ret != RT_EOK) {
    rt_kprintf("[CRSF] disable uart dma rx half/full int failed\n");
    return ret;
  }
  rt_device_set_rx_indicate(crsf_uart_, crsf_uart_rx_ind);
  return RT_EOK;
}

static rt_err_t crsf_register_rc(void) {
  rt_err_t result = hal_rc_register(&crsf_rc_dev, "crsf", RT_DEVICE_FLAG_RDWR, RT_NULL);
  if (result != RT_EOK) {
    rt_kprintf("[CRSF] rc register failed\n");
    return result;
  }
  return RT_EOK;
}

static rt_err_t crsf_start_thread(void) {
  static struct rt_thread crsf_thread;
  static rt_uint8_t crsf_thread_stack[THREAD_STACK_SIZE];

  rt_thread_init(&crsf_thread, "crsf", crsf_thread_entry, RT_NULL, crsf_thread_stack, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&crsf_thread);
  return RT_EOK;
}

static void crsf_thread_entry(void* param) {
  rt_uint32_t recved = 0;
  uint8_t read_buf[64];

  while (1) {
    if (rt_event_recv(g_crsf_decoder_.crsf_data_received_event, EVENT_CRSF_DATA_RECEIVED,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved) == RT_EOK) {
#ifdef RC_CRSF_DEBUGPIN_EN
      DEBUG_PIN_DEBUG0_LOW();
#endif
      rt_size_t rx_len = rt_device_read(crsf_uart_, 0, read_buf, sizeof(read_buf));
      if (rx_len <= 0) {
        continue;
      }
      crsf_input(&g_crsf_decoder_, read_buf, rx_len);

      while (!crsf_islock(&g_crsf_decoder_)) {
        if (!crsf_update(&g_crsf_decoder_)) {
          break;
        }
      }
    }
  }
}

static rt_err_t crsf_rc_control(rc_dev_t rc, int cmd, void* arg) {
  if (cmd == RC_CMD_CHECK_UPDATE && arg) {
    *(uint8_t*)arg = crsf_data_ready(&g_crsf_decoder_) ? 1 : 0;
    return RT_EOK;
  }
  return RT_EOK;
}

static rt_uint16_t crsf_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val) {
  rt_uint16_t written = 0;
  rt_uint32_t recved = 0;
  rt_err_t event_ret;

  event_ret = rt_event_recv(g_crsf_decoder_.crsf_data_ready_event, EVENT_CRSF_DATA_READY,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, CRSF_READ_TIMEOUT_MS, &recved);

  if (event_ret != RT_EOK) {
    for (int i = 0; i < rc->config.channel_num; i++) {
      if (chan_mask & (1 << i)) {
        chan_val[i] = CRSF_INVALID_CHANNEL_VALUE;
      }
    }
    return 0;
  }
#ifdef RC_CRSF_DEBUGPIN_EN
  DEBUG_PIN_DEBUG1_TOGGLE();
#endif
  uint16_t temp_channels[MAX_CRSF_CHANNEL];

  crsf_lock(&g_crsf_decoder_);
  rt_memcpy(temp_channels, g_crsf_decoder_.crsf_val, sizeof(g_crsf_decoder_.crsf_val));
  crsf_data_clear(&g_crsf_decoder_);
  crsf_unlock(&g_crsf_decoder_);

  for (int i = 0; i < rc->config.channel_num; i++) {
    if (chan_mask & (1 << i)) {
      chan_val[i] = temp_channels[i];
      written += 2;
    }
  }

  return written;
}

#ifdef RC_USING_CRSF
INIT_COMPONENT_EXPORT(crsf_thread_auto_start);
#endif


