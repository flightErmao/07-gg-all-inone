#include <rtdevice.h>
#include <rtthread.h>
#include "taskUartTest.h"
#ifdef TASK_DEMO_02_UART_TEST_DEBUGPIN_EN
#include "debugPin.h"
#endif
#include "rtconfig.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

#define LOCAL_RX_BUF_SIZE BSP_UART1_DMA_PING_BUFSIZE

struct uart_rx_msg {
  rt_device_t dev;
  rt_size_t size;
};

static rt_device_t uart_test_dev = RT_NULL;
static struct rt_messagequeue rx_mq;
static char msg_pool[256];

/* UART RX indicate callback: post message to queue */
static rt_err_t uart_test_input(rt_device_t dev, rt_size_t size) {
  struct uart_rx_msg msg;
  rt_err_t result;
  msg.dev = dev;
  msg.size = size;
  result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
  if (result == -RT_EFULL) {
    rt_kprintf("UART test message queue full!\n");
  }
  DEBUG_PIN_DEBUG1_HIGH();
  return result;
}

void uartTestTask(void *param) {
  struct uart_rx_msg msg;
  rt_err_t result;
  rt_uint32_t rx_length;
  static char rx_buffer[LOCAL_RX_BUF_SIZE + 1];

  while (1) {
    rt_memset(&msg, 0, sizeof(msg));
    result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
    DEBUG_PIN_DEBUG1_LOW();
    if (result > 0) {
      rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
      if (rx_length > 0) {
        rx_buffer[rx_length] = '\0';
        rt_device_write(uart_test_dev, 0, rx_buffer, rx_length);
        rt_kprintf("UART RX: %d bytes received\n", rx_length);
      }
    }
  }
}

static int taskUartTestInit(void) {
  rt_err_t ret = RT_EOK;
  char uart_name[RT_NAME_MAX];

  rt_strncpy(uart_name, TASK_DEMO_02_UART_TEST_DEVICE_DEFAULT, RT_NAME_MAX);

  uart_test_dev = rt_device_find(uart_name);
  if (!uart_test_dev) {
    rt_kprintf("Find UART device %s failed!\n", uart_name);
    return -RT_ERROR;
  }

  ret = rt_mq_init(&rx_mq, "uart_test_mq", msg_pool, sizeof(struct uart_rx_msg), sizeof(msg_pool), RT_IPC_FLAG_FIFO);
  if (ret != RT_EOK) {
    rt_kprintf("Initialize message queue failed!\n");
    return ret;
  }

  ret = rt_device_open(uart_test_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (ret != RT_EOK) {
    rt_kprintf("Open UART device failed!\n");
    return ret;
  }

  struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
  config.baud_rate = TASK_DEMO_02_UART_TEST_BAUD_RATE;
  config.rx_bufsz = BSP_UART1_RX_BUFSIZE;
  config.tx_bufsz = BSP_UART1_TX_BUFSIZE;
  config.dma_ping_bufsz = BSP_UART1_DMA_PING_BUFSIZE;
  ret = rt_device_control(uart_test_dev, RT_DEVICE_CTRL_CONFIG, &config);
  if (ret != RT_EOK) {
    rt_kprintf("Configure UART parameters failed!\n");
    return ret;
  }

  rt_device_set_rx_indicate(uart_test_dev, uart_test_input);

  rt_kprintf("DMA RX interrupt status: %s\n",
             (uart_test_dev->open_flag & RT_DEVICE_FLAG_DMA_RX) ? "Enabled" : "Disabled");

  rt_thread_t thread =
      rt_thread_create("uart_test", uartTestTask, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (thread != RT_NULL) {
    rt_thread_startup(thread);
    rt_kprintf("UART test task started on %s, baud: %d\n", uart_name, TASK_DEMO_02_UART_TEST_BAUD_RATE);
    rt_kprintf("Using DMA RX mode with message queue (DMA interrupts disabled)\n");
    rt_kprintf("Data reception handled via UART IDLE interrupt + message queue\n");
  } else {
    rt_kprintf("Create UART test thread failed!\n");
    ret = -RT_ERROR;
  }

  return ret;
}

#ifdef TASK_DEMO_02_UART_TEST_EN
INIT_APP_EXPORT(taskUartTestInit);
#endif