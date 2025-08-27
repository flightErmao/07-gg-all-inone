#include <rtdevice.h>
#include <rtthread.h>
#include "taskUartTest.h"
#ifdef TASK_DEMO_02_UART_TEST_DEBUGPIN_EN
#include "debugPin.h"
#endif
#include "rtconfig.h"

/*
 * UART 测试任务 - 使用 DMA 接收但禁用 DMA 中断
 *
 * 本任务通过以下方式优化 UART 通信：
 * 1. 使用 DMA 接收数据，提高接收效率
 * 2. 禁用 DMA 中断回调，避免 HAL_UART_RxCpltCallback 和 HAL_UART_RxHalfCpltCallback
 * 3. 通过 UART IDLE 中断和消息队列机制处理接收到的数据
 * 4. 使用阻塞式发送，确保数据完整性
 */

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

// Local RX buffer size equals UART ring buffer size
#define LOCAL_RX_BUF_SIZE BSP_UART1_DMA_PING_BUFSIZE

// UART receive message structure
struct uart_rx_msg {
  rt_device_t dev;
  rt_size_t size;
};

// UART device handle
static rt_device_t uart_test_dev = RT_NULL;

// Message queue control block
static struct rt_messagequeue rx_mq;

// Message pool for queue
static char msg_pool[256];

// Receive data callback function
static rt_err_t uart_test_input(rt_device_t dev, rt_size_t size) {
  struct uart_rx_msg msg;
  rt_err_t result;

  msg.dev = dev;
  msg.size = size;

  result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
  if (result == -RT_EFULL) {
    /* Message queue full */
    rt_kprintf("UART test message queue full!\n");
  }
  DEBUG_PIN_DEBUG1_HIGH();
  return result;
}

// // 管理 DMA 中断开关的函数
// static rt_err_t uart_dma_interrupt_control(rt_device_t dev, rt_bool_t enable)
// {
//     rt_err_t ret;

//     if (enable)
//     {
//         /* 启用 DMA 中断 */
//         ret = rt_device_control(dev, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_DMA_RX);
//         if (ret == RT_EOK)
//         {
//             rt_kprintf("DMA RX interrupt enabled\n");
//         }
//         else
//         {
//             rt_kprintf("Enable DMA RX interrupt failed: %d\n", ret);
//         }
//     }
//     else
//     {
//         /* 禁用 DMA 中断 */
//         ret = rt_device_control(dev, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_DMA_RX);
//         if (ret == RT_EOK)
//         {
//             rt_kprintf("DMA RX interrupt disabled\n");
//         }
//         else
//         {
//             rt_kprintf("Disable DMA RX interrupt failed: %d\n", ret);
//         }
//     }

//     return ret;
// }

// Main UART test task
void uartTestTask(void *param) {
  struct uart_rx_msg msg;
  rt_err_t result;
  rt_uint32_t rx_length;
  static char rx_buffer[LOCAL_RX_BUF_SIZE + 1];

  while (1) {
    rt_memset(&msg, 0, sizeof(msg));

    /* Read message from message queue */
    result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
    DEBUG_PIN_DEBUG1_LOW();
    if (result > 0) {
      /* Read data from UART */
      rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
      if (rx_length > 0) {
        rx_buffer[rx_length] = '\0';

        rt_device_write(uart_test_dev, 0, rx_buffer, rx_length);
        /* Print received data length */
        rt_kprintf("UART RX: %d bytes received\n", rx_length);

        /* Print received data directly */
        // rt_kprintf("%s\n", rx_buffer);
      }
    }
  }
}

// Initialize UART test task
static int taskUartTestInit(void) {
  rt_err_t ret = RT_EOK;
  char uart_name[RT_NAME_MAX];

  /* Get UART device name from configuration */
  rt_strncpy(uart_name, TASK_DEMO_02_UART_TEST_DEVICE_DEFAULT, RT_NAME_MAX);

  /* Find UART device */
  uart_test_dev = rt_device_find(uart_name);
  if (!uart_test_dev) {
    rt_kprintf("Find UART device %s failed!\n", uart_name);
    return -RT_ERROR;
  }

  /* Initialize message queue */
  ret = rt_mq_init(&rx_mq, "uart_test_mq", msg_pool, /* Message buffer */
                   sizeof(struct uart_rx_msg),       /* Max message length */
                   sizeof(msg_pool),                 /* Buffer size */
                   RT_IPC_FLAG_FIFO);                /* FIFO mode */
  if (ret != RT_EOK) {
    rt_kprintf("Initialize message queue failed!\n");
    return ret;
  }

  /* Open UART device with DMA RX and blocking TX */
  ret = rt_device_open(uart_test_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (ret != RT_EOK) {
    rt_kprintf("Open UART device failed!\n");
    return ret;
  }

  /* Configure UART parameters */
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

  /* Set receive callback function */
  rt_device_set_rx_indicate(uart_test_dev, uart_test_input);

  // /* 关闭 UART RX DMA 的半满/全满中断，避免使用 HAL_UART_RxCpltCallback 和 HAL_UART_RxHalfCpltCallback */
  //   ret = rt_device_control(uart_test_dev, RT_DEVICE_CTRL_UART_DMA_RX_DISABLE_HALF_FULL_INT, RT_NULL);
  //   if (ret != RT_EOK) {
  //     rt_kprintf("Failed to disable UART RX DMA half/full interrupts!\n");
  //     return ret;
  //   }

  /* 验证 DMA 中断状态 */
  rt_kprintf("DMA RX interrupt status: %s\n",
             (uart_test_dev->open_flag & RT_DEVICE_FLAG_DMA_RX) ? "Enabled" : "Disabled");

  /* Create and start UART test thread */
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