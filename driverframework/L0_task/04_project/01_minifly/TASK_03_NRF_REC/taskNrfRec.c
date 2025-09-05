#include <rtdevice.h>
#include <rtthread.h>
#include "taskNrfRec.h"
#ifdef PROJECT_MINIFLY_TASK_NRF_DEBUGPIN_EN
#include "debugPin.h"
#endif
#include "rtconfig.h"
#include "uartConfig.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define MSG_NUM 10
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)

#define LOCAL_RX_BUF_SIZE BSP_UART2_DMA_PING_BUFSIZE

#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF
#define ATKP_MAX_DATA_SIZE 128

typedef enum {
  waitForStartByte1,
  waitForStartByte2,
  waitForMsgID,
  waitForDataLength,
  waitForData,
  waitForChksum1
} rx_state_t;

rt_align(RT_ALIGN_SIZE) static rt_uint8_t taskNrfStack[THREAD_STACK_SIZE];
static struct rt_thread taskNrfTid;

static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue device_recv_mq_;

static rt_device_t nrf_dev = RT_NULL;

struct uart_rx_msg {
  rt_device_t dev;
  rt_size_t size;
};
static struct rt_messagequeue rx_mq;
static char rx_msg_pool[256];

static rx_state_t rxState;
static atkp_t rxPacket;

/* UART RX indicate callback: post message to queue */
static rt_err_t nrf_rx_indicate(rt_device_t dev, rt_size_t size) {
  struct uart_rx_msg msg;
  msg.dev = dev;
  msg.size = size;
  rt_err_t result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
  if (result == -RT_EFULL) {
    rt_kprintf("NRF rx message queue full!\n");
  }
#ifdef PROJECT_MINIFLY_TASK_NRF_DEBUGPIN_EN
  DEBUG_PIN_DEBUG0_HIGH();
#endif
  return result;
}

static void task_msg_init(void) {
  rt_err_t result;
  result = rt_mq_init(&device_recv_mq_, "nrf_mq", &msg_pool[0], sizeof(atkp_t), POOL_SIZE_BYTE, RT_IPC_FLAG_PRIO);
  if (result != RT_EOK) {
    rt_kprintf("nrf mq init ERR\n");
  } else {
    rt_kprintf("nrf mq init OK\n");
  }
}

void radiolinkTask(void* param) {
  rxState = waitForStartByte1;
  uint8_t dataIndex = 0;
  uint8_t cksum = 0;
  static uint8_t rx_buffer[LOCAL_RX_BUF_SIZE + 1];

  while (1) {
    struct uart_rx_msg msg;
    rt_memset(&msg, 0, sizeof(msg));
    if (rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) > 0) {
#ifdef PROJECT_MINIFLY_TASK_NRF_DEBUGPIN_EN
      DEBUG_PIN_DEBUG0_LOW();
#endif
      rt_size_t read_len = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
      if (read_len <= 0) {
        continue;
      }
      for (rt_size_t i = 0; i < read_len; i++) {
        uint8_t c = rx_buffer[i];
        switch (rxState) {
          case waitForStartByte1:
            rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
            cksum = c;
            break;
          case waitForStartByte2:
            rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
            cksum += c;
            break;
          case waitForMsgID:
            rxPacket.msgID = c;
            rxState = waitForDataLength;
            cksum += c;
            break;
          case waitForDataLength:
            if (c <= ATKP_MAX_DATA_SIZE) {
              rxPacket.dataLen = c;
              dataIndex = 0;
              rxState = (c > 0) ? waitForData : waitForChksum1;
              cksum += c;
            } else {
              rxState = waitForStartByte1;
            }
            break;
          case waitForData:
            rxPacket.data[dataIndex] = c;
            dataIndex++;
            cksum += c;
            if (dataIndex == rxPacket.dataLen) {
              rxState = waitForChksum1;
            }
            break;
          case waitForChksum1:
            if (cksum == c) {
#ifdef PROJECT_MINIFLY_TASK_NRF_DEBUGPIN_EN
              DEBUG_PIN_DEBUG1_TOGGLE();
#endif
              rt_mq_send(&device_recv_mq_, &rxPacket, sizeof(atkp_t));
            } else {
              rt_kprintf("nrf checksum error\n");
            }
            rxState = waitForStartByte1;
            break;
          default:
            rt_kprintf("nrf state error\n");
            rxState = waitForStartByte1;
            break;
        }
      }
    }
  }
}

static int taskNrfRecInit(void) {
  rt_device_t device = rt_device_find(PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
  if (device == RT_NULL) {
    rt_kprintf("nrf device not found: %s\n", PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
    return -1;
  }

  rt_err_t result = rt_device_open(device, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (result != RT_EOK) {
    rt_kprintf("nrf device open failed\n");
    return -1;
  }

  struct serial_configure config;
  rt_err_t config_ret = uart_config_by_device_name(PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT, PROJECT_MINIFLY_TASK_NRF_BAUD_RATE, &config);
  if (config_ret != RT_EOK) {
    rt_kprintf("Configure NRF UART parameters failed!\n");
    return -1;
  }
  rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config);

  rt_err_t mq_ret =
      rt_mq_init(&rx_mq, "nrf_rxmq", rx_msg_pool, sizeof(struct uart_rx_msg), sizeof(rx_msg_pool), RT_IPC_FLAG_FIFO);
  if (mq_ret != RT_EOK) {
    rt_kprintf("nrf rx mq init failed\n");
    return -1;
  }

  rt_device_set_rx_indicate(device, nrf_rx_indicate);
  nrf_dev = device;

  rt_kprintf("NRF DMA RX interrupt status: %s\n",
             (nrf_dev->open_flag & RT_DEVICE_FLAG_DMA_RX) ? "Enabled" : "Disabled");

  task_msg_init();

  rt_thread_init(&taskNrfTid, "L0_minifly_nrfRec", radiolinkTask, RT_NULL, taskNrfStack, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&taskNrfTid);

  rt_kprintf("nrf task started on %s, baud: %d\n", PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT,
             PROJECT_MINIFLY_TASK_NRF_BAUD_RATE);

  return 0;
}

rt_device_t getNrfDevice(void) { return nrf_dev; }
struct rt_messagequeue* getNrfRecvMq(void) { return &device_recv_mq_; }

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
INIT_APP_EXPORT(taskNrfRecInit);
#endif
