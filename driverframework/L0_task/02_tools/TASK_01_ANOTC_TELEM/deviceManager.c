#include <rtdevice.h>
#include <string.h>
#include <stdbool.h>
#include "deviceManager.h"
#include "uartConfig.h"

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
#include "taskNrfRec.h"
#endif

/* Constants */
#define ATKP_MAX_DATA_SIZE 128
#define ATKP_PROTOCOL_HEAD_SIZE 6
#define ATKP_ANOTC_TELEM_BUF_SIZE (ATKP_MAX_DATA_SIZE + ATKP_PROTOCOL_HEAD_SIZE)
#define LOCAL_RX_BUF_SIZE 256

#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define THREAD_PRIORITY 8
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

#define PARSED_DATA_MSG_NUM 10

/* ATKP protocol parsing definitions */
typedef enum {
  waitForStartByte1,
  waitForStartByte2,
  waitForMsgID,
  waitForDataLength,
  waitForData,
  waitForChksum1
} atkp_rx_state_t;

/* Global variables */
rt_device_t dev_anotc_telem_ = RT_NULL;

/* UART thread variables */
rt_align(RT_ALIGN_SIZE) static rt_uint8_t uart_handler_stack_[THREAD_STACK_SIZE];
static struct rt_thread uart_handler_tid_;

/* UART message queue variables */
static struct rt_messagequeue uart_rx_mq_;
static char uart_rx_msg_pool_[256];
static rt_device_t uart_rx_device_ = RT_NULL;

/* Parsed data message queue variables */
static rt_uint8_t parsed_data_pool_[sizeof(atkp_t) * PARSED_DATA_MSG_NUM];
static struct rt_messagequeue parsed_data_mq_;

/* ATKP protocol parsing variables */
static atkp_rx_state_t atkp_rx_state_;
static atkp_t atkp_rx_packet_;
static uint8_t atkp_data_index_;
static uint8_t atkp_checksum_;

/* ATKP protocol parsing functions */
static rt_err_t atkp_parse_byte(uint8_t c) {
  switch (atkp_rx_state_) {
    case waitForStartByte1:
      atkp_rx_state_ = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
      atkp_checksum_ = c;
      break;
    case waitForStartByte2:
      atkp_rx_state_ = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
      atkp_checksum_ += c;
      break;
    case waitForMsgID:
      atkp_rx_packet_.msgID = c;
      atkp_rx_state_ = waitForDataLength;
      atkp_checksum_ += c;
      break;
    case waitForDataLength:
      if (c <= ATKP_MAX_DATA_SIZE) {
        atkp_rx_packet_.dataLen = c;
        atkp_data_index_ = 0;
        atkp_rx_state_ = (c > 0) ? waitForData : waitForChksum1;
        atkp_checksum_ += c;
      } else {
        atkp_rx_state_ = waitForStartByte1;
      }
      break;
    case waitForData:
      atkp_rx_packet_.data[atkp_data_index_] = c;
      atkp_data_index_++;
      atkp_checksum_ += c;
      if (atkp_data_index_ == atkp_rx_packet_.dataLen) {
        atkp_rx_state_ = waitForChksum1;
      }
      break;
    case waitForChksum1:
      if (atkp_checksum_ == c) {
        rt_mq_send(&parsed_data_mq_, &atkp_rx_packet_, sizeof(atkp_t));
        return RT_EOK;
      } else {
        rt_kprintf("ATKP checksum error\n");
      }
      atkp_rx_state_ = waitForStartByte1;
      break;
    default:
      rt_kprintf("ATKP state error\n");
      atkp_rx_state_ = waitForStartByte1;
      break;
  }
  return RT_ERROR;
}

/* ATKP packet send functions */
void anotcDeviceSendDirect(atkp_t* p) {
  int dataSize;
  uint8_t cksum = 0;
  static uint8_t sendBuffer[ATKP_ANOTC_TELEM_BUF_SIZE];

  RT_ASSERT((p->dataLen <= ATKP_MAX_DATA_SIZE) == true);

  sendBuffer[0] = UP_BYTE1;
  sendBuffer[1] = UP_BYTE2;
  sendBuffer[2] = p->msgID;
  sendBuffer[3] = p->dataLen;

  memcpy(&sendBuffer[4], p->data, p->dataLen);
  dataSize = p->dataLen + 5;
  for (int i = 0; i < dataSize - 1; i++) {
    cksum += sendBuffer[i];
  }
  sendBuffer[dataSize - 1] = cksum;
  if (dev_anotc_telem_ != RT_NULL) {
    rt_device_write(dev_anotc_telem_, 0, sendBuffer, dataSize);
  } else {
#ifdef PROJECT_MINIFLY_TASK_NRF_EN
    dev_anotc_telem_ = getNrfDevice();
#else
    dev_anotc_telem_ = uart_rx_device_;
#endif
  }
}

/* UART data handler thread */
static void uart_data_handler_task(void* param) {
  uart_rx_msg_t msg;
  static rt_uint8_t rx_buffer[LOCAL_RX_BUF_SIZE + 1];

  while (1) {
    rt_memset(&msg, 0, sizeof(msg));
    if (rt_mq_recv(&uart_rx_mq_, &msg, sizeof(msg), RT_WAITING_FOREVER) > 0) {
      rt_size_t read_len = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
      if (read_len <= 0) {
        continue;
      }

      for (rt_size_t i = 0; i < read_len; i++) {
        atkp_parse_byte(rx_buffer[i]);
      }
    }
  }
}

/* UART callback functions */
static rt_err_t uart_rx_indicate(rt_device_t dev, rt_size_t size) {
  uart_rx_msg_t msg;
  msg.dev = dev;
  msg.size = size;
  rt_err_t result = rt_mq_send(&uart_rx_mq_, &msg, sizeof(msg));
  if (result == -RT_EFULL) {
    rt_kprintf("UART rx message queue full!\n");
  }
  return result;
}

/* System initialization functions */
static rt_err_t system_mq_init(void) {
  rt_err_t result;

  result = rt_mq_init(&parsed_data_mq_, "parsed_data_mq", parsed_data_pool_, sizeof(atkp_t),
                      sizeof(atkp_t) * PARSED_DATA_MSG_NUM, RT_IPC_FLAG_PRIO);
  if (result != RT_EOK) {
    rt_kprintf("Parsed data mq init failed\n");
    return result;
  }

  result = rt_mq_init(&uart_rx_mq_, "uart_rxmq", uart_rx_msg_pool_, sizeof(uart_rx_msg_t), sizeof(uart_rx_msg_pool_),
                      RT_IPC_FLAG_FIFO);
  if (result != RT_EOK) {
    rt_kprintf("UART rx mq init failed\n");
    return result;
  }

  rt_kprintf("System message queues initialized\n");
  return RT_EOK;
}

static rt_err_t uart_device_init(const char* device_name, rt_uint32_t baud_rate) {
  if (device_name == RT_NULL) {
    return -RT_ERROR;
  }

  rt_device_t device = rt_device_find(device_name);
  if (device == RT_NULL) {
    rt_kprintf("UART device not found: %s\n", device_name);
    return -RT_ERROR;
  }

  rt_err_t result = rt_device_open(device, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (result != RT_EOK) {
    rt_kprintf("UART device open failed\n");
    return -RT_ERROR;
  }

  struct serial_configure config;
  rt_err_t config_ret = uart_config_by_device_name(device_name, baud_rate, &config);
  if (config_ret != RT_EOK) {
    rt_kprintf("Configure UART parameters failed!\n");
    rt_device_close(device);
    return -RT_ERROR;
  }
  rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config);

  rt_device_set_rx_indicate(device, uart_rx_indicate);
  uart_rx_device_ = device;

  rt_kprintf("UART %s configured, baud: %d\n", device_name, baud_rate);
  return RT_EOK;
}

static rt_err_t uart_thread_init(void) {
  atkp_rx_state_ = waitForStartByte1;
  atkp_data_index_ = 0;
  atkp_checksum_ = 0;

  rt_thread_init(&uart_handler_tid_, "antoAnl", uart_data_handler_task, RT_NULL, uart_handler_stack_, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&uart_handler_tid_);

  rt_kprintf("UART data handler thread started\n");
  return RT_EOK;
}

/* Device management functions */
rt_err_t uartDevAnotcInit(char* device_name) {
  if (dev_anotc_telem_ && (dev_anotc_telem_->open_flag & RT_DEVICE_OFLAG_OPEN)) {
    return RT_EOK;
  }

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
  dev_anotc_telem_ = getNrfDevice();
  return RT_EOK;
#endif

  if (!strncmp(device_name, "uart", 4)) {
    if (system_mq_init() != RT_EOK) {
      rt_kprintf("System message queue init failed\n");
      return RT_ERROR;
    }

    if (uart_device_init(device_name, TASK_TOOL_01_ANOTC_TELEM_BAUD_RATE) != RT_EOK) {
      rt_kprintf("UART device init failed\n");
      return RT_ERROR;
    }

    if (uart_thread_init() != RT_EOK) {
      rt_kprintf("UART thread init failed\n");
      return RT_ERROR;
    }

    dev_anotc_telem_ = uart_rx_device_;
    return RT_EOK;
  }

  rt_device_t new_dev = rt_device_find(device_name);
  if (new_dev == NULL) {
    return RT_ERROR;
  }

  if (rt_device_open(new_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING) != RT_EOK) {
    return RT_ERROR;
  }

  dev_anotc_telem_ = new_dev;
  return RT_EOK;
}

struct rt_messagequeue* getAnotcRecMq(void) { return &parsed_data_mq_; }

// struct rt_messagequeue* getAnotcRecMq(void) { return &uart_rx_mq_; }