#include <rtdevice.h>
#include <rtthread.h>
#include "taskNrf.h"
#ifdef PROJECT_MINIFLY_TASK_NRF_DEBUGPIN_EN
#include "debugPin.h"
#endif
#include "rtconfig.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define MSG_NUM 10
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)

/* Event flag for RX notification */
// #define NRF_EVENT_RX (1 << 0)

/* Local RX buffer size equals UART ring buffer size */
#if defined(PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT) && (0)
/* Placeholder if multi-UART mapping is required */
#endif
#define LOCAL_RX_BUF_SIZE BSP_UART2_RX_BUFSIZE

rt_align(RT_ALIGN_SIZE) static rt_uint8_t taskNrfStack[THREAD_STACK_SIZE];
static struct rt_thread taskNrfTid;

static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue device_recv_mq_;

// static struct rt_event nrf_event;
// static uint8_t local_rx_buf[LOCAL_RX_BUF_SIZE];
// static volatile rt_size_t local_rx_len = 0;
static rt_device_t nrf_dev = RT_NULL;

/* Minimal ring buffer for ISR->thread data passing (overwrite when full) */
static uint8_t nrf_rb_pool[LOCAL_RX_BUF_SIZE];
static volatile rt_size_t nrf_rb_head = 0; /* read index */
static volatile rt_size_t nrf_rb_tail = 0; /* write index */

static inline void nrf_rb_init(void) {
  nrf_rb_head = 0;
  nrf_rb_tail = 0;
}

static inline void nrf_rb_putchar(uint8_t ch) {
  rt_size_t next_tail = (nrf_rb_tail + 1) % LOCAL_RX_BUF_SIZE;
  /* overwrite oldest when full */
  if (next_tail == nrf_rb_head) {
    nrf_rb_head = (nrf_rb_head + 1) % LOCAL_RX_BUF_SIZE;
  }
  nrf_rb_pool[nrf_rb_tail] = ch;
  nrf_rb_tail = next_tail;
}

static inline int nrf_rb_getchar(uint8_t *ch) {
  if (nrf_rb_head == nrf_rb_tail) {
    return 0; /* empty */
  }
  *ch = nrf_rb_pool[nrf_rb_head];
  nrf_rb_head = (nrf_rb_head + 1) % LOCAL_RX_BUF_SIZE;
  return 1;
}

static struct rt_semaphore nrf_rx_sem;

static rx_state_t rxState;
static atkp_t rxPacket;

/* small ISR temp buffer for chunked draining */
static uint8_t nrf_isr_tmp[128];

/* UART RX indicate callback: read bytes and push to ring buffer, then notify by semaphore */
static rt_err_t nrf_rx_indicate(rt_device_t dev, rt_size_t size)
{
  /* size is a hint from driver: available bytes this time (DMADONE/IDLE). Drain in chunks. */
  rt_size_t remain = size;
  while (1) {
    rt_size_t to_read =
        (remain > 0 && remain < (rt_size_t)sizeof(nrf_isr_tmp)) ? remain : (rt_size_t)sizeof(nrf_isr_tmp);
    rt_size_t len = rt_device_read(dev, 0, nrf_isr_tmp, to_read);
    if (len <= 0) {
      break;
    }
    for (rt_size_t i = 0; i < len; i++) {
      nrf_rb_putchar(nrf_isr_tmp[i]);
    }
    if (remain > 0) {
      remain -= len;
      if (remain == 0) break;
    }
    /* If remain == 0 (unknown), keep draining until empty (len==0). */
  }
  /* Notify RX by releasing semaphore */
  rt_sem_release(&nrf_rx_sem);
  return RT_EOK;
}

static void task_msg_init(void) {
    rt_err_t result;

    result = rt_mq_init(&device_recv_mq_, "nrf_mq", &msg_pool[0],
                        sizeof(atkp_t),
                        POOL_SIZE_BYTE,
                        RT_IPC_FLAG_PRIO);
    if (result != RT_EOK) {
        rt_kprintf("nrf mq init ERR\n");
    } else {
        rt_kprintf("nrf mq init OK\n");
    }
}

/* Deprecated byte-wise polling helper removed */

void radiolinkTask(void *param) {
    rxState = waitForStartByte1;
    uint8_t dataIndex = 0;
    uint8_t cksum = 0;

    while (1) {
#ifdef PROJECT_MINIFLY_TASK_NRF_DEBUGPIN_EN
        DEBUG_PIN_DEBUG0_TOGGLE();
#endif
        /* Wait for RX semaphore */
        if (rt_sem_take(&nrf_rx_sem, RT_WAITING_FOREVER) == RT_EOK) {
          /* Drain ring buffer until empty */
          uint8_t c;
          while (nrf_rb_getchar(&c) == 1) {
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

static int taskNrfInit(void) {
  rt_device_t device = rt_device_find(PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
  if (device == RT_NULL) {
    rt_kprintf("nrf device not found: %s\n", PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
    return -1;
  }

  /* Open with DMA RX to avoid RX FIFO overrun at high baud rate */
  rt_err_t result = rt_device_open(device, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
  if (result != RT_EOK) {
    rt_kprintf("nrf device open failed\n");
    return -1;
  }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = PROJECT_MINIFLY_TASK_NRF_BAUD_RATE;
    config.rx_bufsz = BSP_UART2_RX_BUFSIZE;
    config.tx_bufsz = BSP_UART2_TX_BUFSIZE;
    rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config);

    /* Init local ring buffer and RX semaphore, then register UART RX callback */
    nrf_rb_init();
    rt_sem_init(&nrf_rx_sem, "nrf_sem", 0, RT_IPC_FLAG_FIFO);
    rt_device_set_rx_indicate(device, nrf_rx_indicate);
    nrf_dev = device;

    task_msg_init();

    rt_thread_init(&taskNrfTid, "task_nrf", radiolinkTask, RT_NULL, taskNrfStack,
                   THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&taskNrfTid);

    rt_kprintf("nrf task started on %s, baud: %d\n", PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT,
               PROJECT_MINIFLY_TASK_NRF_BAUD_RATE);

    return 0;
}

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
INIT_APP_EXPORT(taskNrfInit);
#endif
