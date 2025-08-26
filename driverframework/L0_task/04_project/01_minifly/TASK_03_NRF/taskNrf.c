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
#define NRF_EVENT_RX (1 << 0)

/* Local RX buffer size equals UART ring buffer size */
#if defined(PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT) && (0)
/* Placeholder if multi-UART mapping is required */
#endif
#define LOCAL_RX_BUF_SIZE BSP_UART2_RX_BUFSIZE

rt_align(RT_ALIGN_SIZE) static rt_uint8_t taskNrfStack[THREAD_STACK_SIZE];
static struct rt_thread taskNrfTid;

static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue device_recv_mq_;

static struct rt_event nrf_event;
static uint8_t local_rx_buf[LOCAL_RX_BUF_SIZE];
static volatile rt_size_t local_rx_len = 0;
static rt_device_t nrf_dev = RT_NULL;

static rx_state_t rxState;
static atkp_t rxPacket;

/* UART RX indicate callback: read ring buffer into local buffer and notify */
static rt_err_t nrf_rx_indicate(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    /* Notify RX event; real draining occurs in thread context */
    local_rx_len = size;
    rt_event_send(&nrf_event, NRF_EVENT_RX);
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
        rt_uint32_t recved = 0;
        /* Wait for RX event */
        if (rt_event_recv(&nrf_event, NRF_EVENT_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &recved) == RT_EOK) {
            /* Drain ring buffer in chunks until empty */
            while (1) {
                rt_size_t len = rt_device_read(nrf_dev, 0, local_rx_buf, LOCAL_RX_BUF_SIZE);
                if (len == 0) break;
                for (rt_size_t i = 0; i < len; i++) {
                    uint8_t c = local_rx_buf[i];
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
}

static int taskNrfInit(void) {
  rt_device_t device = rt_device_find(PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
  if (device == RT_NULL) {
    rt_kprintf("nrf device not found: %s\n", PROJECT_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
    return -1;
  }

    rt_err_t result = rt_device_open(device, RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK) {
        rt_kprintf("nrf device open failed\n");
        return -1;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = PROJECT_MINIFLY_TASK_NRF_BAUD_RATE;
    config.rx_bufsz = BSP_UART2_RX_BUFSIZE;
    config.tx_bufsz = BSP_UART2_TX_BUFSIZE;
    rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config);

    /* Prefer DMA RX if available to avoid overrun under burst traffic */
    rt_device_control(device, RT_DEVICE_CTRL_CONFIG, (void *)RT_DEVICE_FLAG_DMA_RX);

    /* Init RX event and register UART RX callback */
    rt_event_init(&nrf_event, "nrf_evt", RT_IPC_FLAG_FIFO);
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
