#include <rtdevice.h>
#include <rtthread.h>
#include "taskNrf.h"
#include "deviceManager.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define MSG_NUM 10
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)

rt_align(RT_ALIGN_SIZE) static rt_uint8_t taskNrfStack[THREAD_STACK_SIZE];
static struct rt_thread taskNrfTid;

static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue device_recv_mq_;

static rx_state_t rxState;
static atkp_t rxPacket;

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

static int uartslkGetDataWithTimout(uint8_t *c) {
    rt_device_t device = rt_device_find(TASK_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
    if (device == RT_NULL) {
        return 0;
    }
    
    if (rt_device_read(device, 0, c, 1) == 1) {
        return 1;
    }
    
    return 0;
}

void radiolinkTask(void *param) {
    rxState = waitForStartByte1;
    
    uint8_t c;
    uint8_t dataIndex = 0;
    uint8_t cksum = 0;

    while(1) {
        if (uartslkGetDataWithTimout(&c)) {
            switch(rxState) {
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
        } else {
            rxState = waitForStartByte1;
        }
    }
}

static int taskNrfInit(void) {
    rt_device_t device = rt_device_find(TASK_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
    if (device == RT_NULL) {
        rt_kprintf("nrf device not found: %s\n", TASK_MINIFLY_TASK_NRF_DEVICE_DEFAULT);
        return -1;
    }

    rt_err_t result = rt_device_open(device, RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK) {
        rt_kprintf("nrf device open failed\n");
        return -1;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = TASK_MINIFLY_TASK_NRF_BAUD_RATE;
    rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config);

    task_msg_init();

    rt_thread_init(&taskNrfTid, "task_nrf", radiolinkTask, RT_NULL, taskNrfStack,
                   THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&taskNrfTid);

    rt_kprintf("nrf task started on %s, baud: %d\n", 
               TASK_MINIFLY_TASK_NRF_DEVICE_DEFAULT, 
               TASK_MINIFLY_TASK_NRF_BAUD_RATE);

    return 0;
}

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
INIT_APP_EXPORT(taskNrfInit);
#endif
