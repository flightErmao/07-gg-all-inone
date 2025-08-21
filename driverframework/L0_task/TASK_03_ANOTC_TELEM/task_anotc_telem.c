
#include "task_anotc_telem.h"
#include <rtdevice.h>

#ifdef BSP_USING_UART1
#define ANOTC_TELEM_UART_NAME "uart1"
#endif
#ifdef BSP_USING_UART2
#define ANOTC_TELEM_UART_NAME "uart2"
#endif

// #define TASK_ANOTC_PIN_DEBUG_EN

// 数据拆分宏定义
#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

// 数据返回周期时间（单位ms）
#define PERIOD_USERDATA 1

#define ATKP_RX_QUEUE_SIZE 10 /*ATKP包接收队列消息个数*/

#define USER_FRAME_F1 1
#define USER_FRAME_F2 2
#define USER_FRAME_F3 3

/*上行帧头*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 128
#define ATKP_PROTOCOL_HEAD_SIZE 6
#define ATKP_ANOTC_TELEM_BUF_SIZE (ATKP_MAX_DATA_SIZE + ATKP_PROTOCOL_HEAD_SIZE)

/*下行指令*/
#define D_COMMAND_ACC_CALIB 0x01
#define D_COMMAND_GYRO_CALIB 0x02
#define D_COMMAND_MAG_CALIB 0x04
#define D_COMMAND_BARO_CALIB 0x05
#define D_COMMAND_ACC_CALIB_EXIT 0x20
#define D_COMMAND_ACC_CALIB_STEP1 0x21
#define D_COMMAND_ACC_CALIB_STEP2 0x22
#define D_COMMAND_ACC_CALIB_STEP3 0x23
#define D_COMMAND_ACC_CALIB_STEP4 0x24
#define D_COMMAND_ACC_CALIB_STEP5 0x25
#define D_COMMAND_ACC_CALIB_STEP6 0x26
#define D_COMMAND_FLIGHT_LOCK 0xA0
#define D_COMMAND_FLIGHT_ULOCK 0xA1

#define D_ACK_READ_PID 0x01
#define D_ACK_READ_VERSION 0xA0
#define D_ACK_RESET_PARAM 0xA1

#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define MSG_NUM 30
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)
#define ANOTC_TELEM_BAUD_RATE 115200

/*通讯数据结构*/
typedef struct
{
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[ATKP_MAX_DATA_SIZE];
} atkp_t;

/*上行指令ID*/
typedef enum
{
    UP_VERSION = 0x00,
    UP_STATUS = 0x01,
    UP_SENSER = 0x02,
    UP_RCDATA = 0x03,
    UP_GPSDATA = 0x04,
    UP_POWER = 0x05,
    UP_MOTOR = 0x06,
    UP_SENSER2 = 0x07,
    UP_FLYMODE = 0x0A,
    UP_SPEED = 0x0B,
    UP_PID1 = 0x10,
    UP_PID2 = 0x11,
    UP_PID3 = 0x12,
    UP_PID4 = 0x13,
    UP_PID5 = 0x14,
    UP_PID6 = 0x15,
    UP_RADIO = 0x40,
    UP_MSG = 0xEE,
    UP_CHECK = 0xEF,

    UP_REMOTER = 0x50,
    UP_PRINTF = 0x51,

    UP_USER_DATA1 = 0xF1,
    UP_USER_DATA2 = 0xF2,
    UP_USER_DATA3 = 0xF3,
    UP_USER_DATA4 = 0xF4,
    UP_USER_DATA5 = 0xF5,
    UP_USER_DATA6 = 0xF6,
    UP_USER_DATA7 = 0xF7,
    UP_USER_DATA8 = 0xF8,
    UP_USER_DATA9 = 0xF9,
    UP_USER_DATA10 = 0xFA,
} upmsgID_e;

/*下行指令ID*/
typedef enum
{
    DOWN_COMMAND = 0x01,
    DOWN_ACK = 0x02,
    DOWN_RCDATA = 0x03,
    DOWN_POWER = 0x05,
    DOWN_FLYMODE = 0x0A,
    DOWN_PID1 = 0x10,
    DOWN_PID2 = 0x11,
    DOWN_PID3 = 0x12,
    DOWN_PID4 = 0x13,
    DOWN_PID5 = 0x14,
    DOWN_PID6 = 0x15,
    DOWN_RADIO = 0x40,

    DOWN_REMOTER = 0x50,
} downmsgID_e;

/* 使用 rtdevice.h 中的 struct serial_configure 定义，删除本地重复定义 */

rt_align(RT_ALIGN_SIZE) static rt_uint8_t task_anotc_telem_cache_stack[THREAD_STACK_SIZE];
static struct rt_thread task_anotc_telem_cache_tid;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t task_anotc_telem_dispatch_stack[THREAD_STACK_SIZE];
static struct rt_thread task_anotc_telem_dispatch_tid;

static rt_device_t anotc_telem_dev = RT_NULL;
static char anotc_output_name[8] = "uart1"; /* 默认输出设备名称 */
extern int usb_change_shell(void); /* 切换vconsole到USB */
static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue mq;

/*打包ATKPPacket数据通过串口DMA发送*/
void anotc_telem_uartSendPacket(atkp_t *p)
{
    int dataSize;
    uint8_t cksum = 0;
    static uint8_t sendBuffer[ATKP_ANOTC_TELEM_BUF_SIZE];

    RT_ASSERT((p->dataLen <= ATKP_MAX_DATA_SIZE) == true);

    sendBuffer[0] = UP_BYTE1;
    sendBuffer[1] = UP_BYTE2;
    sendBuffer[2] = p->msgID;
    sendBuffer[3] = p->dataLen;

    memcpy(&sendBuffer[4], p->data, p->dataLen);
    dataSize = p->dataLen + 5;  // 加上cksum
    /*计算校验和*/
    for (int i = 0; i < dataSize - 1; i++)
    {
        cksum += sendBuffer[i];
    }
    sendBuffer[dataSize - 1] = cksum;
    if (anotc_telem_dev != RT_NULL)
    {
        rt_device_write(anotc_telem_dev, 0, sendBuffer, dataSize);
    }
}

void anotc_telem_stash_msg_to_mq(atkp_t *p)
{
    int result;
    /* 发送消息到消息队列中 */
    result = rt_mq_send(&mq, p, sizeof(atkp_t));
    if (result != RT_EOK)
    {
        rt_kprintf("rt_mq_send ERR\n");
    }
}


static void sendUserDataLine6_int16(uint8_t group, int16_t *buf_data_cat, msg_send_method_e method)
{
    uint8_t _cnt = 0;
    atkp_t p;

    p.msgID = UP_USER_DATA1 + group - 1;

    p.data[_cnt++] = BYTE1(buf_data_cat[0]);
    p.data[_cnt++] = BYTE0(buf_data_cat[0]);
    p.data[_cnt++] = BYTE1(buf_data_cat[1]);
    p.data[_cnt++] = BYTE0(buf_data_cat[1]);
    p.data[_cnt++] = BYTE1(buf_data_cat[2]);
    p.data[_cnt++] = BYTE0(buf_data_cat[2]);
    p.data[_cnt++] = BYTE1(buf_data_cat[3]);
    p.data[_cnt++] = BYTE0(buf_data_cat[3]);
    p.data[_cnt++] = BYTE1(buf_data_cat[4]);
    p.data[_cnt++] = BYTE0(buf_data_cat[4]);
    p.data[_cnt++] = BYTE1(buf_data_cat[5]);
    p.data[_cnt++] = BYTE0(buf_data_cat[5]);

    p.dataLen = _cnt;
    if (method == MSG_ASYNC)
    {
        anotc_telem_stash_msg_to_mq(&p);
    }
    else
    {
        anotc_telem_uartSendPacket(&p);
    }
}

void sendUserDatafloat3(uint8_t group, float a,float b,float c)
{
    uint8_t _cnt = 0;
    atkp_t p;

    p.msgID = UP_USER_DATA1 + group - 1;

    float temp = a;
    p.data[_cnt++] = BYTE3(temp);
    p.data[_cnt++] = BYTE2(temp);
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);

    temp = b;
    p.data[_cnt++] = BYTE3(temp);
    p.data[_cnt++] = BYTE2(temp);
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);

    temp = c;
    p.data[_cnt++] = BYTE3(temp);
    p.data[_cnt++] = BYTE2(temp);
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
    p.dataLen = _cnt;
    anotc_telem_stash_msg_to_mq(&p);
}

void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  float temp = a;
  p.data[_cnt++] = BYTE3(temp);
  p.data[_cnt++] = BYTE2(temp);
  p.data[_cnt++] = BYTE1(temp);
  p.data[_cnt++] = BYTE0(temp);

  temp = b;
  p.data[_cnt++] = BYTE3(temp);
  p.data[_cnt++] = BYTE2(temp);
  p.data[_cnt++] = BYTE1(temp);
  p.data[_cnt++] = BYTE0(temp);

  temp = c;
  p.data[_cnt++] = BYTE3(temp);
  p.data[_cnt++] = BYTE2(temp);
  p.data[_cnt++] = BYTE1(temp);
  p.data[_cnt++] = BYTE0(temp);

  temp = d;
  p.data[_cnt++] = BYTE3(temp);
  p.data[_cnt++] = BYTE2(temp);
  p.data[_cnt++] = BYTE1(temp);
  p.data[_cnt++] = BYTE0(temp);

  temp = e;
  p.data[_cnt++] = BYTE3(temp);
  p.data[_cnt++] = BYTE2(temp);
  p.data[_cnt++] = BYTE1(temp);
  p.data[_cnt++] = BYTE0(temp);

  temp = f;
  p.data[_cnt++] = BYTE3(temp);
  p.data[_cnt++] = BYTE2(temp);
  p.data[_cnt++] = BYTE1(temp);
  p.data[_cnt++] = BYTE0(temp);

  p.dataLen = _cnt;
  anotc_telem_stash_msg_to_mq(&p);
}

static void setUserData_int16(uint8_t index, int16_t *buf, int16_t value)
{
    buf[index] = value;
}

static rt_err_t task_dev_init(void)
{
    if (anotc_telem_dev && (anotc_telem_dev->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        return RT_EOK;
    }

    /* 根据当前选择的输出设备名称查找设备 */
    rt_device_t new_dev = rt_device_find(anotc_output_name);
    if (new_dev == NULL)
    {
        return RT_ERROR;
    }

    /* open new device */
    if (rt_device_open(new_dev, RT_DEVICE_FLAG_INT_TX) != RT_EOK)
    {
        return RT_ERROR;
    }

    /* 仅在UART类设备上进行串口参数配置；USB CDC(vcom)无需配置为serial_configure */
    if (!strncmp(anotc_output_name, "uart", 4))
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = ANOTC_TELEM_BAUD_RATE;
        config.bufsz = 64;
        if (rt_device_control(new_dev, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
        {
            rt_device_close(new_dev);
            return RT_ERROR;
        }
    }

    /* set new device */
    anotc_telem_dev = new_dev;
    return RT_EOK;
}

/* 切换输出设备: 目标可为 "uart1" / "uart2" / "usb"(映射到"vcom") */
static int anotc_telem_switch_output(const char *target)
{
    if (target == RT_NULL)
    {
        return -RT_EINVAL;
    }

    char next_name[8] = {0};
    if (!strcmp(target, "usb"))
    {
        strncpy(next_name, "vcom", sizeof(next_name) - 1);
    }
    else if (!strcmp(target, "uart1"))
    {
        strncpy(next_name, "uart1", sizeof(next_name) - 1);
    }
    else if (!strcmp(target, "uart2"))
    {
        strncpy(next_name, "uart2", sizeof(next_name) - 1);
    }
    else
    {
        rt_kprintf("未知设备: %s\n", target);
        return -RT_ERROR;
    }

    /* 若当前设备已打开，则先关闭 */
    if (anotc_telem_dev)
    {
        rt_device_close(anotc_telem_dev);
        anotc_telem_dev = RT_NULL;
    }

    /* 更新名称并重新初始化设备 */
    strncpy(anotc_output_name, next_name, sizeof(anotc_output_name) - 1);
    if (task_dev_init() != RT_EOK)
    {
        rt_kprintf("切换到 %s 失败\n", anotc_output_name);
        return -RT_ERROR;
    }

    rt_kprintf("anotc 输出已切换到: %s\n", anotc_output_name);

    /* 切到USB时，顺带切换vconsole到USB终端 */
    if (!strcmp(target, "usb"))
    {
#ifdef PKG_USING_VCONSOLE
        usb_change_shell();
#endif
    }

    return RT_EOK;
}

static void task_msg_init(void)
{
    rt_err_t result;

    /* 初始化消息队列 */
    result = rt_mq_init(&mq, "mqt", &msg_pool[0], /* 内存池指向msg_pool */
                        sizeof(atkp_t),           /* 每个消息的大小是 sizeof(atkp_t) 字节 */
                        POOL_SIZE_BYTE,           /* 内存池的大小是msg_pool的大小 */
                        RT_IPC_FLAG_PRIO); /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    if (result != RT_EOK)
    {
        rt_kprintf("rt_mq_init ERR\n");
    }
    else
    {
        rt_kprintf("rt_mq_init OK\n");
    }
}

#ifdef BSP_USING_ICM42688
static void task_anotc_send_icm42688(void *param)
{
    int16_t user_data_f1[6] = {0};
    while (1)
    {
        waitImuEvent();
        int16_t send_count = ImuPackCount();
        uint16_t packet_size = ImuPackSize();
        uint8_t *p_temp = fifoBufImuRawdata();
        uint8_t p[512] = {0};
        memcpy(p, p_temp, packet_size * send_count);
        for (uint16_t index = 0; index < send_count; index++)
        {
            user_data_f1[0] = p[index * packet_size + 0x01] * 256 + p[index * packet_size + 0x02];
            user_data_f1[1] = p[index * packet_size + 0x03] * 256 + p[index * packet_size + 0x04];
            user_data_f1[2] = p[index * packet_size + 0x05] * 256 + p[index * packet_size + 0x06];
            user_data_f1[3] = p[index * packet_size + 0x07] * 256 + p[index * packet_size + 0x08];
            user_data_f1[4] = p[index * packet_size + 0x09] * 256 + p[index * packet_size + 0x0A];
            user_data_f1[5] = p[index * packet_size + 0x0B] * 256 + p[index * packet_size + 0x0C];
            sendUserDataLine6_int16_sync(IMU_DATA, user_data_f1);
        }
    }
}
#endif

/*anotc_telem接收到ATKPPacket预处理*/
static void task_anotc_telem_dispatch_entry(void *param)
{
    atkp_t msg_temp;
    RT_ASSERT(task_dev_init() == RT_EOK);
    task_msg_init();
    while (1)
    {
#if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
        if (rt_mq_recv(&mq, &msg_temp, sizeof(msg_temp), RT_WAITING_FOREVER) > 0)
#else
        if (rt_mq_recv(&mq, &msg_temp, sizeof(msg_temp), RT_WAITING_FOREVER) == RT_EOK)
#endif
        {
            anotc_telem_uartSendPacket(&msg_temp);
        }
    }
}


int task_anotc_telem(void)
{    
    rt_thread_init(&task_anotc_telem_dispatch_tid, "task_anotc_telem_dispatch", task_anotc_telem_dispatch_entry,
                   RT_NULL, task_anotc_telem_dispatch_stack, THREAD_STACK_SIZE, THREAD_PRIORITY + 1, THREAD_TIMESLICE);
    rt_thread_startup(&task_anotc_telem_dispatch_tid);

    return 0;
}

/* 命令: anotc out <uart1|uart2|usb> */
static int cmd_anotc(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("anotc 命令用法:\n");
        rt_kprintf("  anotc out <uart1|uart2|usb>  - 切换输出设备 (默认uart1)\n");
        rt_kprintf("当前输出: %s\n", anotc_output_name);
        return 0;
    }

    if (!rt_strcmp(argv[1], "out"))
    {
        if (argc < 3)
        {
            rt_kprintf("缺少设备参数，支持: uart1|uart2|usb\n");
            return -1;
        }
        return anotc_telem_switch_output(argv[2]);
    }

    rt_kprintf("未知子命令: %s\n", argv[1]);
    rt_kprintf("使用 'anotc' 查看帮助\n");
    return -1;
}

MSH_CMD_EXPORT_ALIAS(cmd_anotc, anotc, anotc telem output switch command);

#ifdef BSP_USING_TASK_03_ANOTC_TELEM
INIT_APP_EXPORT(task_anotc_telem);
#endif

