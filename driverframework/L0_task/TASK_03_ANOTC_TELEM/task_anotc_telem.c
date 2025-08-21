
#include "task_anotc_telem.h"
#include <rtdevice.h>

/* 线程与消息队列配置已移至 task_anotc_telem.h */

// #define TASK_ANOTC_PIN_DEBUG_EN

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


/* 通用: 发送 N 个 float 数据，支持同步/异步 */
static void anotc_telem_send_floats(uint8_t group, const float *values, uint8_t count, msg_send_method_e method)
{
    uint8_t _cnt = 0;
    atkp_t p;

    p.msgID = UP_USER_DATA1 + group - 1;

    for (uint8_t i = 0; i < count; i++)
    {
        float temp = values[i];
        p.data[_cnt++] = BYTE3(temp);
        p.data[_cnt++] = BYTE2(temp);
        p.data[_cnt++] = BYTE1(temp);
        p.data[_cnt++] = BYTE0(temp);
    }

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
    float values[3] = {a, b, c};
    anotc_telem_send_floats(group, values, 3, MSG_ASYNC);
}

void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f) {
  float values[6] = {a, b, c, d, e, f};
  anotc_telem_send_floats(group, values, 6, MSG_ASYNC);
}

static void setUserData_int16(uint8_t index, int16_t *buf, int16_t value)
{
    buf[index] = value;
}

/* 对外：发送6路int16（同步/异步） */
void sendUserDataLine6_int16_sync(uint8_t group, int16_t *buf_data_cat)
{
    sendUserDataLine6_int16(group, buf_data_cat, MSG_SYNC);
}

void sendUserDataLine6_int16_async(uint8_t group, int16_t *buf_data_cat)
{
    sendUserDataLine6_int16(group, buf_data_cat, MSG_ASYNC);
}

/* 兼容接口：发送6路float（同步/异步由参数决定） */
void anotc_telem_sendUserDataLine6_float(uint8_t group, float *buf_data_cat, msg_send_method_e method)
{
    if (buf_data_cat == RT_NULL)
    {
        return;
    }
    anotc_telem_send_floats(group, buf_data_cat, 6, method);
}

/* 兼容接口：设置 float 缓冲区的某一项 */
void setUserData_float(uint8_t index, float *buf, float value)
{
    if (buf == RT_NULL)
    {
        return;
    }
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

