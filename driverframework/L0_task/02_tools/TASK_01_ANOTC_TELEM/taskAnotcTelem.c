#include <rtdevice.h>
#include "deviceManager.h"
#include "taskAnotcTelem.h"
/* 线程与消息队列配置 */
#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
/* 传感器数据发送任务配置 */
#define SENSOR_TASK_PRIORITY 24
#define SENSOR_TASK_STACK_SIZE 1024
#define SENSOR_TASK_TIMESLICE 5

#define MAX_SENSOR_FUNCS 10
#define MSG_NUM 30
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)

rt_align(RT_ALIGN_SIZE) static rt_uint8_t taskAnotcMqRecStack[THREAD_STACK_SIZE];
static struct rt_thread taskAnotcMqRecTid;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t taskAnotcMqSendStack[SENSOR_TASK_STACK_SIZE];
static struct rt_thread taskAnotcMqSendTid;

static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue device_send_mq_;

/* 传感器函数列表管理 */
static sensor_data_send_func_t sensor_func_list_[MAX_SENSOR_FUNCS] = {0};
static uint8_t sensor_func_count_ = 0;

static void task_msg_init(void) {
  rt_err_t result;

  /* 初始化消息队列 */
  result = rt_mq_init(&device_send_mq_, "mqt", &msg_pool[0], /* 内存池指向msg_pool */
                      sizeof(atkp_t),                        /* 每个消息的大小是 sizeof(atkp_t) 字节 */
                      POOL_SIZE_BYTE,                        /* 内存池的大小是msg_pool的大小 */
                      RT_IPC_FLAG_PRIO);                     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
  if (result != RT_EOK) {
    rt_kprintf("rt_mq_init ERR\n");
  } else {
    rt_kprintf("rt_mq_init OK\n");
  }
}

/* 传感器数据发送任务入口函数 */
static void taskAnotcMqSendEntry(void *param) {
  uint16_t count_ms = 0;

  rt_kprintf("Sensor data task started\n");

  while (1) {
    for (uint8_t i = 0; i < sensor_func_count_; i++) {
      if (sensor_func_list_[i] != RT_NULL) {
        sensor_func_list_[i](count_ms);
      }
    }
    count_ms++;
    rt_thread_mdelay(1); /* 1ms周期 */
  }
}

static void taskAnotcMqRecEntry(void *param) {
  atkp_t msg_temp;
  while (1) {
#if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
    if (rt_mq_recv(&device_send_mq_, &msg_temp, sizeof(msg_temp), RT_WAITING_FOREVER) > 0)
#else
    if (rt_mq_recv(&device_send_mq_, &msg_temp, sizeof(msg_temp), RT_WAITING_FOREVER) == RT_EOK)
#endif
    {
      anotcDeviceSendDirect(&msg_temp);
    }
  }
}

static int taskAnotcMqRec(void) {
  task_dev_init(TASK_TOOL_01_ANOTC_TELEM_DEVICE_DEFAULT);
  task_msg_init();
  rt_thread_init(&taskAnotcMqRecTid, "taskAnotcMqRec", taskAnotcMqRecEntry, RT_NULL, taskAnotcMqRecStack,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&taskAnotcMqRecTid);

  return 0;
}

/* 启动传感器数据发送任务 */
static int taskAnotcMqSend(void) {
  rt_thread_init(&taskAnotcMqSendTid, "taskAnotcMqSend", taskAnotcMqSendEntry, RT_NULL, taskAnotcMqSendStack,
                 SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, SENSOR_TASK_TIMESLICE);
  rt_thread_startup(&taskAnotcMqSendTid);

  return 0;
}

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN
INIT_APP_EXPORT(taskAnotcMqRec);
INIT_APP_EXPORT(taskAnotcMqSend);
#endif

void anotcMqStash(atkp_t *p) {
  int result;
  /* 发送消息到消息队列中 */
  result = rt_mq_send(&device_send_mq_, p, sizeof(atkp_t));
  if (result != RT_EOK) {
    rt_kprintf("rt_mq_send ERR\n");
  }
}

/* 添加传感器数据发送函数到列表 */
void anotcTelemAddSensorFunc(sensor_data_send_func_t func) {
  if (func == RT_NULL) {
    return;
  }

  for (uint8_t i = 0; i < sensor_func_count_; i++) {
    if (sensor_func_list_[i] == func) {
      return;
    }
  }

  if (sensor_func_count_ < MAX_SENSOR_FUNCS) {
    sensor_func_list_[sensor_func_count_++] = func;
    rt_kprintf("Added sensor func, total: %d\n", sensor_func_count_);
  } else {
    rt_kprintf("Sensor func list full!\n");
  }
}
