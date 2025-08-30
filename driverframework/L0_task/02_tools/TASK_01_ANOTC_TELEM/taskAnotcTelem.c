#include <rtdevice.h>
#include "deviceManager.h"
#include "taskAnotcTelem.h"
#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define SENSOR_TASK_PRIORITY 24
#define SENSOR_TASK_STACK_SIZE 2048
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

static sensor_data_send_func_t sensor_func_list_[MAX_SENSOR_FUNCS] = {0};
static uint8_t sensor_func_count_ = 0;

/* Init message queue */
static void task_msg_init(void) {
  rt_err_t result;
  result = rt_mq_init(&device_send_mq_, "mqt", &msg_pool[0], sizeof(atkp_t), POOL_SIZE_BYTE, RT_IPC_FLAG_PRIO);
  if (result != RT_EOK) {
    rt_kprintf("rt_mq_init ERR\n");
  } else {
    rt_kprintf("rt_mq_init OK\n");
  }
}

/* Periodic sensor data sender */
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
    rt_thread_mdelay(1);
  }
}

/* Forward queued packets to device */
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
  rt_thread_init(&taskAnotcMqRecTid, "L0_tool_atkp_rec", taskAnotcMqRecEntry, RT_NULL, taskAnotcMqRecStack,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&taskAnotcMqRecTid);

  return 0;
}

static int taskAnotcMqSend(void) {
  rt_thread_init(&taskAnotcMqSendTid, "L0_tool_atkp_send", taskAnotcMqSendEntry, RT_NULL, taskAnotcMqSendStack,
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
  result = rt_mq_send(&device_send_mq_, p, sizeof(atkp_t));
  if (result != RT_EOK) {
    rt_kprintf("rt_mq_send ERR\n");
  }
}

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
