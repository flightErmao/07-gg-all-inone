#include "rtthread.h"
#include <rtdevice.h>
#include "sensorsTypes.h"
#include "taskMiniflyStabilizer.h"
#include "sensfusion6.h"
#include "taskMiniflySensor.h"

/*task definition*/
#define THREAD_PRIORITY 6
#define THREAD_STACK_SIZE 4096

#define THREAD_TIMESLICE 5

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴自稳控制代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

static bool isInit;

static sensorData_t sensorData; /*传感器数据*/
static state_t state_minifly_;  /*四轴姿态*/

// RT-Thread任务相关定义
static struct rt_thread task_tid_stabilizer_minifly;
static rt_uint8_t task_stack_stabilizer_minifly[THREAD_STACK_SIZE];
static rt_timer_t stabilizer_timer = RT_NULL;
static rt_event_t stabilizer_event = RT_NULL;

// 控制变量声明
static control_t control = {0};

// 传感器校准检查函数声明
static bool sensorsAreCalibrated(void) {
  // 这里需要实现传感器校准检查逻辑
  // 暂时返回true，实际应用中需要根据具体需求实现
  return true;
}

// 软件定时器回调函数
static void stabilizer_timer_callback(void* parameter) {
  // 发布事件到稳定器任务
  rt_event_send(stabilizer_event, 0x01);
}

void taskStabilizerInit(void) {
  if (isInit) return;

  // 创建事件对象
  stabilizer_event = rt_event_create("stabilizer_event", RT_IPC_FLAG_FIFO);
  if (stabilizer_event == RT_NULL) {
    rt_kprintf("Failed to create stabilizer event\n");
    return;
  }

  // 创建软件定时器，周期为1ms (1000Hz)
  stabilizer_timer =
      rt_timer_create("stabilizer_timer", stabilizer_timer_callback, RT_NULL, MAIN_LOOP_DT, RT_TIMER_FLAG_PERIODIC);

  if (stabilizer_timer == RT_NULL) {
    rt_kprintf("Failed to create stabilizer timer\n");
    rt_event_delete(stabilizer_event);
    return;
  }

  isInit = true;
}

static void stabilizer_minifly_thread_entry(void* parameter) {
  uint32_t tick = 0;
  rt_uint32_t recv_event = 0;
  rt_err_t result;

  // 等待传感器校准完成
  // while (!sensorsAreCalibrated()) {
  //   rt_thread_mdelay(10);
  // }

  // 启动软件定时器
  rt_timer_start(stabilizer_timer);

  while (1) {
    // 等待定时器事件
    result =
        rt_event_recv(stabilizer_event, 0x01, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recv_event);

    if (result == RT_EOK) {
      // 四元数和欧拉角计算（250Hz）
      if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick)) {
        sensorsAcquire(&sensorData);
        imuUpdate(sensorData.acc_filter, sensorData.gyro_filter, &state_minifly_, ATTITUDE_ESTIMAT_DT);
      }

      tick++;
    }
  }
}

// 任务初始化函数
static int taskStabilizerThreadAutoStart(void) {
  taskStabilizerInit();
  rt_thread_init(&task_tid_stabilizer_minifly, "t_stabilizer", stabilizer_minifly_thread_entry, RT_NULL,
                 task_stack_stabilizer_minifly, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_stabilizer_minifly);
}

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
INIT_APP_EXPORT(taskStabilizerThreadAutoStart);
#endif
