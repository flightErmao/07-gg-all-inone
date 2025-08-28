#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>  // 使用标准C库的math.h
#include "config.h"
#include "taskParam.h"
#include "stmflash.h"

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 配置参数驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

#define VERSION 13 /*13 表示V1.3*/

// RT-Thread相关定义
#define THREAD_PRIORITY 8
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define MSG_NUM 10
#define POOL_SIZE_BYTE (sizeof(configParam_t) * MSG_NUM)

configParam_t configParam;

static configParam_t configParamDefault = {
    .version = VERSION, /*固件版本号*/

    .pidAngle = /*角度PID*/
    {
        .roll =
            {
                .kp = 8.0,
                .ki = 0.0,
                .kd = 0.0,
            },
        .pitch =
            {
                .kp = 8.0,
                .ki = 0.0,
                .kd = 0.0,
            },
        .yaw =
            {
                .kp = 20.0,
                .ki = 0.0,
                .kd = 1.5,
            },
    },
    .pidRate = /*角速度PID*/
    {
        .roll =
            {
                .kp = 300.0,
                .ki = 0.0,
                .kd = 6.5,
            },
        .pitch =
            {
                .kp = 300.0,
                .ki = 0.0,
                .kd = 6.5,
            },
        .yaw =
            {
                .kp = 200.0,
                .ki = 18.5,
                .kd = 0.0,
            },
    },
    .pidPos = /*位置PID - 优化版本，减少抖动*/
    {
        .vx =
            {
                .kp = 2.0f, /*降低比例增益，减少超调*/
                .ki = 0.0f, /*增加积分增益，消除稳态误差*/
                .kd = 0.0f, /*增加微分增益，抑制振荡*/
            },
        .vy =
            {
                .kp = 2.0f, /*降低比例增益，减少超调*/
                .ki = 0.0f, /*增加积分增益，消除稳态误差*/
                .kd = 0.0f, /*增加微分增益，抑制振荡*/
            },
        .vz =
            {
                .kp = 80.0f,  /*降低比例增益，减少垂直抖动*/
                .ki = 120.0f, /*保持积分增益，确保定高精度*/
                .kd = 12.0f,  /*增加微分增益，抑制垂直振荡*/
            },

        .x =
            {
                .kp = 2.0f, /*降低比例增益，减少位置超调*/
                .ki = 0.0f, /*增加积分增益，消除位置误差*/
                .kd = 0.0f, /*增加微分增益，抑制位置振荡*/
            },
        .y =
            {
                .kp = 2.0f, /*降低比例增益，减少位置超调*/
                .ki = 0.0f, /*增加积分增益，消除位置误差*/
                .kd = 0.0f, /*增加微分增益，抑制位置振荡*/
            },
        .z =
            {
                .kp = 4.5f,  /*降低比例增益，减少高度超调*/
                .ki = 0.08f, /*增加积分增益，消除高度误差*/
                .kd = 6.0f,  /*增加微分增益，抑制高度振荡*/
            },
    },

    .trimP = 0.f,        /*pitch微调*/
    .trimR = 0.f,        /*roll微调*/
    .thrustBase = 34000, /*油门基准值*/
};

static uint32_t lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;

// RT-Thread相关变量
static rt_align(RT_ALIGN_SIZE) static rt_uint8_t configParamStack[THREAD_STACK_SIZE];
static struct rt_thread configParamTid;
static rt_uint8_t msg_pool[POOL_SIZE_BYTE];
static struct rt_messagequeue configParam_mq_;
static struct rt_messagequeue *configParam_mq = RT_NULL;

static uint8_t configParamCksum(configParam_t *data) {
  int i;
  uint8_t cksum = 0;
  uint8_t *c = (uint8_t *)data;
  size_t len = sizeof(configParam_t);

  for (i = 0; i < len; i++) cksum += *(c++);
  cksum -= data->cksum;

  return cksum;
}

void configParamInit(void) /*参数配置初始化*/
{
  if (isInit) return;

  lenth = sizeof(configParam);
  lenth = lenth / 4 + (lenth % 4 ? 1 : 0);

  STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth);

  if (configParam.version == VERSION) /*版本正确*/
  {
    if (configParamCksum(&configParam) == configParam.cksum) /*校验正确*/
    {
      rt_kprintf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
      isConfigParamOK = true;
    } else {
      rt_kprintf("Version check [FAIL]\r\n");
      isConfigParamOK = false;
    }
  } else /*版本更新*/
  {
    isConfigParamOK = false;
  }

  if (isConfigParamOK == false) /*配置参数错误，写入默认参数*/
  {
    memcpy((uint8_t *)&configParam, (uint8_t *)&configParamDefault, sizeof(configParam));
    configParam.cksum = configParamCksum(&configParam);                 /*计算校验值*/
    STMFLASH_Write(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth); /*写入stm32 flash*/
    isConfigParamOK = true;
  }

  // 初始化消息队列
  rt_err_t result =
      rt_mq_init(&configParam_mq_, "config_mq", &msg_pool[0], sizeof(configParam_t), POOL_SIZE_BYTE, RT_IPC_FLAG_PRIO);
  if (result != RT_EOK) {
    rt_kprintf("config param mq init ERR\n");
  } else {
    rt_kprintf("config param mq init OK\n");
  }
  configParam_mq = &configParam_mq_;

  isInit = true;
}

void configParamTask(void *param) {
  uint8_t cksum = 0;

  while (1) {
    // 等待消息队列信号
    if (rt_mq_recv(configParam_mq, &configParam, sizeof(configParam), RT_WAITING_FOREVER) > 0) {
      cksum = configParamCksum(&configParam); /*数据校验*/

      if (configParam.cksum != cksum) {
        configParam.cksum = cksum; /*数据校验*/
#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
        // watchdogInit(500);			/*擦除时间比较长，看门狗时间设置大一些*/
        STMFLASH_Write(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth); /*写入stm32 flash*/
        // watchdogInit(WATCHDOG_RESET_MS);		/*重新设置看门狗*/
#endif
      }
    }
  }
}

bool configParamTest(void) { return isInit; }

void configParamGiveSemaphore(void) {
  if (configParam_mq != RT_NULL) {
    rt_mq_send(configParam_mq, &configParam, sizeof(configParam));
  }
}

void resetConfigParamPID(void) {
  configParam.pidAngle = configParamDefault.pidAngle;
  configParam.pidRate = configParamDefault.pidRate;
  configParam.pidPos = configParamDefault.pidPos;
}

void saveConfigAndNotify(void) {
  uint8_t cksum = configParamCksum(&configParam); /*数据校验*/
  if (configParam.cksum != cksum) {
    configParam.cksum = cksum;                                          /*数据校验*/
    STMFLASH_Write(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth); /*写入stm32 flash*/
  }
}

// RT-Thread相关函数实现
rt_err_t configParamTaskInit(void) {
  if (!isInit) {
    rt_kprintf("Config param not initialized yet!\n");
    return -RT_ERROR;
  }

  rt_thread_init(&configParamTid, "L0_minifly_configParam", configParamTask, RT_NULL, configParamStack,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&configParamTid);

  rt_kprintf("Config param task started\r\n");
  return RT_EOK;
}

struct rt_messagequeue *getConfigParamMq(void) { return configParam_mq; }

#ifdef PROJECT_MINIFLY_TASK05_PARAM_EN
INIT_APP_EXPORT(configParamTaskInit);
#endif
