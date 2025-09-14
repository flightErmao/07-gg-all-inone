#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "taskParam.h"
#include "configParamDefault.h"
#include <fal.h>

#define THREAD_PRIORITY 10
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 5

static rt_align(RT_ALIGN_SIZE) rt_uint8_t configParamStack[THREAD_STACK_SIZE];
static struct rt_thread configParamTid;
static rt_sem_t configParam_sem;

static configParam_t configParam;
#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
static uint32_t lenth = 0;
static const struct fal_partition *param_partition = RT_NULL;
#endif
static bool isInit = false;
static bool isConfigParamOK = false;

static uint8_t configParamCksum(configParam_t *data) {
  int i;
  uint8_t cksum = 0;
  uint8_t *c = (uint8_t *)data;
  size_t len = sizeof(configParam_t);

  for (i = 0; i < len; i++) cksum += *(c++);
  cksum -= data->cksum;

  return cksum;
}

static void configParamInit(void) {
  if (isInit) return;

#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
  lenth = sizeof(configParam);
  lenth = lenth / 4 + (lenth % 4 ? 1 : 0);

  param_partition = fal_partition_find("par");
  if (param_partition == RT_NULL) {
    rt_kprintf("FAL partition 'par' not found\r\n");
    isConfigParamOK = false;
  } else {
    int ret = fal_partition_read(param_partition, 0, (rt_uint8_t *)&configParam, sizeof(configParam));
    if (ret < 0) {
      rt_kprintf("FAL partition read failed\r\n");
      isConfigParamOK = false;
    } else {
      if (configParam.version == VERSION) {
        if (configParamCksum(&configParam) == configParam.cksum) {
          rt_kprintf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
          isConfigParamOK = true;
        } else {
          rt_kprintf("Version check [FAIL]\r\n");
          isConfigParamOK = false;
        }
      } else {
        isConfigParamOK = false;
      }
    }
  }
#endif

  if (isConfigParamOK == false) {
    memcpy((uint8_t *)&configParam, (uint8_t *)&configParamDefault, sizeof(configParam));
    configParam.cksum = configParamCksum(&configParam);
#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
    if (param_partition != RT_NULL) {
      int ret = fal_partition_write(param_partition, 0, (const rt_uint8_t *)&configParam, sizeof(configParam));
      if (ret < 0) {
        rt_kprintf("FAL partition write failed\r\n");
      }
    }
#endif
    isConfigParamOK = true;
  }

  // Initialize semaphore
  configParam_sem = rt_sem_create("config_sem", 0, RT_IPC_FLAG_PRIO);
  if (configParam_sem == RT_NULL) {
    rt_kprintf("config param sem create ERR\n");
  } else {
    rt_kprintf("config param sem create OK\n");
  }

  isInit = true;
}

void configParamTask(void *param) {
  uint8_t cksum = 0;

  while (1) {
    // Wait for semaphore signal
    if (rt_sem_take(configParam_sem, RT_WAITING_FOREVER) == RT_EOK) {
      cksum = configParamCksum(&configParam);

      if (configParam.cksum != cksum) {
        configParam.cksum = cksum;
#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
        // watchdogInit(500);
        if (param_partition != RT_NULL) {
          int ret = fal_partition_write(param_partition, 0, (const rt_uint8_t *)&configParam, sizeof(configParam));
          if (ret < 0) {
            rt_kprintf("FAL partition write failed\r\n");
          }
        }
        // watchdogInit(WATCHDOG_RESET_MS);
#endif
      }
    }
  }
}

static int configParamTaskInit(void) {
  configParamInit();

  rt_thread_init(&configParamTid, "configPar", configParamTask, RT_NULL, configParamStack, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&configParamTid);

  rt_kprintf("Config param task started\r\n");
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK05_PARAM_EN
INIT_APP_EXPORT(configParamTaskInit);
#endif

void configParamGiveSemaphore(void) {
  if (configParam_sem != RT_NULL) {
    rt_sem_release(configParam_sem);
  }
}

void resetConfigParamPID(void) {
  configParam.pidAngle = configParamDefault.pidAngle;
  configParam.pidRate = configParamDefault.pidRate;
  configParam.pidPos = configParamDefault.pidPos;
}

void getConfigParam(configParam_t *configParam_temp) {
  while (!isConfigParamOK) {
    rt_thread_mdelay(500);
  }
  if (configParam_temp != NULL) {
    *configParam_temp = configParam;
  }
}