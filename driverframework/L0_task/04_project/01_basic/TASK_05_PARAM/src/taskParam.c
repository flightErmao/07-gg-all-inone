#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "taskParam.h"
#include "configParamDefault.h"

#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
#include <fal.h>
#endif

#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 5
static rt_sem_t configParam_sem = RT_NULL;
static configParam_t configParam;
static bool isInit = false;
static bool isConfigParamOK = false;

#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
static const struct fal_partition *param_partition = RT_NULL;
#endif

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
          rt_kprintf("Read data: version=%d, trimP=%.2f, trimR=%.2f, thrustBase=%d\r\n", configParam.version,
                     configParam.trimP, configParam.trimR, configParam.thrustBase);
          isConfigParamOK = true;
        } else {
          rt_kprintf("Checksum check [FAIL] - expected: %d, actual: %d\r\n", configParam.cksum,
                     configParamCksum(&configParam));
          isConfigParamOK = false;
        }
      } else {
        rt_kprintf("Version check [FAIL] - expected: %d, actual: %d\r\n", VERSION, configParam.version);
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
      rt_enter_critical();
      int ret = fal_partition_erase(param_partition, 0, param_partition->len);
      if (ret < 0) {
        rt_kprintf("FAL partition erase failed\r\n");
      } else {
        rt_kprintf("Writing data: version=%d, trimP=%.2f, trimR=%.2f, thrustBase=%d, cksum=%d\r\n", configParam.version,
                   configParam.trimP, configParam.trimR, configParam.thrustBase, configParam.cksum);
        ret = fal_partition_write(param_partition, 0, (const rt_uint8_t *)&configParam, sizeof(configParam));
        if (ret < 0) {
          rt_kprintf("FAL partition write failed\r\n");
        } else {
          rt_kprintf("FAL partition write success, size: %d bytes\r\n", sizeof(configParam));
        }
      }
      rt_exit_critical();
    }
#endif
    isConfigParamOK = true;
  }

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
    if (rt_sem_take(configParam_sem, RT_WAITING_FOREVER) == RT_EOK) {
      cksum = configParamCksum(&configParam);

      if (configParam.cksum != cksum) {
        configParam.cksum = cksum;
#ifdef PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
        if (param_partition != RT_NULL) {
          rt_enter_critical();
          int ret = fal_partition_erase(param_partition, 0, param_partition->len);
          if (ret < 0) {
            rt_kprintf("FAL partition erase failed\r\n");
          } else {
            rt_kprintf("Updating data: version=%d, trimP=%.2f, trimR=%.2f, thrustBase=%d, cksum=%d\r\n",
                       configParam.version, configParam.trimP, configParam.trimR, configParam.thrustBase,
                       configParam.cksum);
            ret = fal_partition_write(param_partition, 0, (const rt_uint8_t *)&configParam, sizeof(configParam));
            if (ret < 0) {
              rt_kprintf("FAL partition write failed\r\n");
            } else {
              rt_kprintf("FAL partition update success, size: %d bytes\r\n", sizeof(configParam));
            }
          }
          rt_exit_critical();
        }
#endif
      }
    }
  }
}

static int configParamTaskInit(void) {
  static struct rt_thread configParamTid;
  static rt_uint8_t configParamStack[THREAD_STACK_SIZE];

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

void updateConfigAnglePID(pidParam_t pidParam_temp) { configParam.pidAngle = pidParam_temp; }

void updateConfigRatePID(pidParam_t pidParam_temp) { configParam.pidRate = pidParam_temp; }
