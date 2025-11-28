#include "acc_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "acc_init"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
#include "timestamp.h"
#include "../common/inc/init_sync.h"
#include "sensor_alignment.h"  // For sensor_align_e and sensorAlignment_t
}

// 从参数系统加载对齐参数并初始化
void AccBf::loadAlignmentFromParams() {
  uint8_t align_value = 0;
  int16_t custom_align[3] = {0, 0, 0};

  if (getParam("imu_align_method", &align_value, sizeof(align_value)) == RT_EOK) {
    sensor_align_e align = static_cast<sensor_align_e>(align_value);

    if (align == ALIGN_CUSTOM) {
      // 如果是自定义对齐，读取自定义对齐角度
      if (getParam("imu_custom_align", custom_align, sizeof(custom_align)) == RT_EOK) {
        sensorAlignment_t custom_alignment;
        custom_alignment.roll = custom_align[0];
        custom_alignment.pitch = custom_align[1];
        custom_alignment.yaw = custom_align[2];
        setAlignment(align, &custom_alignment);
        LOG_I("Loaded acc alignment: ALIGN_CUSTOM (roll=%d, pitch=%d, yaw=%d decidegrees)", custom_align[0],
              custom_align[1], custom_align[2]);
      } else {
        // 自定义对齐参数不存在，使用默认对齐
        setAlignment(ALIGN_DEFAULT);
        LOG_W("Acc alignment set to ALIGN_CUSTOM but custom_align not found, using ALIGN_DEFAULT");
      }
    } else {
      // 标准对齐方式
      setAlignment(align);
      LOG_I("Loaded acc alignment: %d", align);
    }
  } else {
    // 参数不存在，使用默认对齐
    setAlignment(ALIGN_DEFAULT);
    LOG_I("Acc alignment parameter not found, using ALIGN_DEFAULT");
  }
}

