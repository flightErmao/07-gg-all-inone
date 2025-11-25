#include "gyro_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "gyro_mlog"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
}

#include "../log/inc/mlog_gyro.hpp"

extern "C" {
#include "timestamp.h"
}

// 初始化 mlog_gyro
rt_err_t RateCtrlAngularVelocity::initMlog() {
  // 初始化 mlog_gyro（使用单例）
  bf_mlog::MlogGyro* mlog_gyro = bf_mlog::MlogGyro::getInstance();
  mlog_gyro->init();
  
  // 从参数系统读取 mlog_gyro_en 参数并设置使能状态
  uint8_t mlog_gyro_en = 0;
  if (getParam("mlog_gyro_en", &mlog_gyro_en, sizeof(mlog_gyro_en)) == RT_EOK) {
    mlog_gyro->setParamEnabled(mlog_gyro_en != 0);
    LOG_I("Mlog gyro enabled: %u", mlog_gyro_en);
  } else {
    // 如果参数不存在，使用默认值（禁用）
    mlog_gyro->setParamEnabled(false);
    LOG_W("Mlog gyro parameter not found, disabled by default");
  }

  return RT_EOK;
}

// 推送陀螺仪数据到 mlog（参考 aMlogStabilze.c:208-216）
void RateCtrlAngularVelocity::pushGyroDataToMlog(const imu_raw_msg_t* imu_data) {
  if (imu_data == nullptr) {
    return;
  }

  // 记录滤波前后的陀螺仪数据
  uint32_t timestamp = timestamp_micros();
  bf_mlog::MlogGyro::getInstance()->pushGyroData(imu_data->seq, timestamp, gyro_adc_, gyro_adcf_);
}

