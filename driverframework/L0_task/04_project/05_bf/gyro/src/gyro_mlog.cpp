#include "gyro_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "gyro_mlog"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
#include "timestamp.h"
}

#include "gyro_mlog.h"

namespace bf_mlog {

// 陀螺仪数据结构（对应 mlog 总线定义）
struct mlogGyroData_t {
  uint32_t timestamp;
  uint32_t seq;
  float gyro_adc[3];       // 对齐、校准但未滤波的数据（对应 gyro_adc_）
  float gyro_filtered[3];  // 滤波后的陀螺仪数据（对应 gyro_adcf_）
} __packed;

// Mlog 元素定义（参考 aMlogStabilze.c:81-85）
static mlog_elem_t GyroData_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(seq, MLOG_UINT32),
    MLOG_ELEMENT_VEC(gyro_adc, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filtered, MLOG_FLOAT, 3),
};

// Mlog 总线定义（参考 aMlogStabilze.c:86）
MLOG_BUS_DEFINE(GyroData, GyroData_Elems);

// 单例实例（用于回调）
MlogGyro* MlogGyro::instance_ = nullptr;

MlogGyro::MlogGyro() : bus_id_(-1), enabled_(false), param_enabled_(false) { instance_ = this; }

MlogGyro::~MlogGyro() { instance_ = nullptr; }

MlogGyro* MlogGyro::getInstance() {
  static MlogGyro instance;
  return &instance;
}

void MlogGyro::init() {
  // 获取 mlog 总线 ID（参考 aMlogStabilze.c:123）
  bus_id_ = mlog_get_bus_id("GyroData");
  if (bus_id_ < 0) {
    LOG_I("[MlogGyro] Failed to get mlog bus ID for GyroData\n");
  } else {
    LOG_I("[MlogGyro] GyroData mlog bus ID: %d\n", bus_id_);
  }

  // 注册 mlog 开始回调（参考 aMlogStabilze.c:175）
  mlog_register_callback(MLOG_CB_START, startCallback);
}

void MlogGyro::startCallback() {
  // 当 mlog 开始时启用推送（参考 aMlogStabilze.c:187）
  // 但是需要检查参数使能状态，只有参数使能时才真正启用推送
  if (instance_ != nullptr) {
    // 如果参数已使能，则设置 enabled_ 为 true
    // 否则保持 enabled_ 为 false，这样 pushGyroData 会被跳过
    if (instance_->param_enabled_) {
      instance_->enabled_ = true;
    } else {
      instance_->enabled_ = false;
    }
  }
}

void MlogGyro::pushGyroData(const gyro_mlog_data_t* data) {
  if (data == nullptr) {
    return;
  }

  // 检查是否启用推送（参考 aMlogStabilze.c:210）
  // 必须同时满足：bus_id_ 有效、mlog 系统已启用（enabled_）、参数已使能（param_enabled_）
  if (bus_id_ < 0 || !isEnabled()) {
    return;
  }
  
  static uint32_t timestamp_last = 0;
  // Check for non-monotonic timestamp, but allow small backward jumps (< 1ms) due to precision issues
  // Also handle timestamp wrap-around (32-bit uint wraps every ~71.6 minutes)
  if (data->timestamp <= timestamp_last) {
    uint32_t diff = timestamp_last - data->timestamp;
    // Only warn if the backward jump is significant (> 1ms) and not a wrap-around
    // Wrap-around: if last is close to max (within 1 second) and current is small (< 1 second)
    if (diff > 1000 && !(timestamp_last > (UINT32_MAX - 1000000) && data->timestamp < 1000000)) {
      static uint16_t count = 0;
      LOG_E("[MlogGyro] Non-monotonic timestamp: current=%u, last=%u, diff=%u us, count=%u", 
            data->timestamp, timestamp_last, diff, count);
      count++;
    }
  }
  timestamp_last = data->timestamp;
  
  // 填充数据结构
  mlogGyroData_t gyro_data = {0};
  gyro_data.timestamp = data->timestamp;
  gyro_data.seq = data->seq;
  std::memcpy(gyro_data.gyro_adc, data->gyro_adc, sizeof(gyro_data.gyro_adc));
  std::memcpy(gyro_data.gyro_filtered, data->gyro_filtered, sizeof(gyro_data.gyro_filtered));

  // 推送消息到 mlog（参考 aMlogStabilze.c:211）
  mlog_push_msg(reinterpret_cast<const uint8_t*>(&gyro_data), bus_id_, sizeof(mlogGyroData_t));
}

}  // namespace bf_mlog

// 初始化 mlog_gyro
rt_err_t gyro::initMlog() {
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
void gyro::pushGyroDataToMlog(const bf_mlog::gyro_mlog_data_t* data) {
  if (data == nullptr) {
    return;
  }

  // 使用结构体参数直接推送
  bf_mlog::MlogGyro::getInstance()->pushGyroData(data);
}
