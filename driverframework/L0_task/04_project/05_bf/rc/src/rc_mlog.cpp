#include "rc_class.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "rc_mlog"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
}

#include "rc_mlog.h"

namespace bf_mlog {

// RC 数据结构（对应 mlog 总线定义）
struct mlogRcData_t {
  uint32_t timestamp;
  uint32_t seq;
  float raw_channels[6];      // 前 6 个通道的原始数据 [ch0-ch5] (1000-2000)
  float rawSetpoint[3];       // Raw setpoint rates [roll, pitch, yaw] (deg/s)
  float rcCommandThrottle;    // RC command throttle (1000-2000)
  float rx_rate_hz;           // RC refresh rate (Hz)
  float throttle_cutoff;      // Throttle filter cutoff frequency (Hz)
  uint8_t armed;              // 上锁状态 (0=DISARMED, 1=ARMED)
  uint8_t flight_mode;        // 飞行模式 (0=角速度模式/Rate, 1=角度模式/Angle, 2=高度模式/Altitude)
} __packed;

// Mlog 元素定义（参考 aMlogStabilze.c:81-85）
static mlog_elem_t RcData_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(seq, MLOG_UINT32),
    MLOG_ELEMENT_VEC(raw_channels, MLOG_FLOAT, 6),
    MLOG_ELEMENT_VEC(rawSetpoint, MLOG_FLOAT, 3),
    MLOG_ELEMENT(rcCommandThrottle, MLOG_FLOAT),
    MLOG_ELEMENT(rx_rate_hz, MLOG_FLOAT),
    MLOG_ELEMENT(throttle_cutoff, MLOG_FLOAT),
    MLOG_ELEMENT(armed, MLOG_UINT8),
    MLOG_ELEMENT(flight_mode, MLOG_UINT8),
};

// Mlog 总线定义（参考 aMlogStabilze.c:86）
MLOG_BUS_DEFINE(RcData, RcData_Elems);

// 单例实例（用于回调）
MlogRc* MlogRc::instance_ = nullptr;

MlogRc::MlogRc() : bus_id_(-1), enabled_(false), param_enabled_(false) { instance_ = this; }

MlogRc::~MlogRc() { instance_ = nullptr; }

MlogRc* MlogRc::getInstance() {
  static MlogRc instance;
  return &instance;
}

void MlogRc::init() {
  // 获取 mlog 总线 ID（参考 aMlogStabilze.c:123）
  bus_id_ = mlog_get_bus_id("RcData");
  if (bus_id_ < 0) {
    LOG_I("[MlogRc] Failed to get mlog bus ID for RcData\n");
  } else {
    LOG_I("[MlogRc] RcData mlog bus ID: %d\n", bus_id_);
  }

  // 注册 mlog 开始回调（参考 aMlogStabilze.c:175）
  mlog_register_callback(MLOG_CB_START, startCallback);
}

void MlogRc::startCallback() {
  // 当 mlog 开始时启用推送（参考 aMlogStabilze.c:187）
  // 但是需要检查参数使能状态，只有参数使能时才真正启用推送
  if (instance_ != nullptr) {
    // 如果参数已使能，则设置 enabled_ 为 true
    // 否则保持 enabled_ 为 false，这样 pushRcData 会被跳过
    if (instance_->param_enabled_) {
      instance_->enabled_ = true;
    } else {
      instance_->enabled_ = false;
    }
  }
}

void MlogRc::pushRcData(const rc_mlog_data_t* data) {
  if (data == nullptr) {
    return;
  }

  // 检查是否启用推送（参考 aMlogStabilze.c:210）
  // 必须同时满足：bus_id_ 有效、mlog 系统已启用（enabled_）、参数已使能（param_enabled_）
  if (bus_id_ < 0 || !isEnabled()) {
    return;
  }
  
  static uint32_t timestamp_last = 0;
  if (data->timestamp <= timestamp_last) {
    static uint16_t count = 0;
    LOG_E("[MlogRc] Non-monotonic timestamp: current=%u, last=%u, count=%u", data->timestamp, timestamp_last, count);
    count++;
  }
  timestamp_last = data->timestamp;
  
  // 填充数据结构
  mlogRcData_t rc_data = {0};
  rc_data.timestamp = data->timestamp;
  rc_data.seq = data->seq;
  std::memcpy(rc_data.raw_channels, data->raw_channels, sizeof(rc_data.raw_channels));
  std::memcpy(rc_data.rawSetpoint, data->rawSetpoint, sizeof(rc_data.rawSetpoint));
  rc_data.rcCommandThrottle = data->rcCommandThrottle;
  rc_data.rx_rate_hz = data->rx_rate_hz;
  rc_data.throttle_cutoff = data->throttle_cutoff;
  rc_data.armed = data->armed;
  rc_data.flight_mode = data->flight_mode;

  // 推送消息到 mlog（参考 aMlogStabilze.c:211）
  mlog_push_msg(reinterpret_cast<const uint8_t*>(&rc_data), bus_id_, sizeof(mlogRcData_t));
}

}  // namespace bf_mlog

// 初始化 mlog_rc
rt_err_t RcBf::initMlog() {
  // 初始化 mlog_rc（使用单例）
  bf_mlog::MlogRc* mlog_rc = bf_mlog::MlogRc::getInstance();
  mlog_rc->init();
  
  // 从参数系统读取 mlog_rc_en 参数并设置使能状态
  uint8_t mlog_rc_en = 0;
  if (getParam("mlog_rc_en", &mlog_rc_en, sizeof(mlog_rc_en)) == RT_EOK) {
    mlog_rc->setParamEnabled(mlog_rc_en != 0);
    LOG_I("Mlog RC enabled: %u", mlog_rc_en);
  } else {
    // 如果参数不存在，使用默认值（禁用）
    mlog_rc->setParamEnabled(false);
    LOG_W("Mlog RC parameter not found, disabled by default");
  }

  return RT_EOK;
}

// 推送 RC 数据到 mlog
void RcBf::pushRcDataToMlog(const bf_mlog::rc_mlog_data_t* data) {
  if (data == nullptr) {
    return;
  }

  // 使用结构体参数直接推送
  bf_mlog::MlogRc::getInstance()->pushRcData(data);
}

