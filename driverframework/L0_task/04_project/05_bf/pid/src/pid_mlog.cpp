#include "pid_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_mlog"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
}

#include "pid_mlog.h"

namespace bf_mlog {

// PID 数据结构（对应 mlog 总线定义）
struct mlogPidData_t {
  uint32_t timestamp;
  uint32_t seq;
  
  // 角速度环数据（Rate loop）
  float rate_setpoint[3];  // 角速度期望值 [roll, pitch, yaw] (deg/s)
  float rate_actual[3];    // 角速度当前值 [roll, pitch, yaw] (deg/s)
  
  // 角度环数据（Angle loop）
  float angle_setpoint[2]; // 角度期望值 [roll, pitch] (degrees)
  float angle_actual[2];   // 角度当前值 [roll, pitch] (degrees)
  
  // 滤波后的油门值（来自RC平滑滤波器）
  float smoothed_throttle; // 滤波后的油门值 (1000-2000)
} __packed;

// Mlog 元素定义（参考 aMlogStabilze.c:81-85）
static mlog_elem_t PidData_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(seq, MLOG_UINT32),
    MLOG_ELEMENT_VEC(rate_setpoint, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(rate_actual, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(angle_setpoint, MLOG_FLOAT, 2),
    MLOG_ELEMENT_VEC(angle_actual, MLOG_FLOAT, 2),
    MLOG_ELEMENT(smoothed_throttle, MLOG_FLOAT),
};

// Mlog 总线定义（参考 aMlogStabilze.c:86）
MLOG_BUS_DEFINE(PidData, PidData_Elems);

// 单例实例（用于回调）
MlogPid* MlogPid::instance_ = nullptr;

MlogPid::MlogPid() : bus_id_(-1), enabled_(false), param_enabled_(false) { instance_ = this; }

MlogPid::~MlogPid() { instance_ = nullptr; }

MlogPid* MlogPid::getInstance() {
  static MlogPid instance;
  return &instance;
}

void MlogPid::init() {
  // 获取 mlog 总线 ID（参考 aMlogStabilze.c:123）
  bus_id_ = mlog_get_bus_id("PidData");
  if (bus_id_ < 0) {
    LOG_I("[MlogPid] Failed to get mlog bus ID for PidData\n");
  } else {
    LOG_I("[MlogPid] PidData mlog bus ID: %d\n", bus_id_);
  }

  // 注册 mlog 开始回调（参考 aMlogStabilze.c:175）
  mlog_register_callback(MLOG_CB_START, startCallback);
}

void MlogPid::startCallback() {
  // 当 mlog 开始时启用推送（参考 aMlogStabilze.c:187）
  // 但是需要检查参数使能状态，只有参数使能时才真正启用推送
  if (instance_ != nullptr) {
    // 如果参数已使能，则设置 enabled_ 为 true
    // 否则保持 enabled_ 为 false，这样 pushPidData 会被跳过
    if (instance_->param_enabled_) {
      instance_->enabled_ = true;
    } else {
      instance_->enabled_ = false;
    }
  }
}

void MlogPid::pushPidData(const pid_mlog_data_t* data) {
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
    LOG_E("[MlogPid] Non-monotonic timestamp: current=%u, last=%u, count=%u", data->timestamp, timestamp_last, count);
    count++;
  }
  timestamp_last = data->timestamp;
  
  // 填充数据结构
  mlogPidData_t pid_data = {0};
  pid_data.timestamp = data->timestamp;
  pid_data.seq = data->seq;
  std::memcpy(pid_data.rate_setpoint, data->rate_setpoint, sizeof(pid_data.rate_setpoint));
  std::memcpy(pid_data.rate_actual, data->rate_actual, sizeof(pid_data.rate_actual));
  std::memcpy(pid_data.angle_setpoint, data->angle_setpoint, sizeof(pid_data.angle_setpoint));
  std::memcpy(pid_data.angle_actual, data->angle_actual, sizeof(pid_data.angle_actual));
  pid_data.smoothed_throttle = data->smoothed_throttle;

  // 推送消息到 mlog（参考 aMlogStabilze.c:211）
  mlog_push_msg(reinterpret_cast<const uint8_t*>(&pid_data), bus_id_, sizeof(mlogPidData_t));
}

}  // namespace bf_mlog

// 初始化 mlog_pid
rt_err_t PidBf::initMlog() {
  // 初始化 mlog_pid（使用单例）
  bf_mlog::MlogPid* mlog_pid = bf_mlog::MlogPid::getInstance();
  mlog_pid->init();
  
  // 从参数系统读取 mlog_pid_en 参数并设置使能状态
  uint8_t mlog_pid_en = 0;
  if (getParam("mlog_pid_en", &mlog_pid_en, sizeof(mlog_pid_en)) == RT_EOK) {
    mlog_pid->setParamEnabled(mlog_pid_en != 0);
    LOG_I("Mlog PID enabled: %u", mlog_pid_en);
  } else {
    // 如果参数不存在，使用默认值（禁用）
    mlog_pid->setParamEnabled(false);
    LOG_W("Mlog PID parameter not found, disabled by default");
  }

  return RT_EOK;
}

// 推送 PID 数据到 mlog
void PidBf::pushPidDataToMlog(const bf_mlog::pid_mlog_data_t* data) {
  if (data == nullptr) {
    return;
  }

  // 使用结构体参数直接推送
  bf_mlog::MlogPid::getInstance()->pushPidData(data);
}

