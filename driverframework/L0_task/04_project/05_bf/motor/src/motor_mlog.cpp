#include "motor_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "motor_mlog"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

// Optional dependency: PARAM module
#ifdef PROJECT_BF_PARAM_EN
extern "C" {
#include "param.h"
}
#endif

#include "motor_mlog.h"

#ifdef PROJECT_BF_MOTOR_MLOG_EN

namespace bf_mlog {

// Motor 数据结构（对应 mlog 总线定义）
struct mlogMotorData_t {
  uint32_t timestamp;
  uint32_t seq;
  uint16_t motor_values[4];  // 最终下发给 dshot 的 4 个通道数据 (16bit uint16)
} __packed;

// Mlog 元素定义（参考 aMlogStabilze.c:81-85）
static mlog_elem_t MotorData_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(seq, MLOG_UINT32),
    MLOG_ELEMENT_VEC(motor_values, MLOG_UINT16, 4),
};

// Mlog 总线定义（参考 aMlogStabilze.c:86）
MLOG_BUS_DEFINE(MotorData, MotorData_Elems);

// 单例实例（用于回调）
MlogMotor* MlogMotor::instance_ = nullptr;

MlogMotor::MlogMotor() : bus_id_(-1), enabled_(false), param_enabled_(false) { instance_ = this; }

MlogMotor::~MlogMotor() { instance_ = nullptr; }

MlogMotor* MlogMotor::getInstance() {
  static MlogMotor instance;
  return &instance;
}

void MlogMotor::init() {
  // 获取 mlog 总线 ID（参考 aMlogStabilze.c:123）
  bus_id_ = mlog_get_bus_id("MotorData");
  if (bus_id_ < 0) {
    LOG_I("[MlogMotor] Failed to get mlog bus ID for MotorData\n");
  } else {
    LOG_I("[MlogMotor] MotorData mlog bus ID: %d\n", bus_id_);
  }

  // 注册 mlog 开始回调（参考 aMlogStabilze.c:175）
  mlog_register_callback(MLOG_CB_START, startCallback);
}

void MlogMotor::startCallback() {
  // 当 mlog 开始时启用推送（参考 aMlogStabilze.c:187）
  // 但是需要检查参数使能状态，只有参数使能时才真正启用推送
  if (instance_ != nullptr) {
    // 如果参数已使能，则设置 enabled_ 为 true
    // 否则保持 enabled_ 为 false，这样 pushMotorData 会被跳过
    if (instance_->param_enabled_) {
      instance_->enabled_ = true;
    } else {
      instance_->enabled_ = false;
    }
  }
}

void MlogMotor::pushMotorData(const motor_mlog_data_t* data) {
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
    LOG_E("[MlogMotor] Non-monotonic timestamp: current=%u, last=%u, count=%u", data->timestamp, timestamp_last, count);
    count++;
  }
  timestamp_last = data->timestamp;
  
  // 填充数据结构
  mlogMotorData_t motor_data = {0};
  motor_data.timestamp = data->timestamp;
  motor_data.seq = data->seq;
  std::memcpy(motor_data.motor_values, data->motor_values, sizeof(motor_data.motor_values));

  // 推送消息到 mlog（参考 aMlogStabilze.c:211）
  mlog_push_msg(reinterpret_cast<const uint8_t*>(&motor_data), bus_id_, sizeof(mlogMotorData_t));
}

}  // namespace bf_mlog

#endif  // PROJECT_BF_MOTOR_MLOG_EN

// 初始化 mlog_motor
rt_err_t MotorBf::initMlog() {
#ifdef PROJECT_BF_MOTOR_MLOG_EN
  // 初始化 mlog_motor（使用单例）
  bf_mlog::MlogMotor* mlog_motor = bf_mlog::MlogMotor::getInstance();
  mlog_motor->init();
  
  // 从参数系统读取 mlog_motor_en 参数并设置使能状态
  uint8_t mlog_motor_en = 0;
#ifdef PROJECT_BF_PARAM_EN
  if (getParam("mlog_motor_en", &mlog_motor_en, sizeof(mlog_motor_en)) == RT_EOK) {
    mlog_motor->setParamEnabled(mlog_motor_en != 0);
    LOG_I("Mlog motor enabled: %u", mlog_motor_en);
  } else {
    // 如果参数不存在，使用默认值（禁用）
    mlog_motor->setParamEnabled(false);
    LOG_W("Mlog motor parameter not found, disabled by default");
  }
#else
  // PARAM module not available, use default (disabled)
  mlog_motor->setParamEnabled(false);
  LOG_I("PARAM module not available, mlog motor disabled by default");
#endif
#else
  // MLOG module not available, do nothing
  LOG_I("MLOG module not available, mlog motor disabled");
#endif

  return RT_EOK;
}

// 推送 Motor 数据到 mlog
void MotorBf::pushMotorDataToMlog(const motor_mlog_data_t* data) {
  if (data == nullptr) {
    return;
  }

#ifdef PROJECT_BF_MOTOR_MLOG_EN
  // 使用结构体参数直接推送
  bf_mlog::MlogMotor::getInstance()->pushMotorData(data);
#else
  // MLOG module not available, do nothing
  (void)data;  // Suppress unused parameter warning
#endif
}

