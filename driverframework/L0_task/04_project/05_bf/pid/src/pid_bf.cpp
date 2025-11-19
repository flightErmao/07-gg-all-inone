#include "pid_bf.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "pid_setpoint_msg.h"
#include "pid_output_msg.h"
#include "bfPidParam.h"
}

#include <cstring>
#include <cmath>

// PID constants from ref/pid.h
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f
#define PIDSUM_LIMIT 500
#define PIDSUM_LIMIT_YAW 400
#define ANTIGRAVITY_KI 0.34f
#define ANTIGRAVITY_KP 0.0034f

// Axis indices
#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define XYZ_AXIS_COUNT 3

// Helper macros
#define constrainf(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define fabsf(x) ((x) < 0.0f ? -(x) : (x))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* 定义 PID 消息 MCN（在本文件内完成定义与发布） */
MCN_DEFINE(pid_setpoint, sizeof(pid_setpoint_msg_t));
MCN_DEFINE(pid_output, sizeof(pid_output_msg_t));

// 线程配置（从 Kconfig 获取，如果没有定义则使用默认值）
#ifndef CONFIG_PROJECT_BF_PID_THREAD_STACK_SIZE
#define CONFIG_PROJECT_BF_PID_THREAD_STACK_SIZE 4096
#endif

#ifndef CONFIG_PROJECT_BF_PID_THREAD_PRIORITY
#define CONFIG_PROJECT_BF_PID_THREAD_PRIORITY 8
#endif

#ifndef CONFIG_PROJECT_BF_PID_THREAD_TIMESLICE
#define CONFIG_PROJECT_BF_PID_THREAD_TIMESLICE 5
#endif

// PidBf 单例实现
PidBf& PidBf::instance() {
  static PidBf instance_obj;
  return instance_obj;
}

PidBf::PidBf()
    : thread_(RT_NULL),
      thread_inited_(false),
      gyro_filtered_event_(RT_NULL),
      gyro_filtered_node_(RT_NULL),
      setpoint_event_(RT_NULL),
      setpoint_node_(RT_NULL),
      pid_output_hub_(nullptr),
      gyro_data_ready_(false),
      setpoint_data_ready_(false),
      target_looptime_us_(0) {
  std::memset(&thread_obj_, 0, sizeof(thread_obj_));
  std::memset(thread_stack_, 0, sizeof(thread_stack_));
  std::memset(&gyro_filtered_data_, 0, sizeof(gyro_filtered_data_));
  std::memset(&setpoint_data_, 0, sizeof(setpoint_data_));
  std::memset(&pid_runtime_, 0, sizeof(pid_runtime_));
  std::memset(pid_data_, 0, sizeof(pid_data_));
  std::memset(&pid_profile_, 0, sizeof(pid_profile_));

  // Initialize PID runtime defaults
  pid_runtime_.pidStabilisationEnabled = true;
  pid_runtime_.dT = 0.0f;
  pid_runtime_.pidFrequency = 0.0f;
  for (int i = 0; i < 3; i++) {
    pid_runtime_.previousPidSetpoint[i] = 0.0f;
  }
}

PidBf::~PidBf() {
  // Cleanup if needed
}

rt_err_t PidBf::init() {
  if (thread_inited_) {
    LOG_W("PidBf already initialized");
    return RT_EOK;
  }

  // 创建 MCN 事件信号量（用于 mcn_poll_sync）
  if (gyro_filtered_event_ == RT_NULL) {
    gyro_filtered_event_ = rt_sem_create("pid_gyro_evt", 0, RT_IPC_FLAG_FIFO);
    if (gyro_filtered_event_ == RT_NULL) {
      LOG_E("create gyro_filtered event semaphore failed");
      return -RT_ERROR;
    }
  }

  // 订阅 gyro_filtered MCN 节点
  gyro_filtered_node_ = mcn_subscribe(MCN_HUB(gyro_filtered), gyro_filtered_event_, RT_NULL);
  if (gyro_filtered_node_ == RT_NULL) {
    LOG_E("subscribe gyro_filtered topic failed");
    if (gyro_filtered_event_ != RT_NULL) {
      rt_sem_delete(gyro_filtered_event_);
      gyro_filtered_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to gyro_filtered MCN topic");

  // 订阅 setpoint MCN 节点（暂时创建，等待后续发布者）
  if (setpoint_event_ == RT_NULL) {
    setpoint_event_ = rt_sem_create("pid_setpoint_evt", 0, RT_IPC_FLAG_FIFO);
    if (setpoint_event_ == RT_NULL) {
      LOG_E("create setpoint event semaphore failed");
      // Continue anyway, setpoint might not be available yet
    }
  }

  // 尝试订阅 setpoint（如果不存在，稍后重试）
  setpoint_node_ = mcn_subscribe(MCN_HUB(pid_setpoint), setpoint_event_, RT_NULL);
  if (setpoint_node_ == RT_NULL) {
    LOG_W("pid_setpoint topic not available yet, will use default setpoint");
  } else {
    LOG_I("Subscribed to pid_setpoint MCN topic");
  }

  // 获取 pid_output MCN hub（用于发布 PID 输出数据）
  pid_output_hub_ = MCN_HUB(pid_output);
  if (pid_output_hub_ == nullptr) {
    LOG_E("get pid_output hub failed");
    if (gyro_filtered_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(gyro_filtered), gyro_filtered_node_);
      gyro_filtered_node_ = RT_NULL;
    }
    if (gyro_filtered_event_ != RT_NULL) {
      rt_sem_delete(gyro_filtered_event_);
      gyro_filtered_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  // 读取 pid_process_denom 参数并设置目标循环时间
  uint8_t pid_process_denom = 1;  // 默认值
  if (getParam("pid_process_denom", &pid_process_denom, sizeof(pid_process_denom)) != RT_EOK) {
    pid_process_denom = 1;
  }

  // 从 rateCtrl 获取目标循环时间（通过订阅 gyro_filtered 消息获取）
  // 暂时使用默认值，后续可以从消息中获取
  // 假设 gyro_filtered 的发布频率是 8000Hz，pid_process_denom=1 时 target_looptime = 125us
  // 这里先设置一个合理的默认值
  target_looptime_us_ = 125;  // 8kHz = 125us
  if (pid_process_denom > 1) {
    target_looptime_us_ *= pid_process_denom;
  }

  pid_runtime_.dT = target_looptime_us_ * 1e-6f;
  pid_runtime_.pidFrequency = 1.0f / pid_runtime_.dT;

  LOG_I("Target looptime: %u us, dT: %.6f s, frequency: %.1f Hz", target_looptime_us_, pid_runtime_.dT,
        pid_runtime_.pidFrequency);

  // 初始化 PID 配置（从参数系统读取）
  initConfig();

  // 初始化 PID 滤波器（暂时简化，后续可以添加）
  initFilters();

  // 创建静态线程
  rt_err_t err = rt_thread_init(&thread_obj_, "pid_bf", PidBf::threadEntry, this, thread_stack_,
                                CONFIG_PROJECT_BF_PID_THREAD_STACK_SIZE, CONFIG_PROJECT_BF_PID_THREAD_PRIORITY,
                                CONFIG_PROJECT_BF_PID_THREAD_TIMESLICE);

  if (err != RT_EOK) {
    LOG_E("PidBf thread init failed: %d", err);
    if (gyro_filtered_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(gyro_filtered), gyro_filtered_node_);
      gyro_filtered_node_ = RT_NULL;
    }
    if (gyro_filtered_event_ != RT_NULL) {
      rt_sem_delete(gyro_filtered_event_);
      gyro_filtered_event_ = RT_NULL;
    }
    return err;
  }

  thread_ = &thread_obj_;
  thread_inited_ = true;

  // 启动线程
  rt_thread_startup(thread_);
  LOG_I("PidBf thread started");

  LOG_I("PidBf initialized");
  return RT_EOK;
}

void PidBf::initConfig() {
  // 从参数系统读取 PID 参数
  float pid_rate_roll[3] = {0};
  float pid_rate_pitch[3] = {0};
  float pid_rate_yaw[3] = {0};

  if (getParam("pid_rate_roll", pid_rate_roll, sizeof(pid_rate_roll)) == RT_EOK) {
    pid_profile_.pid[FD_ROLL][0] = pid_rate_roll[0] * PTERM_SCALE;  // P
    pid_profile_.pid[FD_ROLL][1] = pid_rate_roll[1] * ITERM_SCALE;  // I
    pid_profile_.pid[FD_ROLL][2] = pid_rate_roll[2] * DTERM_SCALE;  // D
    LOG_I("PID Roll: P=%.2f, I=%.2f, D=%.2f", pid_rate_roll[0], pid_rate_roll[1], pid_rate_roll[2]);
  } else {
    // 使用默认值
    pid_profile_.pid[FD_ROLL][0] = 45.0f * PTERM_SCALE;
    pid_profile_.pid[FD_ROLL][1] = 80.0f * ITERM_SCALE;
    pid_profile_.pid[FD_ROLL][2] = 30.0f * DTERM_SCALE;
    LOG_W("Using default PID Roll values");
  }

  if (getParam("pid_rate_pitch", pid_rate_pitch, sizeof(pid_rate_pitch)) == RT_EOK) {
    pid_profile_.pid[FD_PITCH][0] = pid_rate_pitch[0] * PTERM_SCALE;
    pid_profile_.pid[FD_PITCH][1] = pid_rate_pitch[1] * ITERM_SCALE;
    pid_profile_.pid[FD_PITCH][2] = pid_rate_pitch[2] * DTERM_SCALE;
    LOG_I("PID Pitch: P=%.2f, I=%.2f, D=%.2f", pid_rate_pitch[0], pid_rate_pitch[1], pid_rate_pitch[2]);
  } else {
    pid_profile_.pid[FD_PITCH][0] = 47.0f * PTERM_SCALE;
    pid_profile_.pid[FD_PITCH][1] = 84.0f * ITERM_SCALE;
    pid_profile_.pid[FD_PITCH][2] = 34.0f * DTERM_SCALE;
    LOG_W("Using default PID Pitch values");
  }

  if (getParam("pid_rate_yaw", pid_rate_yaw, sizeof(pid_rate_yaw)) == RT_EOK) {
    pid_profile_.pid[FD_YAW][0] = pid_rate_yaw[0] * PTERM_SCALE;
    pid_profile_.pid[FD_YAW][1] = pid_rate_yaw[1] * ITERM_SCALE;
    pid_profile_.pid[FD_YAW][2] = pid_rate_yaw[2] * DTERM_SCALE;
    LOG_I("PID Yaw: P=%.2f, I=%.2f, D=%.2f", pid_rate_yaw[0], pid_rate_yaw[1], pid_rate_yaw[2]);
  } else {
    pid_profile_.pid[FD_YAW][0] = 45.0f * PTERM_SCALE;
    pid_profile_.pid[FD_YAW][1] = 80.0f * ITERM_SCALE;
    pid_profile_.pid[FD_YAW][2] = 0.0f * DTERM_SCALE;
    LOG_W("Using default PID Yaw values");
  }

  // 读取 PID sum limit
  float pid_rate_roll_i_limit = 100.0f;
  float pid_rate_pitch_i_limit = 100.0f;
  float pid_rate_yaw_i_limit = 100.0f;

  if (getParam("pid_rate_roll_i_limit", &pid_rate_roll_i_limit, sizeof(pid_rate_roll_i_limit)) == RT_EOK) {
    pid_profile_.pidSumLimit = pid_rate_roll_i_limit;
  } else {
    pid_profile_.pidSumLimit = PIDSUM_LIMIT;
  }

  if (getParam("pid_rate_pitch_i_limit", &pid_rate_pitch_i_limit, sizeof(pid_rate_pitch_i_limit)) == RT_EOK) {
    // Use the same limit for pitch
  }

  if (getParam("pid_rate_yaw_i_limit", &pid_rate_yaw_i_limit, sizeof(pid_rate_yaw_i_limit)) == RT_EOK) {
    pid_profile_.pidSumLimitYaw = pid_rate_yaw_i_limit;
  } else {
    pid_profile_.pidSumLimitYaw = PIDSUM_LIMIT_YAW;
  }

  LOG_I("PID Sum Limits: Roll/Pitch=%.1f, Yaw=%.1f", pid_profile_.pidSumLimit, pid_profile_.pidSumLimitYaw);
}

void PidBf::initFilters() {
  // TODO: 初始化 PID 滤波器（D-term notch, LPF 等）
  // 暂时简化实现，后续可以添加完整的滤波器初始化
  LOG_I("PID filters initialized (simplified)");
}

void PidBf::threadEntry(void* parameter) {
  if (parameter == RT_NULL) {
    return;
  }

  PidBf* instance = static_cast<PidBf*>(parameter);
  instance->threadLoop();
}

void PidBf::threadLoop() {
  LOG_I("PidBf thread loop started");

  while (true) {
    // 阻塞等待 gyro_filtered 数据发布
    if (mcn_poll_sync(gyro_filtered_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      // 复制数据
      if (mcn_copy(MCN_HUB(gyro_filtered), gyro_filtered_node_, &gyro_filtered_data_) == RT_EOK) {
        gyro_data_ready_ = true;
      }
    }

    // 尝试获取 setpoint 数据（非阻塞）
    if (setpoint_node_ != RT_NULL) {
      if (mcn_poll_sync(setpoint_node_, 0) == RT_TRUE) {
        if (mcn_copy(MCN_HUB(pid_setpoint), setpoint_node_, &setpoint_data_) == RT_EOK) {
          setpoint_data_ready_ = true;
        }
      }
    } else {
      // 如果 setpoint 节点不存在，尝试重新订阅
      setpoint_node_ = mcn_subscribe(MCN_HUB(pid_setpoint), setpoint_event_, RT_NULL);
      if (setpoint_node_ != RT_NULL) {
        LOG_I("Successfully subscribed to pid_setpoint topic");
      }
      // 使用默认 setpoint（全零）
      std::memset(&setpoint_data_, 0, sizeof(setpoint_data_));
      setpoint_data_ready_ = true;
    }

    // 如果 gyro 数据就绪，执行 PID 控制器
    if (gyro_data_ready_) {
      uint32_t current_time_us = timestamp_micros();
      pidController(current_time_us);
      gyro_data_ready_ = false;
    }
  }
}

float PidBf::getSetpointRate(int axis) {
  if (setpoint_data_ready_ && axis >= 0 && axis < 3) {
    return setpoint_data_.rate[axis];
  }
  return 0.0f;
}

float PidBf::getFeedforward(int axis) {
  if (setpoint_data_ready_ && axis >= 0 && axis < 3) {
    return setpoint_data_.feedforward[axis];
  }
  return 0.0f;
}

float PidBf::getMaxRcRate(int axis) {
  // TODO: 从参数系统读取 max RC rate
  // 暂时返回默认值 720 deg/s
  return 720.0f;
}

void PidBf::pidController(uint32_t current_time_us) {
  static float previousGyroRateDterm[XYZ_AXIS_COUNT] = {0};

  // 获取当前陀螺仪数据（滤波后的）
  float gyroRate[XYZ_AXIS_COUNT];
  std::memcpy(gyroRate, gyro_filtered_data_.gyro_filtered, sizeof(gyroRate));

  // 遍历三个轴（Roll, Pitch, Yaw）
  for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
    // 获取 setpoint
    float currentPidSetpoint = getSetpointRate(axis);

    // 限制 setpoint 变化率（acceleration limiting）
    // TODO: 实现 acceleration limiting

    // 计算误差率
    float errorRate = currentPidSetpoint - gyroRate[axis];

    // 计算 P 项
    pid_data_[axis].P = pid_profile_.pid[axis][0] * errorRate;  // Kp * errorRate

    // 计算 I 项
    float itermLimit = (axis == FD_YAW) ? pid_profile_.pidSumLimitYaw : pid_profile_.pidSumLimit;
    float iTermChange = pid_profile_.pid[axis][1] * pid_runtime_.dT * errorRate;  // Ki * dT * errorRate
    pid_data_[axis].I = constrainf(pid_data_[axis].I + iTermChange, -itermLimit, itermLimit);

    // 计算 D 项
    float delta = -(gyroRate[axis] - previousGyroRateDterm[axis]) * pid_runtime_.pidFrequency;
    pid_data_[axis].D = pid_profile_.pid[axis][2] * delta;  // Kd * delta

    // 更新 previousGyroRateDterm
    previousGyroRateDterm[axis] = gyroRate[axis];

    // 计算 F 项（Feedforward）
    float pidSetpointDelta = 0.0f;
    if (axis < 3) {
      float currentSetpoint = getSetpointRate(axis);
      pidSetpointDelta = currentSetpoint - pid_runtime_.previousPidSetpoint[axis];
      pid_runtime_.previousPidSetpoint[axis] = currentSetpoint;
    }
    float feedforwardGain = FEEDFORWARD_SCALE * getFeedforward(axis);
    pid_data_[axis].F = feedforwardGain * pidSetpointDelta;

    // S 项（暂时为 0，用于固定翼）
    pid_data_[axis].S = 0.0f;

    // 计算 PID 总和
    pid_data_[axis].Sum =
        pid_data_[axis].P + pid_data_[axis].I + pid_data_[axis].D + pid_data_[axis].F + pid_data_[axis].S;

    // 限制 PID 总和
    float pidSumLimit = (axis == FD_YAW) ? pid_profile_.pidSumLimitYaw : pid_profile_.pidSumLimit;
    pid_data_[axis].Sum = constrainf(pid_data_[axis].Sum, -pidSumLimit, pidSumLimit);
  }

  // 发布 PID 输出到 MCN
  pid_output_msg_t output_msg;
  output_msg.timestamp = current_time_us;
  output_msg.seq = gyro_filtered_data_.seq;

  for (int axis = 0; axis < 3; axis++) {
    output_msg.pid_sum[axis] = pid_data_[axis].Sum;
    output_msg.pid_p[axis] = pid_data_[axis].P;
    output_msg.pid_i[axis] = pid_data_[axis].I;
    output_msg.pid_d[axis] = pid_data_[axis].D;
    output_msg.pid_f[axis] = pid_data_[axis].F;
  }

  if (pid_output_hub_ != nullptr) {
    mcn_publish(pid_output_hub_, &output_msg);
  }
}

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_PID_EN
extern "C" {
static int pid_bf_init_wrapper(void) {
  PidBf& instance = PidBf::instance();
  rt_err_t ret = instance.init();
  if (ret == RT_EOK) {
    LOG_I("PidBf auto-init success");
  } else {
    LOG_E("PidBf auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(pid_bf_init_wrapper);
}
#endif
