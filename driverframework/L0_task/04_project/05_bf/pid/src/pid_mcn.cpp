#include "pid_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "pid_mcn"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "pid_mcn.h"
#include "rc_mcn.h"  // For rc_command_msg_t
#ifdef PROJECT_BF_ATTITUDE_EN
#include "../attitude/inc/attitude_mcn.h"  // For attitude_msg_t
#endif
}

#include <cstring>

MCN_DEFINE(pid, sizeof(pid_output_msg_t));

extern "C" {
static int pid_echo(void* parameter) {
  pid_output_msg_t output_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &output_data) != RT_EOK) {
    return -1;
  }

  // Call PidBf member function to print debug information
  // This keeps the logic in the object while maintaining C linkage for MCN
  PidBf& pid = PidBf::instance();
  pid.echoPidOutput(&output_data);

  return 0;
}
}

// MCN 初始化函数
rt_err_t PidBf::initMcnSubscriptions() {
  // 初始化陀螺仪数据订阅
  if (gyro_filtered_event_ == RT_NULL) {
    gyro_filtered_event_ = rt_sem_create("pid_gyro_evt", 0, RT_IPC_FLAG_FIFO);
    if (gyro_filtered_event_ == RT_NULL) {
      LOG_E("create gyro_filtered event semaphore failed");
      return -RT_ERROR;
    }
  }

  gyro_filtered_node_ = mcn_subscribe(MCN_HUB(gyro), gyro_filtered_event_, RT_NULL);
  if (gyro_filtered_node_ == RT_NULL) {
    LOG_E("subscribe gyro topic failed");
    if (gyro_filtered_event_ != RT_NULL) {
      rt_sem_delete(gyro_filtered_event_);
      gyro_filtered_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to gyro MCN topic");

  // 获取PID输出发布器
  pid_output_hub_ = MCN_HUB(pid);
  if (pid_output_hub_ == nullptr) {
    LOG_E("get pid hub failed");
    cleanupMcnSubscriptions();
    return -RT_ERROR;
  }

  rt_err_t advertise_ret = mcn_advertise(pid_output_hub_, pid_echo);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    LOG_E("pid advertise failed: %d", advertise_ret);
    cleanupMcnSubscriptions();
    return advertise_ret;
  }
  LOG_I("pid MCN topic advertised");

  // 订阅RC辅助通道MCN主题（非阻塞）
  rc_aux_node_ = mcn_subscribe(MCN_HUB(aux), RT_NULL, RT_NULL);
  if (rc_aux_node_ == RT_NULL) {
    LOG_W("subscribe aux topic failed, continuing without aux channel support");
    // 不是关键功能，继续运行
  } else {
    LOG_I("Subscribed to aux MCN topic");
  }

  // 订阅RC命令MCN主题（PID线程独立订阅）
  // 这确保PID线程有自己的节点，不会与RC模块的订阅冲突
  rc_command_node_ = mcn_subscribe(MCN_HUB(rc), RT_NULL, RT_NULL);
  if (rc_command_node_ == RT_NULL) {
    LOG_W("subscribe rc_command topic failed, continuing without RC command support");
    // 不是关键功能，但可能影响RC平滑滤波器
  } else {
    LOG_I("Subscribed to rc_command MCN topic (independent subscription)");
  }

#ifdef PROJECT_BF_ATTITUDE_EN
  // 订阅姿态数据MCN主题（非阻塞，用于角度模式）
  attitude_node_ = mcn_subscribe(MCN_HUB(att), RT_NULL, RT_NULL);
  if (attitude_node_ == RT_NULL) {
    LOG_W("subscribe attitude topic failed, angle mode will be disabled");
    // 角度模式需要姿态数据，如果没有则禁用角度模式
    attitude_data_valid_ = false;
  } else {
    LOG_I("Subscribed to attitude MCN topic");
    // 订阅成功后，尝试获取一次数据（如果有的话）
    // 即使没有数据，也先设置为 true，后续会通过 updateAttitudeDataFromMcn 更新
    // 这样可以确保在第一次获取到数据后，即使后续某次没有新数据，也能继续使用历史数据
    if (mcn_poll(attitude_node_) == RT_TRUE) {
      if (mcn_copy(MCN_HUB(att), attitude_node_, &attitude_data_) == RT_EOK) {
        attitude_data_valid_ = true;
        LOG_I("Initial attitude data loaded from MCN");
      } else {
        attitude_data_valid_ = false;
      }
    } else {
      // 订阅成功但暂时没有数据，先设置为 false，等待第一次数据更新
      attitude_data_valid_ = false;
    }
  }
#endif

  return RT_EOK;
}

// MCN 清理函数
void PidBf::cleanupMcnSubscriptions() {
  if (gyro_filtered_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(gyro), gyro_filtered_node_);
    gyro_filtered_node_ = RT_NULL;
  }
  
  if (gyro_filtered_event_ != RT_NULL) {
    rt_sem_delete(gyro_filtered_event_);
    gyro_filtered_event_ = RT_NULL;
  }

  if (rc_aux_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(aux), rc_aux_node_);
    rc_aux_node_ = RT_NULL;
  }

  if (rc_command_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(rc), rc_command_node_);
    rc_command_node_ = RT_NULL;
  }

#ifdef PROJECT_BF_ATTITUDE_EN
  if (attitude_node_ != RT_NULL) {
    mcn_unsubscribe(MCN_HUB(att), attitude_node_);
    attitude_node_ = RT_NULL;
  }
  attitude_data_valid_ = false;
#endif

  pid_output_hub_ = nullptr;
}

// 从MCN更新RC命令数据
const rc_command_msg_t* PidBf::updateRcCommandFromMcn() {
  // 轮询新的RC命令数据（非阻塞）
  // 这遵循Betaflight模式：PID频率(3.2kHz) >> RC频率(65Hz)
  // 所以我们需要始终返回有效数据：如果有新数据则返回新数据，否则返回缓存的历史数据
  if (rc_command_node_ != RT_NULL) {
    if (mcn_poll(rc_command_node_) == RT_TRUE) {
      // 有新的RC命令数据可用，复制它
      if (mcn_copy(MCN_HUB(rc), rc_command_node_, &rc_command_data_) == RT_EOK) {
        // RC命令数据更新成功，缓存它以备将来使用
        std::memcpy(&rc_command_data_cached_, &rc_command_data_, sizeof(rc_command_data_));
        rc_command_data_valid_ = true;
        return &rc_command_data_;
      }
    }
  }

  // 没有新数据可用，如果有效则返回缓存的历史数据
  // 这确保平滑滤波器可以始终在PID频率下处理RC数据
  if (rc_command_data_valid_) {
    return &rc_command_data_cached_;
  }

  // 还没有缓存数据（在发布任何RC数据之前的第一次调用）
  return nullptr;
}

// 从MCN更新RC辅助通道数据
void PidBf::updateRcAuxFromMcn() {
  // 轮询新的辅助通道数据（非阻塞）
  if (rc_aux_node_ != RT_NULL) {
    if (mcn_poll(rc_aux_node_) == RT_TRUE) {
      // 有新的辅助通道数据可用，复制它
      if (mcn_copy(MCN_HUB(aux), rc_aux_node_, &aux_channels_data_) == RT_EOK) {
        // 辅助通道数据更新成功
      }
    }
  }
}

// 从MCN获取陀螺仪数据（阻塞）
bool PidBf::updateGyroDataFromMcn() {
  // 轮询新的陀螺仪数据（阻塞 - 将等待直到数据可用）
  // 由于我们使用RT_WAITING_FOREVER，一旦获得数据，就保证是有效的
  if (mcn_poll_sync(gyro_filtered_node_, RT_WAITING_FOREVER) == RT_TRUE) {
    if (mcn_copy(MCN_HUB(gyro), gyro_filtered_node_, &gyro_filtered_data_) == RT_EOK) {
      return true;
    }
  }
  return false;
}

#ifdef PROJECT_BF_ATTITUDE_EN
// 从MCN更新姿态数据（非阻塞）
// 参考 Betaflight：一旦获取到有效数据，就持续使用历史数据，即使某次没有新数据更新
bool PidBf::updateAttitudeDataFromMcn() {
  // 如果订阅失败，直接返回 false
  if (attitude_node_ == RT_NULL) {
    attitude_data_valid_ = false;
    return false;
  }
  
  // 轮询新的姿态数据（非阻塞）
  // 姿态数据更新频率较低（通常 < PID 频率），所以使用非阻塞方式
  if (mcn_poll(attitude_node_) == RT_TRUE) {
    // 有新的姿态数据可用，复制它
    if (mcn_copy(MCN_HUB(att), attitude_node_, &attitude_data_) == RT_EOK) {
      // 成功获取新数据，标记为有效
      attitude_data_valid_ = true;
      return true;
    } else {
      // 复制失败，但保持之前的状态（如果有历史数据，继续使用）
      return attitude_data_valid_;
    }
  }
  
  // 没有新数据，但如果有有效的历史数据，继续使用历史数据（Betaflight 的做法）
  // 这样可以避免因为姿态更新频率低于 PID 频率而导致角度模式频繁退出
  return attitude_data_valid_;
}
#endif

// 发布PID输出数据到MCN
void PidBf::publishPidOutput(const pid_output_msg_t& output_msg) {
  if (pid_output_hub_ != nullptr) {
    mcn_publish(pid_output_hub_, &output_msg);
  }
}

// Echo PID output data (called from MCN echo callback)
void PidBf::echoPidOutput(const pid_output_msg_t* output_data) {
  if (output_data == nullptr) {
    return;
  }

  // Print PID output information
  LOG_I("PID Output: seq=%u, ts=%u us, throttle=%.0f", output_data->seq, output_data->timestamp, output_data->smoothed_throttle);
  LOG_I("  Roll:  P=%.2f, I=%.2f, D=%.2f, F=%.2f, Sum=%.2f",
        output_data->pid_p[0], output_data->pid_i[0], output_data->pid_d[0],
        output_data->pid_f[0], output_data->pid_sum[0]);
  LOG_I("  Pitch: P=%.2f, I=%.2f, D=%.2f, F=%.2f, Sum=%.2f",
        output_data->pid_p[1], output_data->pid_i[1], output_data->pid_d[1],
        output_data->pid_f[1], output_data->pid_sum[1]);
  LOG_I("  Yaw:   P=%.2f, I=%.2f, D=%.2f, F=%.2f, Sum=%.2f",
        output_data->pid_p[2], output_data->pid_i[2], output_data->pid_d[2],
        output_data->pid_f[2], output_data->pid_sum[2]);
}