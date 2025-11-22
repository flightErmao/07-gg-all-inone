#include "rc_bf.h"
#include "pid_bf.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include "timestamp.h"
#include "uMCN.h"
#define LOG_TAG "task_pid"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>
#include "rc_setpoint_msg.h"


// Target loop time (8kHz default)
#ifndef CONFIG_PROJECT_BF_PID_MAIN_LOOPTIME_US
#define CONFIG_PROJECT_BF_PID_MAIN_LOOPTIME_US 125
#endif

namespace {

// PID main thread instance
struct PidMainThread {
  rt_thread_t thread_;
  struct rt_thread thread_obj_;
  rt_uint8_t thread_stack_[PROJECT_BF_PID_THREAD_STACK_SIZE];
  bool thread_inited_;
  uint32_t target_looptime_us_;

  PidMainThread()
      : thread_(RT_NULL),
        thread_inited_(false),
        target_looptime_us_(CONFIG_PROJECT_BF_PID_MAIN_LOOPTIME_US) {
    std::memset(&thread_obj_, 0, sizeof(thread_obj_));
    std::memset(thread_stack_, 0, sizeof(thread_stack_));
  }
};

static PidMainThread pid_main_thread_;

// Sub-tasks
static void subTaskRcCommand(uint32_t current_time_us) {
  // PID Task (8kHz): Process RC smoothing filter
  // Note: processRcCommand is now in RC thread (100-200Hz), data passed via MCN to avoid data tearing
  RcBf& rc = RcBf::instance();
  
  // Subscribe to rc MCN topic (non-blocking poll)
  rc_setpoint_msg_t setpoint_msg;
  const rc_setpoint_msg_t* msg_to_use = nullptr;
  
  // Poll for new rc_setpoint data (non-blocking, use latest data if available)
  McnNode_t rc_setpoint_node = rc.getRcSetpointNode();
  if (rc_setpoint_node != RT_NULL) {
    if (mcn_poll_sync(rc_setpoint_node, 0) == RT_TRUE) {
      if (mcn_copy(MCN_HUB(rc), rc_setpoint_node, &setpoint_msg) == RT_EOK) {
        // New data available, use it
        msg_to_use = &setpoint_msg;
      }
    }
  }
  
  // Process RC smoothing filter with MCN data (always run at PID frequency, even if no new RC data)
  // If no new data, use nullptr to use cached data (filter will maintain state)
  // Note: This is called at PID frequency (8kHz) even if RC data is at 100Hz
  rc.processRcSmoothingFilter(msg_to_use);
}

static void subTaskPidController(uint32_t current_time_us) {
  // Process PID controller (moved from PidBf::threadLoop)
  PidBf& pid = PidBf::instance();
  pid.processPidController(current_time_us);
}

static void subTaskMotorUpdate(uint32_t current_time_us) {
  // TODO: Implement motor mixer
  // This will take PID output and convert to motor commands
  // For now, it's empty
  (void)current_time_us;
}

static void subTaskPidSubprocesses(uint32_t current_time_us) {
  // TODO: Additional PID subprocesses
  // For example: attitude estimation updates, sensor fusion, etc.
  (void)current_time_us;
}

// Main PID loop
static void pidMainLoop(void* parameter) {
  (void)parameter;

  LOG_I("PidMain loop started");

  uint32_t last_time_us = timestamp_micros();
  uint32_t loop_count = 0;

  while (true) {
    uint32_t current_time_us = timestamp_micros();
    uint32_t loop_time_us = current_time_us - last_time_us;

    // Run sub-tasks
    subTaskRcCommand(current_time_us);
    subTaskPidController(current_time_us);
    subTaskMotorUpdate(current_time_us);
    subTaskPidSubprocesses(current_time_us);

    // Rate control - sleep to maintain target loop time
    if (loop_time_us < pid_main_thread_.target_looptime_us_) {
      uint32_t sleep_us = pid_main_thread_.target_looptime_us_ - loop_time_us;
      rt_thread_mdelay(sleep_us / 1000);  // Convert to milliseconds
    }

    last_time_us = current_time_us;
    loop_count++;

    // Log every second (at 8kHz, that's 8000 loops)
    if (loop_count >= 8000) {
      LOG_D("PidMain loop running at ~%d Hz", 8000000 / pid_main_thread_.target_looptime_us_);
      loop_count = 0;
    }
  }
}

// Thread entry function
static void pidMainThreadEntry(void* parameter) {
  // Note: parameter can be RT_NULL, pidMainLoop handles it
  pidMainLoop(parameter);
}

}  // namespace

// Public API
rt_err_t pidMainInit(void) {
  if (pid_main_thread_.thread_inited_) {
    LOG_W("PidMain already initialized");
    return RT_EOK;
  }

  // Initialize PidBf instance (without thread)
  PidBf& pid = PidBf::instance();
  rt_err_t ret = pid.init();
  if (ret != RT_EOK) {
    LOG_E("PidBf init failed: %d", ret);
    return ret;
  }
  LOG_I("PidBf initialized successfully");

  // Initialize main thread
  ret = rt_thread_init(&pid_main_thread_.thread_obj_, "pid_main", pidMainThreadEntry, RT_NULL,
                       pid_main_thread_.thread_stack_, PROJECT_BF_PID_THREAD_STACK_SIZE,
                       PROJECT_BF_PID_THREAD_PRIORITY,
                       PROJECT_BF_PID_THREAD_TIMESLICE);

  if (ret != RT_EOK) {
    LOG_E("PidMain thread init failed: %d", ret);
    return ret;
  }

  pid_main_thread_.thread_ = &pid_main_thread_.thread_obj_;
  pid_main_thread_.thread_inited_ = true;

  ret = rt_thread_startup(pid_main_thread_.thread_);
  if (ret != RT_EOK) {
    LOG_E("PidMain thread startup failed: %d", ret);
    pid_main_thread_.thread_inited_ = false;
    return ret;
  }

  LOG_I("PidMain thread started, target looptime: %u us (~%d Hz)", pid_main_thread_.target_looptime_us_,
        8000000 / pid_main_thread_.target_looptime_us_);
  return RT_EOK;
}

// RT-Thread auto initialization wrapper
// Note: pid_bf thread is disabled, using this main PID thread instead
#ifdef PROJECT_BF_PID_EN
extern "C" {
static int pid_main_init_wrapper(void) {
  // Small delay to ensure RcBf is initialized first
  rt_thread_mdelay(10);
  
  rt_err_t ret = pidMainInit();
  if (ret == RT_EOK) {
    LOG_I("PidMain auto-init success");
  } else {
    LOG_E("PidMain auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(pid_main_init_wrapper);
}
#endif

