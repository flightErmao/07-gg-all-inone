#include "rc_bf.h"
#include "rc_smoothing_filter.h"
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

  PidMainThread()
      : thread_(RT_NULL),
        thread_inited_(false) {
    std::memset(&thread_obj_, 0, sizeof(thread_obj_));
    std::memset(thread_stack_, 0, sizeof(thread_stack_));
  }
};

static PidMainThread pid_main_thread_;

// Sub-tasks
static void subTaskRcCommand(uint32_t current_time_us) {
  // PID Task (8kHz): Process RC smoothing filter
  // Note: processRcCommand is now in RC thread (100-200Hz), data passed via MCN to avoid data tearing
  (void)current_time_us;

  PidBf& pid = PidBf::instance();

  // Get RC smoothing filter instance from PID
  RcSmoothingFilter* smoothing_filter = pid.getRcSmoothingFilter();
  if (smoothing_filter == nullptr) {
    return;  // RC smoothing filter not initialized yet
  }

  // Step 1: Get RC command data from MCN (for smoothing filter)
  // This polls MCN topic using mcn_poll (non-blocking) and always returns valid data pointer
  // Returns new data if available, otherwise cached historical data
  // This ensures smoothing filter can process RC data at PID frequency (3.2kHz) even when RC frequency is only 65Hz
  const rc_setpoint_msg_t* rc_command_msg = pid.updateRcCommandFromMcn();

  // Step 2: Process smoothing filter with RC command data
  // Get PID setpoint data reference for direct write (filter will write filtered data here)
  pid_setpoint_msg_t* pid_setpoint_out = &pid.getSetpointDataRef();

  // Process RC smoothing filter: rc_command_msg → filtered → pid_setpoint_out
  // rc_command_msg is always valid (new data or cached historical data)
  // Filter will process RC command data at PID frequency (3.2kHz) even when RC frequency is only 65Hz
  // The filtered setpoint is directly written to PID singleton's setpoint_data_ member
  smoothing_filter->processFilter(rc_command_msg, pid_setpoint_out);

  // Step 3: Get RC aux channels data from MCN
  // This polls MCN topic and updates internal aux_channels_data_ in PID module
  pid.updateRcAuxFromMcn();

}

static void subTaskPidController(uint32_t current_time_us) {
  // Process PID controller (moved from PidBf::threadLoop)
  PidBf& pid = PidBf::instance();
  pid.processPidController(current_time_us);
}

// Main PID loop
static void pidMainLoop(void* parameter) {
  (void)parameter;

  LOG_I("PidMain loop started");

  while (true) {
    uint32_t current_time_us = timestamp_micros();
    subTaskRcCommand(current_time_us);
    subTaskPidController(current_time_us);
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

