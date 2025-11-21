#include "rc_bf.h"
#include "pid_bf.hpp"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "task_pid"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

// Thread configuration
#ifndef CONFIG_PROJECT_BF_PID_MAIN_THREAD_STACK_SIZE
#define CONFIG_PROJECT_BF_PID_MAIN_THREAD_STACK_SIZE 4096
#endif

#ifndef CONFIG_PROJECT_BF_PID_MAIN_THREAD_PRIORITY
#define CONFIG_PROJECT_BF_PID_MAIN_THREAD_PRIORITY 8
#endif

#ifndef CONFIG_PROJECT_BF_PID_MAIN_THREAD_TIMESLICE
#define CONFIG_PROJECT_BF_PID_MAIN_THREAD_TIMESLICE 5
#endif

// Target loop time (8kHz default)
#ifndef CONFIG_PROJECT_BF_PID_MAIN_LOOPTIME_US
#define CONFIG_PROJECT_BF_PID_MAIN_LOOPTIME_US 125
#endif

namespace {

// PID main thread instance
struct PidMainThread {
  rt_thread_t thread_;
  struct rt_thread thread_obj_;
  rt_uint8_t thread_stack_[CONFIG_PROJECT_BF_PID_MAIN_THREAD_STACK_SIZE];
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
  // PID Task (1-4kHz): Process RC command and smoothing filter
  RcBf& rc = RcBf::instance();
  
  // Step 1: Process RC command - rcCommand[] → applyRates() → rawSetpoint[]
  rc.processRcCommand(current_time_us);
  
  // Step 2: Process RC smoothing filter
  // Input: rcCommand[THROTTLE] or rawSetpoint[ROLL/PITCH/YAW]
  // Output: rcCommand[THROTTLE] (in-place update), setpointRate[ROLL/PITCH/YAW] (new array)
  rc.processRcSmoothingFilter();
}

static void subTaskPidController(uint32_t current_time_us) {
  // PidBf runs in its own thread and processes gyro data
  // This sub-task can be used for additional PID-related processing if needed
  // For example: PID output monitoring, logging, etc.
  (void)current_time_us;
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
  if (parameter == RT_NULL) {
    return;
  }
  pidMainLoop(parameter);
}

}  // namespace

// Public API
rt_err_t pidMainInit(void) {
  if (pid_main_thread_.thread_inited_) {
    LOG_W("PidMain already initialized");
    return RT_EOK;
  }

  // Note: RcBf and PidBf are initialized separately by their init wrappers
  // This function only starts the main PID thread

  // Initialize main thread
  rt_err_t ret = rt_thread_init(&pid_main_thread_.thread_obj_, "pid_main", pidMainThreadEntry, RT_NULL,
                       pid_main_thread_.thread_stack_, CONFIG_PROJECT_BF_PID_MAIN_THREAD_STACK_SIZE,
                       CONFIG_PROJECT_BF_PID_MAIN_THREAD_PRIORITY,
                       CONFIG_PROJECT_BF_PID_MAIN_THREAD_TIMESLICE);

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
// Note: pid_bf_init_wrapper is already exported by pid_bf.cpp
// This main init should run after pid_bf is initialized
#ifdef PROJECT_BF_PID_EN
extern "C" {
static int pid_main_init_wrapper(void) {
  // Small delay to ensure pid_bf is initialized first
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

