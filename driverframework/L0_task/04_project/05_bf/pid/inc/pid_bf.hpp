#ifndef PID_BF_HPP__
#define PID_BF_HPP__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include "gyro_filtered_msg.h"
#include "pid_setpoint_msg.h"
#include "pid_output_msg.h"
#include "aMcnStabilize.h"
#include "param.h"
#include "timestamp.h"
}

// PID controller class (C++ wrapper for Betaflight PID)
class PidBf {
 public:
  // Singleton pattern
  static PidBf& instance();

  PidBf();
  ~PidBf();

  rt_err_t init();

  // Thread entry function (static)
  static void threadEntry(void* parameter);

 private:
  PidBf(const PidBf&) = delete;
  PidBf& operator=(const PidBf&) = delete;

  // Thread main loop
  void threadLoop();

  // Initialize PID filters and configuration
  void initFilters();
  void initConfig();

  // PID controller main function
  void pidController(uint32_t current_time_us);

  // Helper functions
  float getSetpointRate(int axis);
  float getFeedforward(int axis);
  float getMaxRcRate(int axis);

  // PID runtime data (mapped from pidRuntime_t)
  struct {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[3];
    // Add more fields as needed from pidRuntime_t
  } pid_runtime_;

  // PID axis data (mapped from pidAxisData_t)
  struct {
    float P;
    float I;
    float D;
    float F;
    float S;
    float Sum;
  } pid_data_[3];

  // PID profile (mapped from pidProfile_t)
  struct {
    float pid[3][3];  // [axis][P, I, D]
    float pidSumLimit;
    float pidSumLimitYaw;
    // Add more fields as needed from pidProfile_t
  } pid_profile_;

  // Thread related
  rt_thread_t thread_;
  struct rt_thread thread_obj_;
  rt_uint8_t thread_stack_[4096];
  bool thread_inited_;

  // MCN subscription and publication
  rt_sem_t gyro_filtered_event_;
  McnNode_t gyro_filtered_node_;
  rt_sem_t setpoint_event_;
  McnNode_t setpoint_node_;
  McnHub_t pid_output_hub_;

  // Current data
  gyro_filtered_msg_t gyro_filtered_data_;
  pid_setpoint_msg_t setpoint_data_;
  bool gyro_data_ready_;
  bool setpoint_data_ready_;

  // Target looptime (from pid_process_denom)
  uint32_t target_looptime_us_;
};

#endif /* PID_BF_HPP__ */

