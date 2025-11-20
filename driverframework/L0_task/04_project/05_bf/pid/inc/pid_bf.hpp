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
#include "param.h"
#include "timestamp.h"
}

#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define XYZ_AXIS_COUNT 3

struct pidf_t {
  float P;
  float I;
  float D;
  float F;
  float S;
};

struct pidProfile_t {
  pidf_t pid[XYZ_AXIS_COUNT];
  float pidSumLimit;
  float pidSumLimitYaw;
  float itermWindup;
  float dtermLpfHz;
  float yawLpfHz;
};

struct pidCoefficient_t {
  float Kp;
  float Ki;
  float Kd;
  float Kf;
};

struct pidAxisData_t {
  float P;
  float I;
  float D;
  float F;
  float S;
  float Sum;
};

struct pidRuntime_t {
  float dT;
  float pidFrequency;
  bool pidStabilisationEnabled;
  float previousPidSetpoint[XYZ_AXIS_COUNT];
  pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
  float itermLimit;
  float itermLimitYaw;
};

struct SimpleLowpass {
  float state;
  float alpha;
  bool enabled;
};

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

  pidRuntime_t pid_runtime_;
  pidProfile_t pid_profile_;
  pidAxisData_t pid_data_[XYZ_AXIS_COUNT];
  SimpleLowpass dterm_lpf_[XYZ_AXIS_COUNT];
  SimpleLowpass yaw_pterm_lpf_;

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

