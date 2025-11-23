#ifndef MOTOR_BF_H__
#define MOTOR_BF_H__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cstring>

extern "C" {
#include "motor_output_msg.h"
#include "pid_output_msg.h"
#include "rc_setpoint_msg.h"
#include "rc_aux_msg.h"
#include "timestamp.h"
#include "param.h"
}

#define MAX_SUPPORTED_MOTORS 8
#define XYZ_AXIS_COUNT 3

// Mixer modes (from mixer.h)
#define MIXER_QUADX 3
#define MIXER_QUADP 2
#define MIXER_TRI 1

// Mixer types
#define MIXER_LEGACY 0
#define MIXER_LINEAR 1
#define MIXER_DYNAMIC 2
#define MIXER_EZLANDING 3

// Motor mixer configuration per motor
struct motorMixer_t {
  float throttle;
  float roll;
  float pitch;
  float yaw;
};

class MotorBf {
 public:
  // Singleton pattern
  static MotorBf& instance();

  MotorBf();
  ~MotorBf();

  rt_err_t init();

 private:
  MotorBf(const MotorBf&) = delete;
  MotorBf& operator=(const MotorBf&) = delete;

  // Initialize MCN
  rt_err_t initMcn();

  // MCN subscription management
  rt_err_t subscribeMcnTopics(McnNode_t* pid_output_node, McnNode_t* rc_setpoint_node, McnNode_t* rc_aux_node);
  void unsubscribeMcnTopics();

  // MCN data update helpers
  bool updateAuxData(McnNode_t rc_aux_node, rc_aux_msg_t* aux_data, bool* aux_data_valid);
  bool updateRcSetpointData(McnNode_t rc_setpoint_node, rc_setpoint_msg_t* rc_setpoint);

  // Initialize mixer configuration
  void initMixerConfig();

  // Motor mixing functions
  // Note: rc_setpoint parameter is kept for future use, but throttle now comes from pid_output->smoothed_throttle
  void mixTable(const pid_output_msg_t* pid_output, const rc_setpoint_msg_t* rc_setpoint, float* motor_output);
  void applyMixerAdjustment(float* motorMix, float motorMixMin, float motorMixMax);
  void applyMixToMotors(const float* motorMix, const motorMixer_t* activeMixer, float throttle, float* motor_output);
  
  // Write motors to device
  void writeMotors(const float* motor_output, rt_device_t motor_device);

  // Helper functions
  float constrainf(float x, float min, float max) const;
  float scaleRangef(float x, float in_min, float in_max, float out_min, float out_max) const;

  // Motor thread entry point
  static void motorThreadEntry(void* parameter);

  // Motor thread initialization helpers
  static rt_device_t initMotorDevice();
  static void cleanupMotorDevice(rt_device_t motor_device);

  // Mixer configuration
  uint8_t mixer_mode_;          // Mixer mode (MIXER_QUADX, etc.)
  uint8_t mixer_type_;          // Mixer type (MIXER_LEGACY, etc.)
  uint8_t motor_count_;         // Number of motors
  motorMixer_t current_mixer_[MAX_SUPPORTED_MOTORS];  // Current mixer configuration

  // Motor output values
  float motor_[MAX_SUPPORTED_MOTORS];

  // MCN nodes and events
  rt_sem_t pid_output_event_;  // Event semaphore for pid (required for mcn_poll_sync)
  McnNode_t motor_output_node_;
  McnNode_t pid_output_node_;
  McnNode_t rc_setpoint_node_;
  McnNode_t rc_aux_node_;  // For arm status and flight mode

  // Thread
  rt_thread_t motor_thread_;
  struct rt_thread motor_thread_obj_;
  rt_uint8_t* motor_thread_stack_;
  bool thread_inited_;

  // Sequence counter
  uint32_t seq_;
};

#endif /* MOTOR_BF_H__ */

