#ifndef MOTOR_BF_H__
#define MOTOR_BF_H__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cstring>

extern "C" {
#include "motor_mcn.h"
#include "pid_mcn.h"
#include "rc_mcn.h"  // For rc_aux_msg_t, rc_armed_status_t
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

  // MCN subscription management (each subscription has its own function)
  rt_err_t subscribePidOutput();
  rt_err_t subscribeRcAux();
  void unsubscribeMcnTopics();

  // MCN data update helpers
  bool updateAuxData(rc_aux_msg_t* aux_data, bool* aux_data_valid);

  // Initialize mixer configuration
  void initMixerConfig();

  // Motor mixing functions
  // Throttle now comes from pid_output->smoothed_throttle
  void mixTable(const pid_output_msg_t* pid_output, float* motor_output);
  
  // Normalize motor mix values if range > 1.0 (LEGACY mode - same as Betaflight)
  // Updates motorMix array in-place and returns normalized min/max values
  void normalizeMotorMix(float* motorMix, float* motorMixMin, float* motorMixMax);
  
  // Constrain throttle to prevent clipping when combined with motor mix
  // Ensures motorMix[i] + throttle stays in [0.0, 1.0] for all motors
  void constrainThrottleForMix(float* throttle, float motorMixMin, float motorMixMax);
  
  // Apply mix to motors (with throttle) - store result in motor_output array
  void applyMixToMotors(const float* motorMix, const motorMixer_t* activeMixer, float throttle, float* motor_output);
  
  // Write motors to device
  void writeMotors(const float* motor_output, rt_device_t motor_device);

  // Helper functions
  float constrainf(float x, float min, float max) const;
  float scaleRangef(float x, float in_min, float in_max, float out_min, float out_max) const;

  // Motor thread entry point
  static void motorThreadEntry(void* parameter);

  // Motor thread initialization helpers
  rt_err_t initThreadResources();
  rt_err_t startThread();
  void cleanupThreadResources();
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
  McnNode_t pid_output_node_;
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

