extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include <rtdevice.h>
#define LOG_TAG "motor_class"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "pid_mcn.h"
#include "uMCN.h"
#include "rc_mcn.h"  // For PWM_RANGE_MIN and PWM_RANGE constants (if defined there)
}

#include <cmath>
#include <cstring>

// PID mixer scaling (same as Betaflight)
// PID_MIXER_SCALING is 1000.0f in Betaflight (used for division)
#define PID_MIXER_SCALING 1000.0f

// Motor output range (normalized to 0.0-1.0, or 1000-2000 PWM range)
#define MOTOR_OUTPUT_MIN 0.0f
#define MOTOR_OUTPUT_MAX 1.0f

// QuadX mixer configuration (default)
static const motorMixer_t mixerQuadX[] = {
    {1.0f, -1.0f, 1.0f, -1.0f},   // REAR_R
    {1.0f, -1.0f, -1.0f, 1.0f},   // FRONT_R
    {1.0f, 1.0f, 1.0f, 1.0f},     // REAR_L
    {1.0f, 1.0f, -1.0f, -1.0f},   // FRONT_L
};

// Singleton instance
MotorBf& MotorBf::instance() {
  static MotorBf instance_obj;
  return instance_obj;
}

// Constructor
MotorBf::MotorBf()
    : mixer_mode_(MIXER_QUADX),
      mixer_type_(MIXER_LEGACY),
      motor_count_(4),
      pid_output_event_(RT_NULL),
      pid_output_node_(RT_NULL),
      rc_aux_node_(RT_NULL),
      motor_thread_(RT_NULL),
      motor_thread_stack_(nullptr),
      thread_inited_(false),
      seq_(0) {
  std::memset(current_mixer_, 0, sizeof(current_mixer_));
  std::memset(motor_, 0, sizeof(motor_));
  std::memset(&motor_thread_obj_, 0, sizeof(motor_thread_obj_));
}

MotorBf::~MotorBf() {
  if (thread_inited_ && motor_thread_ != RT_NULL) {
    rt_thread_delete(motor_thread_);
  }
  if (motor_thread_stack_ != nullptr) {
    delete[] motor_thread_stack_;
  }
  // Unsubscribe from MCN topics
  unsubscribeMcnTopics();
  // Delete event semaphore
  if (pid_output_event_ != RT_NULL) {
    rt_sem_delete(pid_output_event_);
    pid_output_event_ = RT_NULL;
  }
}

rt_err_t MotorBf::init() {
  rt_err_t ret;

  // Step 1: Initialize mixer configuration
  initMixerConfig();

  // Step 2: Initialize MCN
  ret = initMcn();
  if (ret != RT_EOK) {
    LOG_E("MCN init failed");
    return ret;
  }

  // Step 3: Initialize thread resources
  ret = initThreadResources();
  if (ret != RT_EOK) {
    return ret;
  }

  // Step 4: Start thread
  ret = startThread();
  if (ret != RT_EOK) {
    cleanupThreadResources();
    return ret;
  }

  LOG_I("MotorBf initialized successfully");
  return RT_EOK;
}

// Initialize thread resources (stack allocation and thread object initialization)
rt_err_t MotorBf::initThreadResources() {
  // Allocate thread stack
  size_t stack_size = PROJECT_BF_MOTOR_THREAD_STACK_SIZE;
  motor_thread_stack_ = new rt_uint8_t[stack_size];
  if (motor_thread_stack_ == nullptr) {
    LOG_E("Failed to allocate motor thread stack");
    return -RT_ENOMEM;
  }
  std::memset(motor_thread_stack_, 0, stack_size);

  // Initialize thread object
  rt_err_t ret = rt_thread_init(&motor_thread_obj_, "motor_bf", motorThreadEntry, this, motor_thread_stack_, stack_size,
                                PROJECT_BF_MOTOR_THREAD_PRIORITY, PROJECT_BF_MOTOR_THREAD_TIMESLICE);
  if (ret != RT_EOK) {
    LOG_E("Motor thread init failed: %d", ret);
    delete[] motor_thread_stack_;
    motor_thread_stack_ = nullptr;
    return ret;
  }

  motor_thread_ = &motor_thread_obj_;
  thread_inited_ = true;
  return RT_EOK;
}

// Start the motor thread
rt_err_t MotorBf::startThread() {
  rt_err_t ret = rt_thread_startup(motor_thread_);
  if (ret != RT_EOK) {
    LOG_E("Motor thread startup failed: %d", ret);
    thread_inited_ = false;
    return ret;
  }
  return RT_EOK;
}

// Cleanup thread resources
void MotorBf::cleanupThreadResources() {
  if (thread_inited_ && motor_thread_ != RT_NULL) {
    rt_thread_delete(motor_thread_);
    thread_inited_ = false;
  }
  if (motor_thread_stack_ != nullptr) {
    delete[] motor_thread_stack_;
    motor_thread_stack_ = nullptr;
  }
}

// initMcn() is now in motor_mcn.cpp

void MotorBf::initMixerConfig() {
  // Load mixer parameters
  uint8_t mixer_mode_temp = MIXER_QUADX;
  getParam("mixer_mode", &mixer_mode_temp, sizeof(mixer_mode_temp));
  mixer_mode_ = mixer_mode_temp;

  uint8_t mixer_type_temp = MIXER_LEGACY;
  getParam("mixer_type", &mixer_type_temp, sizeof(mixer_type_temp));
  mixer_type_ = mixer_type_temp;

  // Initialize mixer based on mode
  if (mixer_mode_ == MIXER_QUADX) {
    motor_count_ = 4;
    for (int i = 0; i < motor_count_; i++) {
      current_mixer_[i] = mixerQuadX[i];
    }
  } else {
    // Default to QuadX
    motor_count_ = 4;
    for (int i = 0; i < motor_count_; i++) {
      current_mixer_[i] = mixerQuadX[i];
    }
    LOG_W("Unsupported mixer mode %d, using QuadX", mixer_mode_);
  }

  LOG_I("Mixer initialized: mode=%d, type=%d, motors=%d", mixer_mode_, mixer_type_, motor_count_);
}

void MotorBf::mixTable(const pid_output_msg_t* pid_output, float* motor_output) {
  if (pid_output == nullptr || motor_output == nullptr) {
    return;
  }

  // Use smoothed throttle from PID output (same as Betaflight: rcCommand[THROTTLE] is smoothed)
  // Convert throttle from PWM range (PWM_RANGE_MIN to PWM_RANGE_MAX) to normalized (0.0-1.0)
  // Same as Betaflight: throttle = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / PWM_RANGE
  // Note: throttleAngleCorrection is not applied here (only needed for angle mode compensation)
  float throttle = constrainf((pid_output->smoothed_throttle - PWM_RANGE_MIN) / PWM_RANGE, 0.0f, 1.0f);

  // Scale PID outputs (same as Betaflight)
  // PID sum is already in deg/s range, we need to scale it for mixer
  const float pidSumLimit = 500.0f;  // Default limit, should be configurable
  float scaledAxisPidRoll = constrainf(pid_output->pid_sum[0], -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING;
  float scaledAxisPidPitch = constrainf(pid_output->pid_sum[1], -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING;
  float scaledAxisPidYaw = constrainf(pid_output->pid_sum[2], -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING;

  // Yaw reversal: Betaflight behavior is to negate yaw by default (when yaw_motors_reversed is false)
  // Since we removed yaw_motors_reversed parameter, always negate yaw (normal behavior)
  scaledAxisPidYaw = -scaledAxisPidYaw;

  // Calculate motor mix (roll, pitch, yaw components)
  float motorMix[MAX_SUPPORTED_MOTORS];
  float motorMixMax = 0.0f, motorMixMin = 0.0f;

  for (int i = 0; i < motor_count_; i++) {
    float mix = scaledAxisPidRoll * current_mixer_[i].roll + scaledAxisPidPitch * current_mixer_[i].pitch +
                scaledAxisPidYaw * current_mixer_[i].yaw;

    if (mix > motorMixMax) {
      motorMixMax = mix;
    } else if (mix < motorMixMin) {
      motorMixMin = mix;
    }
    motorMix[i] = mix;
  }

  float motorMixRange = motorMixMax - motorMixMin;

  // Apply mixer adjustment (LEGACY mode)
  applyMixerAdjustment(motorMix, motorMixMin, motorMixMax);

  // Recalculate normalized range after adjustment
  motorMixRange = motorMixMax - motorMixMin;

  // Constrain throttle to prevent clipping
  float normalizedMotorMixMin = motorMixMin * (motorMixRange > 1.0f ? 1.0f / motorMixRange : 1.0f);
  float normalizedMotorMixMax = motorMixMax * (motorMixRange > 1.0f ? 1.0f / motorMixRange : 1.0f);
  throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);

  // Apply mix to motors (with throttle) - store result in motor_output array
  applyMixToMotors(motorMix, current_mixer_, throttle, motor_output);
}

void MotorBf::applyMixerAdjustment(float* motorMix, float motorMixMin, float motorMixMax) {
  // Mixer adjustment (LEGACY mode) - same as Betaflight
  // Normalize motor mix to prevent clipping
  float motorMixRange = motorMixMax - motorMixMin;
  float airmodeTransitionPercent = 1.0f;  // Simplified: no airmode support for now
  
  const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;
  
  for (int i = 0; i < motor_count_; i++) {
    motorMix[i] *= motorMixNormalizationFactor;
  }
}

void MotorBf::applyMixToMotors(const float* motorMix, const motorMixer_t* activeMixer, float throttle, float* motor_output) {
  // Apply mix to each motor (same as Betaflight applyMixToMotors)
  for (int i = 0; i < motor_count_; i++) {
    // motor output = throttle * mixer.throttle + mix (from roll/pitch/yaw)
    float motorOutput = motorMix[i] + throttle * activeMixer[i].throttle;
    
    // Constrain to valid range [0.0, 1.0]
    motorOutput = constrainf(motorOutput, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    
    motor_output[i] = motorOutput;
  }
}

float MotorBf::constrainf(float x, float min, float max) const {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

float MotorBf::scaleRangef(float x, float in_min, float in_max, float out_min, float out_max) const {
  if (in_max == in_min) return out_min;
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

// Write motors to device
void MotorBf::writeMotors(const float* motor_output, rt_device_t motor_device) {
  if (motor_output == nullptr || motor_device == nullptr) {
    return;
  }

  // Convert normalized motor values (0.0-1.0) to device-specific format
  // For DShot, convert to 48-2047 range (reference: ref/motor.c)
  // For PWM, convert to 1000-2000 range
  uint16_t motor_values[MAX_SUPPORTED_MOTORS];
  
  for (int i = 0; i < motor_count_; i++) {
    float normalized = motor_output[i];
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    // Scale from [0.0, 1.0] to [48, 2047] for DShot
    motor_values[i] = (uint16_t)(normalized * (2047.0f - 48.0f) + 48.0f);
  }

  // Write motor values to device
  rt_device_write(motor_device, 0, motor_values, motor_count_ * sizeof(uint16_t));
}

// Motor thread initialization helpers
rt_device_t MotorBf::initMotorDevice() {
#ifndef PROJECT_BF_MOTOR_DEVICE_NAME
  const char* device_name = "dshot";
#else
  const char* device_name = PROJECT_BF_MOTOR_DEVICE_NAME;
#endif
  rt_device_t motor_device = rt_device_find(device_name);
  if (motor_device == nullptr) {
    LOG_E("Motor device %s not found", device_name);
    return nullptr;
  }

  rt_err_t ret = rt_device_open(motor_device, RT_DEVICE_OFLAG_WRONLY);
  if (ret != RT_EOK) {
    LOG_E("Failed to open motor device: %d", ret);
    return nullptr;
  }

  LOG_I("Motor device opened successfully");
  return motor_device;
}

void MotorBf::cleanupMotorDevice(rt_device_t motor_device) {
  if (motor_device != nullptr) {
    rt_device_close(motor_device);
  }
}

// Motor thread entry point - subscribes to PID output, performs mixing, and writes to device
void MotorBf::motorThreadEntry(void* parameter) {
  MotorBf* instance = static_cast<MotorBf*>(parameter);
  if (instance == nullptr) {
    return;
  }

  LOG_I("Motor thread started");

  // Step 1: Initialize motor device
  rt_device_t motor_device = initMotorDevice();
  if (motor_device == nullptr) {
    return;
  }

  // Step 2: Subscribe to MCN topics (each subscription is independent)
  rt_err_t ret = instance->subscribePidOutput();
  if (ret != RT_EOK) {
    LOG_E("Failed to subscribe to PID output MCN topic");
    cleanupMotorDevice(motor_device);
    return;
  }

  // Subscribe to RC aux (non-critical, continue even if fails)
  instance->subscribeRcAux();

  // Step 3: Initialize thread-local data
  float motor_output_array[MAX_SUPPORTED_MOTORS];
  rc_aux_msg_t aux_data = {0};
  bool aux_data_valid = false;

  // Step 4: Main loop - process PID output and control motors
  while (true) {
    // Wait for PID output data (blocking) - this releases CPU when waiting
    // This ensures CPU is released even when disarmed
    if (mcn_poll_sync(instance->pid_output_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      pid_output_msg_t pid_output;
      if (mcn_copy(MCN_HUB(pid), instance->pid_output_node_, &pid_output) == RT_EOK) {
        // Update aux data (non-blocking)
        instance->updateAuxData(&aux_data, &aux_data_valid);

        // Safety: If disarmed or no aux data available (assume disarmed for safety), stop motors
        // Betaflight behavior: motors must be stopped when disarmed
        if (aux_data.armed == RC_ARMED_STATUS_DISARMED) {
          // Disarmed or no data: stop all motors immediately
          for (int i = 0; i < instance->motor_count_; i++) {
            motor_output_array[i] = 0.0f;
          }
          instance->writeMotors(motor_output_array, motor_device);
          continue;  // Skip motor mixing when disarmed
        }

        // Perform motor mixing (only when armed)
        // Throttle comes from pid_output->smoothed_throttle
        instance->mixTable(&pid_output, motor_output_array);

        // Write motors to device
        instance->writeMotors(motor_output_array, motor_device);
      }
    }
  }
}

// RT-Thread auto initialization wrapper
#ifdef PROJECT_BF_MOTOR_EN
extern "C" {
static int motor_bf_init_wrapper(void) {
  MotorBf& instance = MotorBf::instance();
  rt_err_t ret = instance.init();
  if (ret == RT_EOK) {
    LOG_I("MotorBf auto-init success");
  } else {
    LOG_E("MotorBf auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(motor_bf_init_wrapper);
}
#endif

