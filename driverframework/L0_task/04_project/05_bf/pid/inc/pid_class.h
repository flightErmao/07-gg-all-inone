#ifndef PID_BF_HPP__
#define PID_BF_HPP__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include <rtconfig.h>  // For PROJECT_BF_PID_D_MAX_EN and other config macros
#include "gyro_mcn.h"
#include "pid_mcn.h"
#include "rc_mcn.h"  // For rc_aux_msg_t, rc_command_msg_t, rc_armed_status_t
#include "timestamp.h"
#include "filter.h"  // For biquadFilter_t, pt1Filter_t, pt2Filter_t, pt3Filter_t, filterApplyFnPtr, lowpassFilterType_e
#ifdef PROJECT_BF_ATTITUDE_EN
#include "../attitude/inc/attitude_mcn.h"  // For attitude_msg_t
#endif
}

#include "pid_mlog.h"

// Forward declaration
class RcSmoothingFilter;

#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#define XYZ_AXIS_COUNT 3
#define RP_AXIS_COUNT 2  // Roll and Pitch only (for angle mode)

struct pidf_t {
  float P;
  float I;
  float D;
  float F;
  float S;
};

// Dterm lowpass filter union (same as Betaflight)
// Can be PT1, BIQUAD, PT2, or PT3 filter type
typedef union dtermLowpass_u {
  pt1Filter_t pt1Filter;
  biquadFilter_t biquadFilter;
  pt2Filter_t pt2Filter;
  pt3Filter_t pt3Filter;
} dtermLowpass_t;

// Dynamic LPF filter type (same as Betaflight)
typedef enum {
  DYN_LPF_NONE = 0,
  DYN_LPF_PT1,
  DYN_LPF_BIQUAD,
  DYN_LPF_PT2,
  DYN_LPF_PT3,
} dynLpfFilterType_e;

struct pidProfile_t {
  pidf_t pid[XYZ_AXIS_COUNT];
  float pidSumLimit;
  float pidSumLimitYaw;
  float itermWindup;
  
  // Dterm filter parameters (same as Betaflight)
  uint16_t dterm_notch_hz;              // Dterm notch filter center frequency (Hz)
  uint16_t dterm_notch_cutoff;          // Dterm notch filter cutoff frequency (Hz)
  uint16_t dterm_lpf1_static_hz;        // Dterm lowpass filter 1 static cutoff (Hz)
  uint8_t dterm_lpf1_type;              // Dterm lowpass filter 1 type (FILTER_PT1, FILTER_BIQUAD, FILTER_PT2, FILTER_PT3)
  uint16_t dterm_lpf2_static_hz;        // Dterm lowpass filter 2 static cutoff (Hz)
  uint8_t dterm_lpf2_type;              // Dterm lowpass filter 2 type (FILTER_PT1, FILTER_BIQUAD, FILTER_PT2, FILTER_PT3)
  
  // Dynamic LPF parameters (same as Betaflight)
  uint16_t dterm_lpf1_dyn_min_hz;       // Dterm lowpass filter 1 min hz when in dynamic mode
  uint16_t dterm_lpf1_dyn_max_hz;       // Dterm lowpass filter 1 max hz when in dynamic mode
  uint8_t dterm_lpf1_dyn_expo;          // Set the curve for dynamic dterm lowpass filter
  
#ifdef PROJECT_BF_PID_D_MAX_EN
  // D_MAX parameters (same as Betaflight)
  uint8_t d_max[XYZ_AXIS_COUNT];        // Maximum D value on each axis
  uint8_t d_max_gain;                   // Gain factor for amount of gyro / setpoint activity required to boost D
  uint8_t d_max_advance;                // Percentage multiplier for setpoint input to boost algorithm
#endif
  
  // Yaw P term filter parameters
  uint16_t yaw_lowpass_hz;              // Yaw P term lowpass filter cutoff (Hz) - replaces yawLpfHz
  float yawLpfHz;                       // Legacy: kept for backward compatibility, maps to yaw_lowpass_hz
  
  // Angle mode parameters (same as Betaflight)
  uint8_t angle_limit;                  // Max angle in degrees in Angle mode
  uint8_t angle_earth_ref;              // Control amount of "co-ordination" from yaw into roll while pitched forward in angle mode (0-100)
  float angle_p_gain;                   // Angle mode P gain
  float angle_feedforward;              // Angle mode feedforward gain
  uint8_t angle_feedforward_smoothing_ms; // Smoothing factor for angle feedforward as time constant in milliseconds
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
  
  // Dterm filters (same pattern as Betaflight)
  filterApplyFnPtr dtermNotchApplyFn;          // Dterm notch filter apply function pointer
  biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];   // Dterm notch filter state
  
  filterApplyFnPtr dtermLowpassApplyFn;        // Dterm lowpass filter 1 apply function pointer
  dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT]; // Dterm lowpass filter 1 state (union of PT1/BIQUAD/PT2/PT3)
  
  filterApplyFnPtr dtermLowpass2ApplyFn;       // Dterm lowpass filter 2 apply function pointer
  dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT]; // Dterm lowpass filter 2 state (union of PT1/BIQUAD/PT2/PT3)
  
  filterApplyFnPtr ptermYawLowpassApplyFn;     // Yaw P term lowpass filter apply function pointer
  pt1Filter_t ptermYawLowpass;                 // Yaw P term lowpass filter state
  
  // Dynamic LPF parameters (same as Betaflight)
  dynLpfFilterType_e dynLpfFilter;            // Dynamic LPF filter type
  uint16_t dynLpfMin;                         // Dynamic LPF minimum frequency (Hz)
  uint16_t dynLpfMax;                         // Dynamic LPF maximum frequency (Hz)
  uint8_t dynLpfCurveExpo;                    // Dynamic LPF curve exponent
  
#ifdef PROJECT_BF_PID_D_MAX_EN
  // D_MAX runtime parameters (same as Betaflight)
  pt2Filter_t dMaxRange[XYZ_AXIS_COUNT];       // PT2 filter for D_MAX range detection
  pt2Filter_t dMaxLowpass[XYZ_AXIS_COUNT];    // PT2 filter for D_MAX multiplier smoothing
  float dMaxPercent[XYZ_AXIS_COUNT];          // D_MAX percentage multiplier (dMax / D)
  float dMaxGyroGain;                         // D_MAX gyro gain factor
  float dMaxSetpointGain;                     // D_MAX setpoint gain factor
#endif
  
  float previousGyroRateDterm[XYZ_AXIS_COUNT]; // Previous gyro rate for Dterm calculation
  
#ifdef PROJECT_BF_ATTITUDE_EN
  // Angle mode runtime parameters (same as Betaflight)
  float angleGain;                      // Angle gain (from angle_p_gain parameter)
  float angleFeedforwardGain;           // Angle feedforward gain
  float angleTarget[RP_AXIS_COUNT];     // Angle target [roll, pitch] (degrees)
  float angleYawSetpoint;               // Yaw setpoint for earth reference compensation
  float angleEarthRef;                  // Earth reference gain factor (0.0-1.0)
  pt3Filter_t attitudeFilter[RP_AXIS_COUNT];  // Attitude filter (PT3) for smoothing angle rate output [roll, pitch]
  pt3Filter_t angleFeedforwardPt3[XYZ_AXIS_COUNT]; // Angle feedforward filter (PT3) [roll, pitch, yaw]
  bool axisInAngleMode[3];              // Flag indicating if axis is in angle mode [roll, pitch, yaw]
  
  // Angle loop debug data for logging (per axis: target, current, errorGain, feedforward)
  struct angleLoopDebug_t {
    float target;        // Angle target (degrees)
    float current;       // Current angle (degrees)
    float errorGain;     // errorAngle * angleGain
    float feedforward;   // Angle feedforward
  } angleLoopDebug[RP_AXIS_COUNT];  // [roll, pitch]
#endif
  
  // Rate loop debug data for logging (per axis: currentSetpoint snapshot)
  struct rateLoopDebug_t {
    float currentSetpoint;  // Current setpoint snapshot (saved before errorRate calculation)
  } rateLoopDebug[XYZ_AXIS_COUNT];  // [roll, pitch, yaw]
};

/* PID setpoint message type (internal to PID module) */
/* This message contains filtered setpoint data from RC smoothing filter */
struct pid_setpoint_msg_t {
  float rate[3];            // Filtered setpoint rates [roll, pitch, yaw] (deg/s)
  float feedforward[3];     // Feedforward values [roll, pitch, yaw]
  float smoothed_throttle; // Smoothed throttle value (from RC smoothing filter, in PWM range 1000-2000)
  uint32_t timestamp;      // Timestamp in microseconds
  uint32_t seq;            // Sequence number
};

class PidBf {
 public:
  // Singleton pattern
  static PidBf& instance();

  PidBf();
  ~PidBf();

  rt_err_t init();

  // Set RC smoothing filter instance (called from RC init)
  void setRcSmoothingFilter(RcSmoothingFilter* filter) { rc_smoothing_filter_ = filter; }
  RcSmoothingFilter* getRcSmoothingFilter() { return rc_smoothing_filter_; }

  // Process PID controller (called from main thread loop)
  void processPidController(uint32_t current_time_us);

  // Start main PID thread (replaces pidMainInit)
  rt_err_t startMainThread();

  // Mlog related functions
  rt_err_t initMlog();
  void pushPidDataToMlog(const bf_mlog::pid_mlog_data_t* data);

  // Get current data (for logging/debugging)
  const gyro_filtered_msg_t& getGyroFilteredData() const { return gyro_filtered_data_; }
  const pid_setpoint_msg_t& getSetpointData() const { return setpoint_data_; }
  pid_setpoint_msg_t& getSetpointDataRef() { return setpoint_data_; }
  const rc_aux_msg_t& getAuxChannelsData() const { return aux_channels_data_; }
  const pidAxisData_t* getPidData() const { return pid_data_; }

  // MCN 相关操作方法
  rt_err_t initMcnSubscriptions();
  void cleanupMcnSubscriptions();
  
  // Update RC command data from MCN (called from subTaskRcCommand)
  // This method polls MCN topic and updates internal data
  // Always returns valid data pointer: new data if available, otherwise cached historical data
  // This ensures PID can always use RC data for smoothing filter even when RC frequency (65Hz) < PID frequency (3.2kHz)
  const rc_command_msg_t* updateRcCommandFromMcn();

  // Update RC aux channels data from MCN (called from subTaskRcCommand)
  // This method polls MCN topic and updates internal aux_channels_data_
  void updateRcAuxFromMcn();
  
  // Update gyro data from MCN (blocking)
  bool updateGyroDataFromMcn();
  
#ifdef PROJECT_BF_ATTITUDE_EN
  // Update attitude data from MCN (non-blocking, polls for new data)
  bool updateAttitudeDataFromMcn();
  
  // Get current attitude data (for angle mode)
  const attitude_msg_t& getAttitudeData() const { return attitude_data_; }
  
  // Get angle loop debug data for logging
  // axis: 0 = roll, 1 = pitch
  // Returns pointer to debug data structure, or nullptr if invalid axis
  const pidRuntime_t::angleLoopDebug_t* getAngleLoopDebug(int axis) const {
    if (axis >= 0 && axis < RP_AXIS_COUNT) {
      return &pid_runtime_.angleLoopDebug[axis];
    }
    return nullptr;
  }
#endif
  
  // Get rate loop debug data for logging
  // axis: 0 = roll, 1 = pitch, 2 = yaw
  // Returns pointer to debug data structure, or nullptr if invalid axis
  const pidRuntime_t::rateLoopDebug_t* getRateLoopDebug(int axis) const;
  
  // Publish PID output to MCN
  void publishPidOutput(const pid_output_msg_t& output_msg);

  // Echo PID output data (for MCN echo callback)
  void echoPidOutput(const pid_output_msg_t* output_data);

#ifdef USE_DYN_LPF
  // Dynamic LPF update function for Dterm
  void dynLpfDTermUpdate(float throttle);
#endif

 private:
  PidBf(const PidBf&) = delete;
  PidBf& operator=(const PidBf&) = delete;

  // Initialize PID filters
  void initFilters();
  
  // Initialize defaults (profile values) - moved to pid_init.cpp
  void initDefaults();
  
  // Initialize runtime state - moved to pid_init.cpp
  void initRuntime();
  
  // Initialize angle mode filters - moved to pid_init.cpp
  void initAngleModeFilters();
  
  // Initialize Dterm filters (called from initFilters, but can be called separately after parameter changes)
  // This function is in pid_init.cpp and initializes filter function pointers based on parameters
  void initDtermFilters();
  
  // Load PID parameters from param system
  void loadPidParameters();
  
  // Calculate PID coefficients from loaded parameters
  void calculatePidCoefficients();
  
  // Load PID process denominator and calculate target looptime
  void loadPidProcessDenom();
  
  // Initialize PID configuration (calls loadPidParameters and calculatePidCoefficients)
  void initConfig();

  // PID controller main function
  void pidController(uint32_t current_time_us);

  // Main thread functions
  void subTaskRcCommand(uint32_t current_time_us);
  void pidMainLoop();
  static void workerEntry(void* parameter);

  // Helper functions
  float getSetpointRate(int axis);
  float getFeedforward(int axis);
  float getMaxRcRate(int axis);
  
#ifdef PROJECT_BF_ATTITUDE_EN
  // Angle mode helper functions
  float pidLevel(int axis, float currentPidSetpoint, float horizonLevelStrength);
  float calcHorizonLevelStrength() const;
  float getCurrentAngle(int axis) const;  // Get current angle from attitude data (degrees)
#endif

  pidRuntime_t pid_runtime_;
  pidProfile_t pid_profile_;
  pidAxisData_t pid_data_[XYZ_AXIS_COUNT];

  // MCN subscription and publication
  rt_sem_t gyro_filtered_event_;
  McnNode_t gyro_filtered_node_;
  rt_sem_t setpoint_event_;
  McnNode_t setpoint_node_;
  McnHub_t pid_output_hub_;

  // MCN subscription for RC auxiliary channels
  McnNode_t rc_aux_node_;

  // MCN subscription for RC command (independent subscription for PID thread)
  McnNode_t rc_command_node_;

  // Current auxiliary channels data
  rc_aux_msg_t aux_channels_data_;

#ifdef PROJECT_BF_ATTITUDE_EN
  // MCN subscription for attitude data
  McnNode_t attitude_node_;
  
  // Current attitude data
  attitude_msg_t attitude_data_;
  bool attitude_data_valid_;  // Flag to indicate if attitude data is valid
#endif

  // Current RC command data (from MCN)
  rc_command_msg_t rc_command_data_;

  // Cached RC command data (for historical data when no new data available)
  // This ensures PID can always use RC data for smoothing filter even when RC frequency < PID frequency
  rc_command_msg_t rc_command_data_cached_;
  bool rc_command_data_valid_;  // Flag to indicate if cached data is valid

  // Current data
  gyro_filtered_msg_t gyro_filtered_data_;
  pid_setpoint_msg_t setpoint_data_;

  // RC smoothing filter instance (set by RC thread during initialization)
  RcSmoothingFilter* rc_smoothing_filter_;

  // Cached max RC rates for each axis (deg/s)
  // Initialized once from RC module to avoid repeated singleton calls
  float max_rc_rate_[XYZ_AXIS_COUNT];

  // Target looptime (from pid_process_denom)
  uint32_t target_looptime_us_;

  // Main thread instance
  rt_thread_t main_thread_;
  struct rt_thread main_thread_obj_;
  rt_uint8_t main_thread_stack_[PROJECT_BF_PID_THREAD_STACK_SIZE];
  bool main_thread_inited_;
};

#endif /* PID_BF_HPP__ */

