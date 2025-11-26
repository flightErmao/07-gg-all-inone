#ifndef RC_CONTROLS_BF_H__
#define RC_CONTROLS_BF_H__

#include <stdint.h>
#include <stdbool.h>

#include "rc_mcn.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// Constants
#define ARM_DELAY_MS 500
#define STICK_DELAY_MS 50
#define STICK_AUTOREPEAT_MS 250

// Stick position masks (from Betaflight)
#define ROL_LO (1 << (2 * 0))  // Roll LOW
#define ROL_CE (3 << (2 * 0))  // Roll CENTER
#define ROL_HI (2 << (2 * 0))  // Roll HIGH
#define PIT_LO (1 << (2 * 1))  // Pitch LOW
#define PIT_CE (3 << (2 * 1))  // Pitch CENTER
#define PIT_HI (2 << (2 * 1))  // Pitch HIGH
#define YAW_LO (1 << (2 * 2))  // Yaw LOW
#define YAW_CE (3 << (2 * 2))  // Yaw CENTER
#define YAW_HI (2 << (2 * 2))  // Yaw HIGH
#define THR_LO (1 << (2 * 3))  // Throttle LOW
#define THR_CE (3 << (2 * 3))  // Throttle CENTER
#define THR_HI (2 << (2 * 3))  // Throttle HIGH
#define THR_MASK (3 << (2 * 3))

class RcControls {
 public:
  // Singleton instance
  static RcControls& instance();

  // Initialize RC controls
  // Called from RC thread during initialization
  int init();

  // Process RC stick positions
  // Called from RC thread periodically (100-200Hz)
  // This handles stick commands (arm/disarm, calibration, etc.)
  void processRcStickPositions(const float* rc_data, uint32_t current_time_us);

  // Get current arming state (true = armed, false = disarmed)
  bool isArmed() const { return armed_; }

  // Get current flight mode
  uint8_t getFlightMode() const { return flight_mode_; }

  // Check if using stick arming
  bool isUsingStickArming() const { return use_stick_arming_; }

  // Update AUX channel values (called from RC thread when new data arrives)
  void updateAuxChannels(const float* rc_data, uint8_t channel_count);

 private:
  RcControls();
  ~RcControls();
  RcControls(const RcControls&) = delete;
  RcControls& operator=(const RcControls&) = delete;

  // Load parameters from param system
  void loadParameters();

  // Check if AUX channel value is in range (for mode activation)
  bool isAuxChannelActive(uint8_t channel, uint16_t threshold_low,
                          uint16_t threshold_high) const;

  // Check throttle status (low or high)
  enum class ThrottleStatus { LOW, HIGH };
  ThrottleStatus calculateThrottleStatus(const float* rc_data) const;

  // Process stick positions for arming/disarming
  void processStickArming(const float* rc_data, uint32_t current_time_us);

  // Process AUX channel for arming/disarming
  void processAuxArming(const float* rc_data, uint32_t current_time_us);

  // Process AUX channel for flight mode switching
  void processFlightMode(const float* rc_data);

  // Arming state
  bool armed_;
  bool initialized_;

  // Parameters
  uint16_t mincheck_;      // Min position threshold
  uint16_t maxcheck_;      // Max position threshold
  uint16_t midrc_;         // Middle position value
  uint16_t rx_min_usec_;   // RX minimum value
  uint16_t rx_max_usec_;   // RX maximum value

  uint16_t arm_delay_ms_;         // Arm delay (ms)
  uint16_t stick_delay_ms_;       // Stick delay (ms)
  uint16_t stick_autorepeat_ms_;  // Stick autorepeat delay (ms)
  bool use_stick_arming_;         // Use stick arming (true) or AUX switch (false)
  uint8_t auto_disarm_delay_;     // Auto disarm delay (seconds, 0=disabled)
  uint8_t gyro_cal_on_first_arm_; // Calibrate gyro on first arm

  // AUX channel configuration for arming
  uint8_t arm_aux_channel_;        // Arm AUX channel (0=disabled)
  uint16_t arm_aux_threshold_low_;  // Arm AUX threshold low
  uint16_t arm_aux_threshold_high_; // Arm AUX threshold high

  // AUX channel configuration for flight mode
  uint8_t mode_aux_channel_;        // Mode AUX channel (0=disabled)
  uint16_t mode_aux_threshold_low_;  // Mode AUX threshold low
  uint16_t mode_aux_threshold_high_; // Mode AUX threshold high

  // Current AUX channel values (from latest RC data)
  float aux_channels_[14];  // AUX1-AUX14 (channels 5-18)

  // Current flight mode
  uint8_t flight_mode_;

  // Stick position state (for detecting stick commands)
  uint8_t rc_sticks_;      // Current stick position pattern
  int16_t rc_delay_ms_;    // Time sticks have been held in current position
  bool do_not_repeat_;     // Flag to prevent repeating actions
  uint8_t rc_disarm_ticks_; // Guard for disarming through switch
  bool aux_arm_channel_high_; // Flag to track if AUX channel value is > midrc
  bool aux_arm_throttle_ready_; // Flag to track if throttle is low and ready for arming
  bool aux_arm_error_logged_; // Flag to prevent repeated error logging
};

#endif  // __cplusplus

#endif  // RC_CONTROLS_BF_H__

