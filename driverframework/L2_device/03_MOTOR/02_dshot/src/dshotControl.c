#include "dshotConfig.h"
#include <stdbool.h>
#include <string.h>
#include "actuator.h"

/* Direction change timing */
#define DSHOT_DIR_CHANGE_IDLE_TIME_MS 10
#define DSHOT_DIR_CHANGE_CMD_TIME_MS 1
#define DSHOT_SAVE_SETTINGS_DELAY_MS 1

// Add direction change state machine
typedef enum {
  DIR_CHANGE_IDLE_1,  // First idle period (10ms)
  DIR_CHANGE_CMD,     // Send direction change commands
  DIR_CHANGE_IDLE_2,  // Second idle period (10ms)
  DIR_CHANGE_SAVE,    // Send save settings commands
  DIR_CHANGE_STOP,    // Complete the process
} dir_change_state_t;

/* External variables from main driver */
extern dshot_config_t dshot_config_;
extern struct actuator_device act_dev_;

/* Direction change state machine variables */
static dir_change_state_t dir_change_state[DSHOT_MOTOR_NUMS] = {DIR_CHANGE_IDLE_1, DIR_CHANGE_IDLE_1, DIR_CHANGE_IDLE_1,
                                                                DIR_CHANGE_IDLE_1};
static rt_bool_t dir_change_done[DSHOT_MOTOR_NUMS] = {RT_TRUE, RT_TRUE, RT_TRUE, RT_TRUE};
static uint8_t dir_change_counter[DSHOT_MOTOR_NUMS] = {0, 0, 0, 0};
static uint32_t dir_change_time[DSHOT_MOTOR_NUMS] = {0, 0, 0, 0};
static uint8_t target_direction[DSHOT_MOTOR_NUMS] = {0, 0, 0, 0};
/* Bus idle control - if true, motor bus will be set to idle state instead of DShot encoding */
rt_bool_t isBusNeedToBeIdle[DSHOT_MOTOR_NUMS] = {RT_FALSE, RT_FALSE, RT_FALSE, RT_FALSE};

/* Set motor bus idle state - intelligent idle control based on DShot mode */
static rt_err_t setMotorBusIdleState(uint8_t motor_id, rt_bool_t need_idle) {
  if (motor_id >= DSHOT_MOTOR_NUMS) {
    return -RT_EINVAL;
  }
  isBusNeedToBeIdle[motor_id] = need_idle;
  return RT_EOK;
}

/* Motor reverse with explicit direction control */
static rt_err_t motor_reverse_with_direction(uint8_t motor_id, uint8_t direction) {
  if (motor_id >= DSHOT_MOTOR_NUMS) {
    return -RT_EINVAL;  // Invalid motor ID
  }

  if (direction != DSHOT_CMD_SPIN_DIRECTION_1 && direction != DSHOT_CMD_SPIN_DIRECTION_2) {
    return -RT_EINVAL;  // Invalid direction
  }

  // Don't start new direction change if one is already in progress
  if (!dir_change_done[motor_id]) {
    return -RT_EBUSY;
  }

  // Set target parameters for this motor
  target_direction[motor_id] = direction;
  dshot_config_.motor_dir_ctrl[motor_id].motor_dir = direction;

  // Start direction change state machine for this motor
  dir_change_done[motor_id] = false;
  dir_change_counter[motor_id] = 0;
  dir_change_state[motor_id] = DIR_CHANGE_IDLE_1;
  dir_change_time[motor_id] = rt_tick_get();
  act_dev_.direction_changing[motor_id] = true;

  rt_kprintf("[DSHOT] Motor %d: Starting direction change to %s\n", motor_id,
             (direction == DSHOT_CMD_SPIN_DIRECTION_1) ? "DIRECTION_1" : "DIRECTION_2");

  return RT_EOK;
}

/* Write DShot value to single channel */
void writeDshotValueSingleChannle(uint8_t chan_id, rt_uint16_t dc) { dshot_config_.dshot_output_value[chan_id] = dc; }

/* Motor stop function */
void motor_stop(void) {
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    writeDshotValueSingleChannle(i, DSHOT_CMD_MOTOR_STOP);
  }
}

/* Handle direction change state machine */
void dshot_handle_dir_change(uint8_t motor_index) {
  switch (dir_change_state[motor_index]) {
    case DIR_CHANGE_IDLE_1:
      // First idle period - 10ms, bus needs to be idle
      if ((rt_tick_get() - dir_change_time[motor_index]) < rt_tick_from_millisecond(DSHOT_DIR_CHANGE_IDLE_TIME_MS)) {
        setMotorBusIdleState(motor_index, RT_TRUE);
        break;
      } else {
        rt_kprintf("[DSHOT] Motor %d first idle complete, sending direction commands\n", motor_index);
        dir_change_state[motor_index] = DIR_CHANGE_CMD;
        dir_change_counter[motor_index] = 0;
        dir_change_time[motor_index] = rt_tick_get();
        // State will be handled in next call - no bus state change here
        break;
      }
    case DIR_CHANGE_CMD: {
      if ((rt_tick_get() - dir_change_time[motor_index]) < rt_tick_from_millisecond(DSHOT_DIR_CHANGE_CMD_TIME_MS)) {
        setMotorBusIdleState(motor_index, RT_TRUE);  // Set bus to idle during interval
        break;
      } else if (dir_change_counter[motor_index] < 10) {
        setMotorBusIdleState(motor_index, RT_FALSE);  // Enable command sending
        writeDshotValueSingleChannle(motor_index, target_direction[motor_index]);
        dir_change_counter[motor_index]++;
        rt_kprintf("[DSHOT] Motor %d direction command %d/10\n", motor_index, dir_change_counter[motor_index]);
        dir_change_time[motor_index] = rt_tick_get();
        break;
      } else {
        rt_kprintf("[DSHOT] Motor %d direction commands sent, entering second idle phase\n", motor_index);
        dir_change_state[motor_index] = DIR_CHANGE_IDLE_2;
        dir_change_counter[motor_index] = 0;
        dir_change_time[motor_index] = rt_tick_get();
        setMotorBusIdleState(motor_index, RT_TRUE);  // Enter idle state
        break;
      }
    }

    case DIR_CHANGE_IDLE_2: {
      if ((rt_tick_get() - dir_change_time[motor_index]) < rt_tick_from_millisecond(DSHOT_DIR_CHANGE_IDLE_TIME_MS)) {
        setMotorBusIdleState(motor_index, RT_TRUE);  // Set bus to idle
        break;
      } else {
        rt_kprintf("[DSHOT] Motor %d second idle complete, sending save commands\n", motor_index);
        dir_change_state[motor_index] = DIR_CHANGE_SAVE;
        dir_change_counter[motor_index] = 0;
        dir_change_time[motor_index] = rt_tick_get();
        // State will be handled in next call - no bus state change here
        break;
      }
    }

    case DIR_CHANGE_SAVE: {
      if ((rt_tick_get() - dir_change_time[motor_index]) < rt_tick_from_millisecond(DSHOT_SAVE_SETTINGS_DELAY_MS)) {
        setMotorBusIdleState(motor_index, RT_TRUE);  // Set bus to idle during interval
        break;
      } else if (dir_change_counter[motor_index] < 10) {
        setMotorBusIdleState(motor_index, RT_FALSE);  // Enable command sending
        writeDshotValueSingleChannle(motor_index, DSHOT_CMD_SAVE_SETTINGS);
        dir_change_counter[motor_index]++;
        rt_kprintf("[DSHOT] Motor %d save settings %d/10\n", motor_index, dir_change_counter[motor_index]);
        dir_change_time[motor_index] = rt_tick_get();
        break;
      } else {
        rt_kprintf("[DSHOT] Motor %d save settings complete\n", motor_index);
        dir_change_state[motor_index] = DIR_CHANGE_STOP;
        dir_change_counter[motor_index] = 0;
        // State will be handled in next call - final cleanup pending
        break;
      }
    }

    case DIR_CHANGE_STOP:
      dir_change_done[motor_index] = true;
      dir_change_state[motor_index] = DIR_CHANGE_IDLE_1;
      act_dev_.direction_changing[motor_index] = false;
      setMotorBusIdleState(motor_index, RT_FALSE);
      rt_kprintf("[DSHOT] Direction change completed for motor %d\n", motor_index);
      break;
  }
}

/* Legacy motor reverse function (for backward compatibility) */
rt_err_t motor_reverse(uint8_t *arg) {
  // Fix: arg is a pointer to uint8_t, not a string pointer
  uint8_t motor_id = *arg;  // Directly dereference the pointer to get the motor ID
  if (motor_id >= DSHOT_MOTOR_NUMS) {
    return -RT_EINVAL;
  }

  // Simple alternating logic for backward compatibility
  uint8_t new_direction;
  if (dshot_config_.motor_dir_ctrl[motor_id].motor_dir == DSHOT_CMD_SPIN_DIRECTION_1) {
    new_direction = DSHOT_CMD_SPIN_DIRECTION_2;
  } else {
    new_direction = DSHOT_CMD_SPIN_DIRECTION_1;
  }

  return motor_reverse_with_direction(motor_id, new_direction);
}