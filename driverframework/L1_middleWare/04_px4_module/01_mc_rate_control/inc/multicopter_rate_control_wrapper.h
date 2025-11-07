/****************************************************************************
 *
 * C wrapper header for MulticopterRateControl C++ class
 *
 ****************************************************************************/

#ifndef __MULTICOPTER_RATE_CONTROL_WRAPPER_H__
#define __MULTICOPTER_RATE_CONTROL_WRAPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MulticopterRateControl instance
 * @param vtol VTOL mode flag (0 = false, non-zero = true)
 * @return 0 on success, -1 on failure
 */
int multicopter_rate_control_init(int vtol);

/**
 * @brief Run one cycle of rate control
 * @param angular_velocity Current angular velocity (rad/s) [3 elements: roll, pitch, yaw]
 * @param angular_accel Current angular acceleration (rad/s^2) [3 elements]
 * @param rates_setpoint Desired rate setpoint (rad/s) [3 elements: roll, pitch, yaw]
 * @param thrust_setpoint Desired thrust setpoint (normalized [-1, 1]) [3 elements: x, y, z]
 * @param control_mode_flags Control mode flags: bit0=manual, bit1=attitude, bit2=rates, bit3=armed
 * @param landed Whether vehicle is landed (0=airborne, 1=landed)
 * @param dt Time step in seconds
 * @param torque_setpoint Output: computed torque setpoint (normalized [-1, 1]) [3 elements: roll, pitch, yaw]
 * @param thrust_setpoint_out Output: thrust setpoint (may be modified) [3 elements]
 * @return 1 if control was successful, 0 otherwise
 */
int multicopter_rate_control_update(
    const float angular_velocity[3],
    const float angular_accel[3],
    const float rates_setpoint[3],
    const float thrust_setpoint[3],
    unsigned int control_mode_flags,
    int landed,
    float dt,
    float torque_setpoint[3],
    float thrust_setpoint_out[3]);

void multicopter_rate_control_step(const float angular_velocity[3], const float angular_accel[3], float dt);

/**
 * @brief Reset integrator (call when disarming)
 */
void multicopter_rate_control_reset_integral(void);

/**
 * @brief Set saturation status from control allocator
 * @param saturation_positive Positive saturation flags [3 elements: roll, pitch, yaw]
 * @param saturation_negative Negative saturation flags [3 elements: roll, pitch, yaw]
 */
void multicopter_rate_control_set_saturation(
    const int saturation_positive[3],
    const int saturation_negative[3]);

/**
 * @brief Set battery scale (for battery compensation)
 * @param scale Battery scale factor (0.0 to disable)
 */
void multicopter_rate_control_set_battery_scale(float scale);

/**
 * @brief Cleanup MulticopterRateControl instance
 */
void multicopter_rate_control_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __MULTICOPTER_RATE_CONTROL_WRAPPER_H__ */
