/****************************************************************************
 *
 * C wrapper for MulticopterRateControl C++ class
 * This file provides C interface to be used in C task files
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"
#include <rtthread.h>
#include <string.h>

extern "C" {
    // Global instance pointer
    static MulticopterRateControl* g_rate_control_instance = nullptr;

    /**
     * @brief Initialize MulticopterRateControl instance
     * @param vtol VTOL mode flag
     * @return 0 on success, -1 on failure
     */
    int multicopter_rate_control_init(int vtol) {
        if (g_rate_control_instance != nullptr) {
            // Already initialized
            return 0;
        }

        g_rate_control_instance = new MulticopterRateControl(vtol != 0);
        
        if (g_rate_control_instance == nullptr) {
            return -1;
        }

        if (!g_rate_control_instance->init()) {
            delete g_rate_control_instance;
            g_rate_control_instance = nullptr;
            return -1;
        }

        return 0;
    }

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
        float thrust_setpoint_out[3])
    {
        if (g_rate_control_instance == nullptr) {
            return 0;
        }

        VehicleAngularVelocity ang_vel;
        memcpy(ang_vel.xyz, angular_velocity, sizeof(float) * 3);
        memcpy(ang_vel.xyz_derivative, angular_accel, sizeof(float) * 3);
        ang_vel.timestamp_sample = rt_tick_get();  // Use RT-Thread tick

        VehicleRatesSetpoint rates_sp;
        rates_sp.roll = rates_setpoint[0];
        rates_sp.pitch = rates_setpoint[1];
        rates_sp.yaw = rates_setpoint[2];
        rates_sp.timestamp = rt_tick_get();

        VehicleControlMode ctrl_mode;
        ctrl_mode.flag_control_manual_enabled = (control_mode_flags & 0x01) != 0;
        ctrl_mode.flag_control_attitude_enabled = (control_mode_flags & 0x02) != 0;
        ctrl_mode.flag_control_rates_enabled = (control_mode_flags & 0x04) != 0;
        ctrl_mode.flag_armed = (control_mode_flags & 0x08) != 0;

        bool is_landed = (landed != 0);

        bool result = g_rate_control_instance->update(
            ang_vel,
            rates_sp,
            thrust_setpoint,
            ctrl_mode,
            is_landed,
            dt,
            torque_setpoint,
            thrust_setpoint_out);

        return result ? 1 : 0;
    }

    void multicopter_rate_control_step(const float angular_velocity[3], const float angular_accel[3], float dt)
    {
        if (g_rate_control_instance == nullptr) {
            return;
        }

        float rates_setpoint[3] = {0.0f, 0.0f, 0.0f};
        float thrust_setpoint[3] = {0.0f, 0.0f, 0.0f};
        float torque_output[3] = {0.0f, 0.0f, 0.0f};
        float thrust_output[3] = {0.0f, 0.0f, 0.0f};

        VehicleAngularVelocity ang_vel{};
        if (angular_velocity != nullptr) {
            memcpy(ang_vel.xyz, angular_velocity, sizeof(float) * 3);
        }
        if (angular_accel != nullptr) {
            memcpy(ang_vel.xyz_derivative, angular_accel, sizeof(float) * 3);
        }
        ang_vel.timestamp_sample = rt_tick_get();

        VehicleRatesSetpoint rates_sp{};
        rates_sp.roll = rates_setpoint[0];
        rates_sp.pitch = rates_setpoint[1];
        rates_sp.yaw = rates_setpoint[2];
        memcpy(rates_sp.thrust_body, thrust_setpoint, sizeof(float) * 3);
        rates_sp.timestamp = rt_tick_get();

        VehicleControlMode ctrl_mode{};
        ctrl_mode.flag_control_rates_enabled = true;
        ctrl_mode.flag_armed = true;

        g_rate_control_instance->update(
            ang_vel,
            rates_sp,
            thrust_setpoint,
            ctrl_mode,
            false,
            dt,
            torque_output,
            thrust_output);
    }

    /**
     * @brief Reset integrator (call when disarming)
     */
    void multicopter_rate_control_reset_integral(void) {
        if (g_rate_control_instance != nullptr) {
            g_rate_control_instance->resetIntegral();
        }
    }

    /**
     * @brief Set saturation status from control allocator
     * @param saturation_positive Positive saturation flags [3 elements: roll, pitch, yaw]
     * @param saturation_negative Negative saturation flags [3 elements: roll, pitch, yaw]
     */
    void multicopter_rate_control_set_saturation(
        const int saturation_positive[3],
        const int saturation_negative[3])
    {
        if (g_rate_control_instance != nullptr) {
            bool sat_pos[3], sat_neg[3];
            for (int i = 0; i < 3; i++) {
                sat_pos[i] = (saturation_positive[i] != 0);
                sat_neg[i] = (saturation_negative[i] != 0);
            }
            g_rate_control_instance->setSaturationStatus(sat_pos, sat_neg);
        }
    }

    /**
     * @brief Set battery scale (for battery compensation)
     * @param scale Battery scale factor (0.0 to disable)
     */
    void multicopter_rate_control_set_battery_scale(float scale)
    {
        if (g_rate_control_instance != nullptr) {
            g_rate_control_instance->setBatteryScale(scale);
        }
    }

    /**
     * @brief Cleanup MulticopterRateControl instance
     */
    void multicopter_rate_control_cleanup(void) {
        if (g_rate_control_instance != nullptr) {
            delete g_rate_control_instance;
            g_rate_control_instance = nullptr;
        }
    }
}
