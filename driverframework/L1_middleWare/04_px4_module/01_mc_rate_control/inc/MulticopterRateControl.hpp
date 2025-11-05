/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/rate_control/rate_control.hpp>
// #include <lib/mathlib/math/filter/AlphaFilter.hpp>  // TODO: Need to check if available
#include <lib/matrix/matrix/math.hpp>
// #include <lib/perf/perf_counter.h>  // PX4 specific, commented out
// #include <px4_platform_common/defines.h>  // PX4 specific, commented out
// #include <px4_platform_common/module.h>  // PX4 specific, commented out
// #include <px4_platform_common/module_params.h>  // PX4 specific, commented out
// #include <px4_platform_common/posix.h>  // PX4 specific, commented out
// #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>  // PX4 specific, commented out
// #include <lib/systemlib/mavlink_log.h>  // PX4 specific, commented out
// #include <uORB/Publication.hpp>  // PX4 specific, commented out
// #include <uORB/PublicationMulti.hpp>  // PX4 specific, commented out
// #include <uORB/Subscription.hpp>  // PX4 specific, commented out
// #include <uORB/SubscriptionCallback.hpp>  // PX4 specific, commented out
// PX4 uORB topics - commented out, replaced with direct data structures
// #include <uORB/topics/actuator_controls_status.h>
// #include <uORB/topics/battery_status.h>
// #include <uORB/topics/control_allocator_status.h>
// #include <uORB/topics/manual_control_setpoint.h>
// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/rate_ctrl_status.h>
// #include <uORB/topics/vehicle_angular_velocity.h>
// #include <uORB/topics/vehicle_control_mode.h>
// #include <uORB/topics/vehicle_land_detected.h>
// #include <uORB/topics/vehicle_rates_setpoint.h>
// #include <uORB/topics/vehicle_status.h>
// #include <uORB/topics/vehicle_thrust_setpoint.h>
// #include <uORB/topics/vehicle_torque_setpoint.h>

#include "mc_rate_control_params.h"

// Simple data structures to replace uORB topics
struct VehicleAngularVelocity {
    float xyz[3];  // rad/s
    float xyz_derivative[3];  // rad/s^2
    uint64_t timestamp_sample;
};

struct VehicleRatesSetpoint {
    float roll;  // rad/s
    float pitch;  // rad/s
    float yaw;  // rad/s
    float thrust_body[3];  // normalized [-1, 1]
    uint64_t timestamp;
};

struct VehicleControlMode {
    bool flag_control_manual_enabled;
    bool flag_control_attitude_enabled;
    bool flag_control_rates_enabled;
    bool flag_armed;
};

struct VehicleStatus {
    uint8_t vehicle_type;
};

struct VehicleLandDetected {
    bool landed;
    bool maybe_landed;
};

struct ManualControlSetpoint {
    float roll;  // normalized [-1, 1]
    float pitch;  // normalized [-1, 1]
    float yaw;  // normalized [-1, 1]
    float throttle;  // normalized [-1, 1]
};

struct ControlAllocatorStatus {
    bool torque_setpoint_achieved;
    float unallocated_torque[3];
};

struct BatteryStatus {
    bool connected;
    float scale;
};

// Output structures
struct VehicleThrustSetpoint {
    float xyz[3];  // normalized [-1, 1]
    uint64_t timestamp_sample;
    uint64_t timestamp;
};

struct VehicleTorqueSetpoint {
    float xyz[3];  // normalized [-1, 1]
    uint64_t timestamp_sample;
    uint64_t timestamp;
};

// using namespace time_literals;  // PX4 specific, commented out

// Simplified class without PX4 dependencies
// Removed: ModuleBase, ModuleParams, WorkItem inheritance
class MulticopterRateControl
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl();

	// Simplified initialization
	bool init();

	/**
	 * @brief Run one control cycle
	 * @param angular_velocity Current angular velocity (rad/s) and derivative (rad/s^2)
	 * @param rates_setpoint Desired rate setpoint (rad/s)
	 * @param thrust_setpoint Desired thrust setpoint (normalized [-1, 1])
	 * @param control_mode Current control mode flags
	 * @param landed Whether vehicle is landed
	 * @param dt Time step in seconds
	 * @param torque_setpoint Output: computed torque setpoint (normalized [-1, 1])
	 * @param thrust_setpoint_out Output: thrust setpoint (may be modified by battery scaling)
	 * @return true if control was successful
	 */
	bool update(const VehicleAngularVelocity &angular_velocity,
		    const VehicleRatesSetpoint &rates_setpoint_input,
		    const float thrust_setpoint_input[3],
		    const VehicleControlMode &control_mode,
		    bool landed,
		    float dt,
		    float torque_setpoint[3],
		    float thrust_setpoint_out[3]);

	/**
	 * @brief Update parameters (called after parameter changes)
	 */
	void parameters_updated();

	/**
	 * @brief Reset integrator (call when disarming)
	 */
	void resetIntegral();

	/**
	 * @brief Set saturation status from control allocator
	 */
	void setSaturationStatus(const bool saturation_positive[3],
				 const bool saturation_negative[3]);

	/**
	 * @brief Set battery scale (for battery compensation)
	 */
	void setBatteryScale(float scale);

private:
	RateControl _rate_control; ///< class for rate control calculations

	// Removed uORB subscriptions - data passed directly via update() function
	// uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	// uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	// uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	// uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	// uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	// uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	// uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	// uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Removed uORB publications - outputs passed via function parameters
	// uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	// uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	// uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	// uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;
	// uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;

	// Local state storage (replacing uORB subscriptions)
	VehicleControlMode	_vehicle_control_mode{};
	VehicleStatus		_vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};

	// uint64_t _last_run{0};  // Removed, dt passed directly

	// perf_counter_t	_loop_perf;  // PX4 specific, commented out

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};
	matrix::Vector3f _thrust_setpoint{};

	float _battery_status_scale{0.0f};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	// AlphaFilter<float> _output_lpf_yaw;  // TODO: Need to implement or find alternative
	// Simple first-order low-pass filter for yaw
	float _yaw_lpf_state{0.0f};
	float _yaw_lpf_cutoff{MC_YAW_TQ_CUTOFF};

	// Parameters (now using compile-time constants from header)
	// Removed: DEFINE_PARAMETERS macro - using header defines instead
	float _param_mc_rollrate_p;
	float _param_mc_rollrate_i;
	float _param_mc_rr_int_lim;
	float _param_mc_rollrate_d;
	float _param_mc_rollrate_ff;
	float _param_mc_rollrate_k;

	float _param_mc_pitchrate_p;
	float _param_mc_pitchrate_i;
	float _param_mc_pr_int_lim;
	float _param_mc_pitchrate_d;
	float _param_mc_pitchrate_ff;
	float _param_mc_pitchrate_k;

	float _param_mc_yawrate_p;
	float _param_mc_yawrate_i;
	float _param_mc_yr_int_lim;
	float _param_mc_yawrate_d;
	float _param_mc_yawrate_ff;
	float _param_mc_yawrate_k;

	float _param_mc_yaw_tq_cutoff;
	bool _param_mc_bat_scale_en;

	float _param_mc_acro_r_max;
	float _param_mc_acro_p_max;
	float _param_mc_acro_y_max;
	float _param_mc_acro_expo;
	float _param_mc_acro_expo_y;
	float _param_mc_acro_supexpo;
	float _param_mc_acro_supexpoy;

	// Helper function for yaw low-pass filter
	float applyYawLowPassFilter(float input, float dt);
};
