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

#include "MulticopterRateControl.hpp"

// Removed PX4 specific includes
// #include <drivers/drv_hrt.h>  // PX4 specific, commented out
// #include <circuit_breaker/circuit_breaker.h>  // PX4 specific, commented out
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
// #include <px4_platform_common/events.h>  // PX4 specific, commented out
#include <cmath>
#include <cfloat>

using namespace matrix;
// using namespace time_literals;  // PX4 specific, commented out
using math::radians;

// Helper function for superexpo (from PX4 mathlib)
static inline float superexpo(float stick_input, float expo, float supexpo)
{
	if (expo <= 0.0f) {
		return stick_input;
	}

	float expo_stick = stick_input * (1.0f - expo) + stick_input * stick_input * stick_input * expo;

	if (supexpo <= 0.0f) {
		return expo_stick;
	}

	return expo_stick * (1.0f - supexpo) + expo_stick * expo_stick * expo_stick * supexpo;
}

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	// Removed PX4 inheritance initialization
	// ModuleParams(nullptr),  // PX4 specific, removed
	// WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),  // PX4 specific, removed
	// _vehicle_thrust_setpoint_pub(...),  // PX4 specific, removed
	// _vehicle_torque_setpoint_pub(...),  // PX4 specific, removed
	// _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))  // PX4 specific, removed
	_vehicle_status{},
	_landed(true),
	_maybe_landed(true),
	_battery_status_scale(0.0f),
	_energy_integration_time(0.0f),
	_yaw_lpf_state(0.0f),
	_yaw_lpf_cutoff(MC_YAW_TQ_CUTOFF)
{
	_vehicle_status.vehicle_type = 1; // VEHICLE_TYPE_ROTARY_WING (simplified)

	// Initialize parameters from header defines
	_param_mc_rollrate_p = MC_ROLLRATE_P;
	_param_mc_rollrate_i = MC_ROLLRATE_I;
	_param_mc_rr_int_lim = MC_RR_INT_LIM;
	_param_mc_rollrate_d = MC_ROLLRATE_D;
	_param_mc_rollrate_ff = MC_ROLLRATE_FF;
	_param_mc_rollrate_k = MC_ROLLRATE_K;

	_param_mc_pitchrate_p = MC_PITCHRATE_P;
	_param_mc_pitchrate_i = MC_PITCHRATE_I;
	_param_mc_pr_int_lim = MC_PR_INT_LIM;
	_param_mc_pitchrate_d = MC_PITCHRATE_D;
	_param_mc_pitchrate_ff = MC_PITCHRATE_FF;
	_param_mc_pitchrate_k = MC_PITCHRATE_K;

	_param_mc_yawrate_p = MC_YAWRATE_P;
	_param_mc_yawrate_i = MC_YAWRATE_I;
	_param_mc_yr_int_lim = MC_YR_INT_LIM;
	_param_mc_yawrate_d = MC_YAWRATE_D;
	_param_mc_yawrate_ff = MC_YAWRATE_FF;
	_param_mc_yawrate_k = MC_YAWRATE_K;

	_param_mc_yaw_tq_cutoff = MC_YAW_TQ_CUTOFF;
	_param_mc_bat_scale_en = (MC_BAT_SCALE_EN != 0);

	_param_mc_acro_r_max = MC_ACRO_R_MAX;
	_param_mc_acro_p_max = MC_ACRO_P_MAX;
	_param_mc_acro_y_max = MC_ACRO_Y_MAX;
	_param_mc_acro_expo = MC_ACRO_EXPO;
	_param_mc_acro_expo_y = MC_ACRO_EXPO_Y;
	_param_mc_acro_supexpo = MC_ACRO_SUPEXPO;
	_param_mc_acro_supexpoy = MC_ACRO_SUPEXPOY;

	for (int i = 0; i < 4; i++) {
		_control_energy[i] = 0.0f;
	}

	parameters_updated();
	// _controller_status_pub.advertise();  // PX4 specific, removed
}

MulticopterRateControl::~MulticopterRateControl()
{
	// perf_free(_loop_perf);  // PX4 specific, removed
}

bool
MulticopterRateControl::init()
{
	// Removed uORB callback registration
	// if (!_vehicle_angular_velocity_sub.registerCallback()) {  // PX4 specific, removed
	// 	PX4_ERR("callback registration failed");  // PX4 specific, removed
	// 	return false;
	// }

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k, _param_mc_pitchrate_k, _param_mc_yawrate_k);

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p, _param_mc_pitchrate_p, _param_mc_yawrate_p)),
		rate_k.emult(Vector3f(_param_mc_rollrate_i, _param_mc_pitchrate_i, _param_mc_yawrate_i)),
		rate_k.emult(Vector3f(_param_mc_rollrate_d, _param_mc_pitchrate_d, _param_mc_yawrate_d)));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim, _param_mc_pr_int_lim, _param_mc_yr_int_lim));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff, _param_mc_pitchrate_ff, _param_mc_yawrate_ff));

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max), radians(_param_mc_acro_p_max),
				  radians(_param_mc_acro_y_max));

	_yaw_lpf_cutoff = _param_mc_yaw_tq_cutoff;
	// _output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff);  // PX4 specific, replaced with simple filter
}

bool
MulticopterRateControl::update(const VehicleAngularVelocity &angular_velocity,
				const VehicleRatesSetpoint &rates_setpoint_input,
				const float thrust_setpoint_input[3],
				const VehicleControlMode &control_mode,
				bool landed,
				float dt,
				float torque_setpoint[3],
				float thrust_setpoint_out[3])
{
	// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
	dt = math::constrain(dt, 0.000125f, 0.02f);

	const Vector3f rates{angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]};
	const Vector3f angular_accel{angular_velocity.xyz_derivative[0],
				     angular_velocity.xyz_derivative[1],
				     angular_velocity.xyz_derivative[2]};

	// Update local state
	_vehicle_control_mode = control_mode;
	_landed = landed;
	_maybe_landed = landed;  // Simplified

	// Process rates setpoint
	_rates_setpoint(0) = (std::isfinite(rates_setpoint_input.roll)) ? rates_setpoint_input.roll : rates(0);
	_rates_setpoint(1) = (std::isfinite(rates_setpoint_input.pitch)) ? rates_setpoint_input.pitch : rates(1);
	_rates_setpoint(2) = (std::isfinite(rates_setpoint_input.yaw)) ? rates_setpoint_input.yaw : rates(2);

	_thrust_setpoint = Vector3f(thrust_setpoint_input[0], thrust_setpoint_input[1], thrust_setpoint_input[2]);

	// Removed manual control setpoint processing (ACRO mode) - can be added later if needed
	// if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
	// 	// generate the rate setpoint from sticks
	// 	// This would process manual_control_setpoint and convert to rate setpoint
	// }

	// run the rate controller
	if (_vehicle_control_mode.flag_control_rates_enabled) {

		// reset integral if disarmed
		if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != 1) {  // VEHICLE_TYPE_ROTARY_WING
			_rate_control.resetIntegral();
		}

		// Removed control allocator status update - can be set via setSaturationStatus()
		// if (_control_allocator_status_sub.update(&control_allocator_status)) {
		// 	...
		// }

		// run rate controller
		Vector3f torque_setpoint_vec =
			_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

		// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
		torque_setpoint_vec(2) = applyYawLowPassFilter(torque_setpoint_vec(2), dt);

		// Removed rate controller status publishing
		// rate_ctrl_status_s rate_ctrl_status{};
		// _rate_control.getRateControlStatus(rate_ctrl_status);
		// rate_ctrl_status.timestamp = hrt_absolute_time();
		// _controller_status_pub.publish(rate_ctrl_status);

		// Copy thrust setpoint
		for (int i = 0; i < 3; i++) {
			thrust_setpoint_out[i] = _thrust_setpoint(i);
		}

		// Copy torque setpoint
		torque_setpoint[0] = (std::isfinite(torque_setpoint_vec(0))) ? torque_setpoint_vec(0) : 0.f;
		torque_setpoint[1] = (std::isfinite(torque_setpoint_vec(1))) ? torque_setpoint_vec(1) : 0.f;
		torque_setpoint[2] = (std::isfinite(torque_setpoint_vec(2))) ? torque_setpoint_vec(2) : 0.f;

		// scale setpoints by battery status if enabled
		if (_param_mc_bat_scale_en) {
			if (_battery_status_scale > 0.f) {
				for (int i = 0; i < 3; i++) {
					thrust_setpoint_out[i] = math::constrain(thrust_setpoint_out[i] * _battery_status_scale, -1.f, 1.f);
					torque_setpoint[i] = math::constrain(torque_setpoint[i] * _battery_status_scale, -1.f, 1.f);
				}
			}
		}

		// Removed actuator controls status update
		// updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		return true;
	}

	return false;
}

void
MulticopterRateControl::resetIntegral()
{
	_rate_control.resetIntegral();
}

void
MulticopterRateControl::setSaturationStatus(const bool saturation_positive[3],
					     const bool saturation_negative[3])
{
	Vector<bool, 3> sat_pos;
	Vector<bool, 3> sat_neg;
	for (int i = 0; i < 3; i++) {
		sat_pos(i) = saturation_positive[i];
		sat_neg(i) = saturation_negative[i];
	}
	_rate_control.setSaturationStatus(sat_pos, sat_neg);
}

void
MulticopterRateControl::setBatteryScale(float scale)
{
	_battery_status_scale = scale;
}

float
MulticopterRateControl::applyYawLowPassFilter(float input, float dt)
{
	// Simple first-order low-pass filter
	// Replaces AlphaFilter from PX4
	if (_yaw_lpf_cutoff > 0.0f && dt > 0.0f) {
		float alpha = dt / (1.0f / (2.0f * 3.14159f * _yaw_lpf_cutoff) + dt);
		_yaw_lpf_state = _yaw_lpf_state + alpha * (input - _yaw_lpf_state);
		return _yaw_lpf_state;
	}
	return input;
}

// Removed PX4 module functions
// int MulticopterRateControl::task_spawn(int argc, char *argv[])
// {
// 	// PX4 specific, removed
// }
//
// int MulticopterRateControl::custom_command(int argc, char *argv[])
// {
// 	// PX4 specific, removed
// }
//
// int MulticopterRateControl::print_usage(const char *reason)
// {
// 	// PX4 specific, removed
// }
//
// extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
// {
// 	// PX4 specific, removed
// }
