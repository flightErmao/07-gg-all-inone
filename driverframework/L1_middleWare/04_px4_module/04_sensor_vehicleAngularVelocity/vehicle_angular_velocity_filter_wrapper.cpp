#include "vehicle_angular_velocity_filter_wrapper.h"

#include <cmath>
#include <cstring>

#include "mathlib.h"
#include "filter/AlphaFilter.hpp"
#include "filter/LowPassFilter2p.hpp"
#include "filter/NotchFilter.hpp"

namespace {

constexpr float kSampleRateMinHz = 50.0f;
constexpr float kSampleRateMaxHz = 4000.0f;

struct FilterState {
    imu_filter_params_t params{};
    float sample_rate_hz{500.0f};

    math::LowPassFilter2p<float> velocity_lpf[3]{};
    math::NotchFilter<float> velocity_notch0[3]{};
    math::NotchFilter<float> velocity_notch1[3]{};
    AlphaFilter<float> accel_lpf[3]{};

    float filtered_gyro[3]{};
    float filtered_accel[3]{};
    bool configured{false};
};

FilterState *g_filter_state = nullptr;

float constrainf(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

bool is_valid_frequency(float hz) {
    return std::isfinite(hz) && hz > 0.0f;
}

float sanitize_frequency(float hz, float fallback) {
    if (!std::isfinite(hz)) {
        return fallback;
    }
    return hz;
}

void configure_velocity_filters(FilterState &state) {
    const float sample_rate = constrainf(state.sample_rate_hz, kSampleRateMinHz, kSampleRateMaxHz);

    for (int axis = 0; axis < 3; ++axis) {
        if (is_valid_frequency(state.params.gyro_cutoff_hz)) {
            state.velocity_lpf[axis].set_cutoff_frequency(sample_rate, state.params.gyro_cutoff_hz);
        } else {
            state.velocity_lpf[axis].disable();
        }

        if (is_valid_frequency(state.params.notch0_freq_hz) && is_valid_frequency(state.params.notch0_bw_hz)) {
            state.velocity_notch0[axis].setParameters(sample_rate, state.params.notch0_freq_hz, state.params.notch0_bw_hz);
        } else {
            state.velocity_notch0[axis].disable();
        }

        if (is_valid_frequency(state.params.notch1_freq_hz) && is_valid_frequency(state.params.notch1_bw_hz)) {
            state.velocity_notch1[axis].setParameters(sample_rate, state.params.notch1_freq_hz, state.params.notch1_bw_hz);
        } else {
            state.velocity_notch1[axis].disable();
        }

        if (is_valid_frequency(state.params.accel_cutoff_hz)) {
            if (!state.accel_lpf[axis].setCutoffFreq(sample_rate, state.params.accel_cutoff_hz)) {
                state.accel_lpf[axis].setAlpha(1.0f);
            }
        } else {
            state.accel_lpf[axis].setAlpha(1.0f);
        }
    }

    state.configured = true;
}

void reset_filters(FilterState &state, const float gyro[3], const float accel[3]) {
    if (!state.configured) {
        configure_velocity_filters(state);
    }

    const float *gyro_in = gyro ? gyro : state.filtered_gyro;
    const float *accel_in = accel ? accel : state.filtered_accel;

    for (int axis = 0; axis < 3; ++axis) {
        const float gyro_sample = std::isfinite(gyro_in[axis]) ? gyro_in[axis] : 0.0f;
        const float accel_sample = std::isfinite(accel_in[axis]) ? accel_in[axis] : 0.0f;

        state.filtered_gyro[axis] = state.velocity_lpf[axis].reset(gyro_sample);
        state.velocity_notch0[axis].reset(state.filtered_gyro[axis]);
        state.velocity_notch1[axis].reset(state.filtered_gyro[axis]);
        state.accel_lpf[axis].reset(accel_sample);
        state.filtered_accel[axis] = state.accel_lpf[axis].getState();
    }
}

void update_sample_rate(FilterState &state, float dt) {
    if (dt <= 0.0f || !std::isfinite(dt)) {
        return;
    }

    const float new_sample_rate = constrainf(1.0f / dt, kSampleRateMinHz, kSampleRateMaxHz);

    const float delta = fabsf(new_sample_rate - state.sample_rate_hz);
    if (delta > (0.01f * state.sample_rate_hz)) {
        state.sample_rate_hz = new_sample_rate;
        configure_velocity_filters(state);
    }
}

void update_filters(FilterState &state, const float gyro[3], const float accel[3]) {
    const float gyro_zero[3] = {0.0f, 0.0f, 0.0f};
    const float accel_zero[3] = {0.0f, 0.0f, 0.0f};

    const float *gyro_in = gyro ? gyro : gyro_zero;
    const float *accel_in = accel ? accel : accel_zero;

    for (int axis = 0; axis < 3; ++axis) {
        float value = gyro_in[axis];

        if (!std::isfinite(value)) {
            value = state.filtered_gyro[axis];
        }

        value = state.velocity_notch0[axis].apply(value);
        value = state.velocity_notch1[axis].apply(value);
        value = state.velocity_lpf[axis].apply(value);

        state.filtered_gyro[axis] = value;

        float accel_value = accel_in[axis];
        if (!std::isfinite(accel_value)) {
            accel_value = state.filtered_accel[axis];
        }
        state.filtered_accel[axis] = state.accel_lpf[axis].update(accel_value);
    }
}

}  // namespace

int vehicle_angular_velocity_filter_init(const imu_filter_params_t *params) {
    if (g_filter_state != nullptr) {
        return 0;
    }

    g_filter_state = new FilterState();

    if (g_filter_state == nullptr) {
        return -1;
    }

    if (params != nullptr) {
        g_filter_state->params = *params;
    } else {
        g_filter_state->params.sample_rate_hz = 500.0f;
        g_filter_state->params.gyro_cutoff_hz = 80.0f;
        g_filter_state->params.notch0_freq_hz = 0.0f;
        g_filter_state->params.notch0_bw_hz = 0.0f;
        g_filter_state->params.notch1_freq_hz = 0.0f;
        g_filter_state->params.notch1_bw_hz = 0.0f;
        g_filter_state->params.accel_cutoff_hz = 30.0f;
    }

    g_filter_state->sample_rate_hz = sanitize_frequency(g_filter_state->params.sample_rate_hz, 500.0f);

    configure_velocity_filters(*g_filter_state);
    reset_filters(*g_filter_state, nullptr, nullptr);

    return 0;
}

void vehicle_angular_velocity_filter_deinit(void) {
    delete g_filter_state;
    g_filter_state = nullptr;
}

void vehicle_angular_velocity_filter_reset(const float gyro[3], const float accel[3]) {
    if (g_filter_state == nullptr) {
        return;
    }

    reset_filters(*g_filter_state, gyro, accel);
}

void vehicle_angular_velocity_filter_update_params(const imu_filter_params_t *params) {
    if ((g_filter_state == nullptr) || (params == nullptr)) {
        return;
    }

    g_filter_state->params = *params;
    g_filter_state->sample_rate_hz = sanitize_frequency(params->sample_rate_hz, g_filter_state->sample_rate_hz);
    configure_velocity_filters(*g_filter_state);
}

void vehicle_angular_velocity_filter_push_measurement(float dt, const float gyro[3], const float accel[3]) {
    if (g_filter_state == nullptr) {
        return;
    }

    update_sample_rate(*g_filter_state, dt);
    update_filters(*g_filter_state, gyro, accel);
}

int vehicle_angular_velocity_filter_get_latest(float gyro[3], float accel[3]) {
    if (g_filter_state == nullptr) {
        return -1;
    }

    if (gyro != nullptr) {
        std::memcpy(gyro, g_filter_state->filtered_gyro, sizeof(g_filter_state->filtered_gyro));
    }

    if (accel != nullptr) {
        std::memcpy(accel, g_filter_state->filtered_accel, sizeof(g_filter_state->filtered_accel));
    }

    return 0;
}


