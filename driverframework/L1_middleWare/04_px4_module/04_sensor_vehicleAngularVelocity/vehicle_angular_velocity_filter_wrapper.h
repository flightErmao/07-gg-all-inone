#ifndef VEHICLE_ANGULAR_VELOCITY_FILTER_WRAPPER_H
#define VEHICLE_ANGULAR_VELOCITY_FILTER_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float sample_rate_hz;
    float gyro_cutoff_hz;
    float notch0_freq_hz;
    float notch0_bw_hz;
    float notch1_freq_hz;
    float notch1_bw_hz;
    float accel_cutoff_hz;
} imu_filter_params_t;

int vehicle_angular_velocity_filter_init(const imu_filter_params_t *params);
void vehicle_angular_velocity_filter_deinit(void);
void vehicle_angular_velocity_filter_reset(const float gyro[3], const float accel[3]);
void vehicle_angular_velocity_filter_update_params(const imu_filter_params_t *params);
void vehicle_angular_velocity_filter_push_measurement(float dt, const float gyro[3], const float accel[3]);
int vehicle_angular_velocity_filter_get_latest(float gyro[3], float accel[3]);

#ifdef __cplusplus
}
#endif

#endif  // VEHICLE_ANGULAR_VELOCITY_FILTER_WRAPPER_H


