#ifndef APPLICATIONS_IMU_READER_THREAD_H
#define APPLICATIONS_IMU_READER_THREAD_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int imu_reader_thread_probe_count(void);
int imu_reader_thread_start(void);
int imu_reader_thread_start_for_test(const char *test_name);
int imu_reader_thread_set_duration_ms(rt_uint32_t duration_ms);
int imu_reader_thread_stop(void);
int imu_reader_thread_is_recording(void);
rt_uint32_t imu_reader_thread_recorded_frames(void);
rt_uint32_t imu_reader_thread_duration_ms(void);
int imu_reader_thread_last_error(void);
int imu_reader_thread_last_run_ok(void);
const char *imu_reader_thread_output_path(void);
const char *imu_reader_thread_output_dir(void);
rt_uint32_t imu_reader_thread_output_index(void);
rt_uint32_t imu_reader_thread_flush_count(void);
rt_uint32_t imu_reader_thread_max_gap_ms(void);

#ifdef __cplusplus
}
#endif

#endif
