#ifndef APPLICATIONS_IMU_DATA_TYPES_H
#define APPLICATIONS_IMU_DATA_TYPES_H

#include <rtthread.h>

#include "imu_config.h"

typedef struct imu_raw_frame
{
    rt_uint32_t timestamp_ms;
    rt_int16_t acc_x;
    rt_int16_t acc_y;
    rt_int16_t acc_z;
    rt_int16_t gyro_x;
    rt_int16_t gyro_y;
    rt_int16_t gyro_z;
    rt_int16_t temp_raw;
} imu_raw_frame_t;

typedef struct imu_fifo_batch
{
    rt_uint32_t poll_timestamp_ms;
    rt_uint16_t pkt_cnt;
    imu_raw_frame_t frames[IMU_DEFAULT_FIFO_DEPTH];
} imu_fifo_batch_t;

#endif
