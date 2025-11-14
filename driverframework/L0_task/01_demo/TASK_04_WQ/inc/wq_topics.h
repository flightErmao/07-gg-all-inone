#ifndef WQ_TOPICS_H__
#define WQ_TOPICS_H__

#include <rtthread.h>
#include <uMCN.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel[3];
    float gyro[3];
    rt_uint32_t seq;
} imu_raw_msg_t;

typedef struct {
    float velocity[3];
    rt_uint32_t seq;
} vehicle_accelerate_velocity_msg_t;

typedef struct {
    float actuator_outputs[4];
    rt_uint32_t seq;
} rate_ctrl_actuator_cmd_msg_t;

MCN_DECLARE(imu_raw);
MCN_DECLARE(vehicle_accelerate_velocity);
MCN_DECLARE(rate_ctrl_actuator_cmd);

#ifdef __cplusplus
}
#endif

#endif /* WQ_TOPICS_H__ */


