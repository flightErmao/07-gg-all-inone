#include "rateCtrlVelocity.h"

extern "C" {
#include <rtthread.h>
#define LOG_TAG "rate_vel"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

#include "workqueueManage.h"

// Matrix types are available through LowPassFilter2p.hpp
using namespace matrix;

MCN_DEFINE(vehicle_accelerate_velocity, sizeof(vehicle_accelerate_velocity_msg_t));

// Filter parameters: sample_freq = 333Hz (3ms interval), cutoff_freq = 30Hz
#define VELOCITY_FILTER_SAMPLE_FREQ 333.0f
#define VELOCITY_FILTER_CUTOFF_FREQ 30.0f

RateCtrlVelocity::RateCtrlVelocity()
    : imu_node_(RT_NULL),
      accel_filter_(VELOCITY_FILTER_SAMPLE_FREQ, VELOCITY_FILTER_CUTOFF_FREQ),
      gyro_filter_(VELOCITY_FILTER_SAMPLE_FREQ, VELOCITY_FILTER_CUTOFF_FREQ)
{
    std::memset(&latest_imu_, 0, sizeof(latest_imu_));
    rt_work_init(&work_, RateCtrlVelocity::workHandler, this);
}

rt_err_t RateCtrlVelocity::init()
{
    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        LOG_E("workqueue init failed (%d)", ret);
        return ret;
    }

    ret = mcn_advertise(MCN_HUB(vehicle_accelerate_velocity), RT_NULL);
    if (ret != RT_EOK && ret != -RT_EBUSY) {
        LOG_E("advertise velocity topic failed (%d)", ret);
        return ret;
    }

    imu_node_ = mcn_subscribe(MCN_HUB(imu_raw), RT_NULL, RT_NULL);
    if (imu_node_ == RT_NULL) {
        LOG_E("subscribe imu topic failed");
        return -RT_ERROR;
    }

    ret = mcn_register_async_cb(imu_node_, RateCtrlVelocity::asyncCallback, this);
    if (ret != RT_EOK) {
        LOG_E("register imu callback failed (%d)", ret);
        return ret;
    }

    LOG_I("initialized");
    return RT_EOK;
}

void RateCtrlVelocity::workHandler(struct rt_work* work, void* parameter)
{
    RT_UNUSED(work);
    if (parameter == RT_NULL) {
        return;
    }

    static_cast<RateCtrlVelocity*>(parameter)->handleWork();
}

void RateCtrlVelocity::asyncCallback(const void* data, void* user_data)
{
    if ((data == RT_NULL) || (user_data == RT_NULL)) {
        return;
    }

    RateCtrlVelocity* instance = static_cast<RateCtrlVelocity*>(user_data);
    std::memcpy(&instance->latest_imu_, data, sizeof(instance->latest_imu_));

    if (wq_add_work(&instance->work_) != RT_EOK) {
        LOG_E("submit velocity work failed");
    }
}

void RateCtrlVelocity::handleWork()
{
    vehicle_accelerate_velocity_msg_t velocity_msg = { 0 };
    velocity_msg.seq = latest_imu_.seq;

    // Convert array to Vector3f
    Vector3f accel_raw(latest_imu_.accel[0], latest_imu_.accel[1], latest_imu_.accel[2]);
    Vector3f gyro_raw(latest_imu_.gyro[0], latest_imu_.gyro[1], latest_imu_.gyro[2]);

    // Apply low pass filters
    Vector3f accel_filtered = accel_filter_.apply(accel_raw);
    Vector3f gyro_filtered = gyro_filter_.apply(gyro_raw);

    // Calculate velocity: weighted combination of filtered accel and gyro
    Vector3f velocity = 0.8f * accel_filtered + 0.2f * gyro_filtered;

    // Convert back to array for message
    velocity_msg.velocity[0] = velocity(0);
    velocity_msg.velocity[1] = velocity(1);
    velocity_msg.velocity[2] = velocity(2);

    if (mcn_publish(MCN_HUB(vehicle_accelerate_velocity), &velocity_msg) != RT_EOK) {
        LOG_E("publish velocity failed");
    } else {
        LOG_I("seq:%u velocity(%.3f, %.3f, %.3f)",
            velocity_msg.seq,
            velocity_msg.velocity[0],
            velocity_msg.velocity[1],
            velocity_msg.velocity[2]);
    }
}


