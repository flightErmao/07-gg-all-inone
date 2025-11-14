#include "rateCtrlVelocity.h"

extern "C" {
#include <rtthread.h>
#define LOG_TAG "rate_vel"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

#include "workqueueManage.h"

MCN_DEFINE(vehicle_accelerate_velocity, sizeof(vehicle_accelerate_velocity_msg_t));

RateCtrlVelocity::RateCtrlVelocity()
    : imu_node_(RT_NULL)
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

    for (size_t i = 0; i < 3; ++i) {
        float accel = latest_imu_.accel[i];
        float gyro = latest_imu_.gyro[i];
        velocity_msg.velocity[i] = 0.8f * accel + 0.2f * gyro;
    }

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


