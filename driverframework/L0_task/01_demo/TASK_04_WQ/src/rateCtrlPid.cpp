#include "rateCtrlPid.h"

extern "C" {
#include <rtthread.h>
#define LOG_TAG "rate_pid"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

#include "workqueueManage.h"

MCN_DEFINE(rate_ctrl_actuator_cmd, sizeof(rate_ctrl_actuator_cmd_msg_t));

RateCtrlPid::RateCtrlPid()
    : velocity_node_(RT_NULL)
{
    std::memset(&latest_velocity_, 0, sizeof(latest_velocity_));
    std::memset(&actuator_output_, 0, sizeof(actuator_output_));
    rt_work_init(&work_, RateCtrlPid::workHandler, this);
}

rt_err_t RateCtrlPid::init()
{
    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        LOG_E("workqueue init failed (%d)", ret);
        return ret;
    }

    ret = mcn_advertise(MCN_HUB(rate_ctrl_actuator_cmd), RT_NULL);
    if (ret != RT_EOK && ret != -RT_EBUSY) {
        LOG_E("advertise actuator topic failed (%d)", ret);
        return ret;
    }

    velocity_node_ = mcn_subscribe(MCN_HUB(vehicle_accelerate_velocity), RT_NULL, RT_NULL);
    if (velocity_node_ == RT_NULL) {
        LOG_E("subscribe velocity topic failed");
        return -RT_ERROR;
    }

    ret = mcn_register_async_cb(velocity_node_, RateCtrlPid::asyncCallback, this);
    if (ret != RT_EOK) {
        LOG_E("register velocity cb failed (%d)", ret);
        return ret;
    }

    LOG_I("initialized");
    return RT_EOK;
}

void RateCtrlPid::workHandler(struct rt_work* work, void* parameter)
{
    RT_UNUSED(work);
    if (parameter == RT_NULL) {
        return;
    }

    static_cast<RateCtrlPid*>(parameter)->handleWork();
}

void RateCtrlPid::asyncCallback(const void* data, void* user_data)
{
    if ((data == RT_NULL) || (user_data == RT_NULL)) {
        return;
    }

    RateCtrlPid* instance = static_cast<RateCtrlPid*>(user_data);
    std::memcpy(&instance->latest_velocity_, data, sizeof(instance->latest_velocity_));

    if (wq_add_work(&instance->work_) != RT_EOK) {
        LOG_E("submit pid work failed");
    }
}

void RateCtrlPid::handleWork()
{
    actuator_output_.seq = latest_velocity_.seq;

    const float kp = 0.5f;
    const float kd = 0.1f;

    for (size_t i = 0; i < 3; ++i) {
        float velocity = latest_velocity_.velocity[i];
        float command = kp * velocity + kd * (velocity - 0.0f);
        actuator_output_.actuator_outputs[i] = command;
    }

    actuator_output_.actuator_outputs[3] = actuator_output_.actuator_outputs[0]
        + actuator_output_.actuator_outputs[1]
        + actuator_output_.actuator_outputs[2];

    if (mcn_publish(MCN_HUB(rate_ctrl_actuator_cmd), &actuator_output_) != RT_EOK) {
        LOG_E("publish actuator cmd failed");
    } else {
        LOG_I("seq:%u actuator(%.3f, %.3f, %.3f, %.3f)",
            actuator_output_.seq,
            actuator_output_.actuator_outputs[0],
            actuator_output_.actuator_outputs[1],
            actuator_output_.actuator_outputs[2],
            actuator_output_.actuator_outputs[3]);
    }
}


