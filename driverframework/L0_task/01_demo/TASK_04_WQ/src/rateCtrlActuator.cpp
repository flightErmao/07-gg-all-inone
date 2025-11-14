#include "rateCtrlActuator.h"

extern "C" {
#include <rtthread.h>
#define LOG_TAG "rate_act"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

#include "workqueueManage.h"

RateCtrlActuator::RateCtrlActuator()
    : actuator_node_(RT_NULL)
{
    std::memset(&latest_cmd_, 0, sizeof(latest_cmd_));
    rt_work_init(&work_, RateCtrlActuator::workHandler, this);
}

rt_err_t RateCtrlActuator::init()
{
    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        LOG_E("workqueue init failed (%d)", ret);
        return ret;
    }

    actuator_node_ = mcn_subscribe(MCN_HUB(rate_ctrl_actuator_cmd), RT_NULL, RT_NULL);
    if (actuator_node_ == RT_NULL) {
        LOG_E("subscribe actuator topic failed");
        return -RT_ERROR;
    }

    ret = mcn_register_async_cb(actuator_node_, RateCtrlActuator::asyncCallback, this);
    if (ret != RT_EOK) {
        LOG_E("register actuator cb failed (%d)", ret);
        return ret;
    }

    LOG_I("initialized");
    return RT_EOK;
}

void RateCtrlActuator::workHandler(struct rt_work* work, void* parameter)
{
    RT_UNUSED(work);
    if (parameter == RT_NULL) {
        return;
    }

    static_cast<RateCtrlActuator*>(parameter)->handleWork();
}

void RateCtrlActuator::asyncCallback(const void* data, void* user_data)
{
    if ((data == RT_NULL) || (user_data == RT_NULL)) {
        return;
    }

    RateCtrlActuator* instance = static_cast<RateCtrlActuator*>(user_data);
    std::memcpy(&instance->latest_cmd_, data, sizeof(instance->latest_cmd_));

    if (wq_add_work(&instance->work_) != RT_EOK) {
        LOG_E("submit actuator work failed");
    }
}

void RateCtrlActuator::handleWork()
{
    applyActuatorOutputs(latest_cmd_);
}

void RateCtrlActuator::applyActuatorOutputs(const rate_ctrl_actuator_cmd_msg_t& cmd)
{
    LOG_I("apply seq:%u outputs(%.3f, %.3f, %.3f, %.3f)",
        cmd.seq,
        cmd.actuator_outputs[0],
        cmd.actuator_outputs[1],
        cmd.actuator_outputs[2],
        cmd.actuator_outputs[3]);
}


