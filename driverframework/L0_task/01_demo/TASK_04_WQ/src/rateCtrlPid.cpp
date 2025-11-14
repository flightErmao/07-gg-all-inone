#include "rateCtrlPid.h"

extern "C" {
#include <rtthread.h>
#define LOG_TAG "rate_pid"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>
#include <math.h>
#include "../../L1_middleWare/05_px4_lib/02_mathlib/filter/LowPassFilter2p.hpp"

#include "workqueueManage.h"

// Matrix types are available through LowPassFilter2p.hpp
using namespace matrix;

MCN_DEFINE(rate_ctrl_actuator_cmd, sizeof(rate_ctrl_actuator_cmd_msg_t));

// PID gains: P, I, D for roll, pitch, yaw
#define PID_GAIN_P_ROLL  0.5f
#define PID_GAIN_P_PITCH 0.5f
#define PID_GAIN_P_YAW   0.5f
#define PID_GAIN_I_ROLL  0.1f
#define PID_GAIN_I_PITCH 0.1f
#define PID_GAIN_I_YAW   0.1f
#define PID_GAIN_D_ROLL  0.1f
#define PID_GAIN_D_PITCH 0.1f
#define PID_GAIN_D_YAW   0.1f
#define PID_INT_LIMIT    10.0f

RateCtrlPid::RateCtrlPid()
    : velocity_node_(RT_NULL),
      gain_p_(PID_GAIN_P_ROLL, PID_GAIN_P_PITCH, PID_GAIN_P_YAW),
      gain_i_(PID_GAIN_I_ROLL, PID_GAIN_I_PITCH, PID_GAIN_I_YAW),
      gain_d_(PID_GAIN_D_ROLL, PID_GAIN_D_PITCH, PID_GAIN_D_YAW),
      rate_int_(0.0f, 0.0f, 0.0f),
      lim_int_(PID_INT_LIMIT, PID_INT_LIMIT, PID_INT_LIMIT)
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

    // Convert array to Vector3f
    Vector3f velocity(latest_velocity_.velocity[0], 
                      latest_velocity_.velocity[1], 
                      latest_velocity_.velocity[2]);
    
    // For simplicity, assume setpoint is zero (rate control)
    Vector3f rate_sp(0.0f, 0.0f, 0.0f);
    Vector3f rate_error = rate_sp - velocity;
    
    // Calculate dt (assuming 3ms interval from IMU publish)
    const float dt = 0.003f;
    
    // Update integral term with anti-windup
    for (int i = 0; i < 3; i++) {
        float rate_i = rate_int_(i) + gain_i_(i) * rate_error(i) * dt;
        // Constrain integral term
        if (rate_i > lim_int_(i)) {
            rate_int_(i) = lim_int_(i);
        } else if (rate_i < -lim_int_(i)) {
            rate_int_(i) = -lim_int_(i);
        } else {
            rate_int_(i) = rate_i;
        }
    }
    
    // PID control: P + I + D
    // Note: D term would need angular acceleration, simplified here
    Vector3f torque = gain_p_.emult(rate_error) + rate_int_ - gain_d_.emult(velocity);
    
    // Convert to actuator outputs
    actuator_output_.actuator_outputs[0] = torque(0);
    actuator_output_.actuator_outputs[1] = torque(1);
    actuator_output_.actuator_outputs[2] = torque(2);
    
    // 4th output is sum (for testing)
    actuator_output_.actuator_outputs[3] = torque(0) + torque(1) + torque(2);

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


