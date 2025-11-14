#ifndef RATE_CTRL_PID_H__
#define RATE_CTRL_PID_H__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>
#include <matrix/math.hpp>

#include "wq_topics.h"

using namespace matrix;

class RateCtrlPid {
public:
    RateCtrlPid();

    rt_err_t init();

private:
    static void workHandler(struct rt_work* work, void* parameter);
    static void asyncCallback(const void* data, void* user_data);

    void handleWork();

    struct rt_work work_;
    McnNode_t velocity_node_;
    vehicle_accelerate_velocity_msg_t latest_velocity_;
    rate_ctrl_actuator_cmd_msg_t actuator_output_;
    
    // PID gains
    Vector3f gain_p_;
    Vector3f gain_i_;
    Vector3f gain_d_;
    Vector3f rate_int_;
    Vector3f lim_int_;
};

#endif /* RATE_CTRL_PID_H__ */


