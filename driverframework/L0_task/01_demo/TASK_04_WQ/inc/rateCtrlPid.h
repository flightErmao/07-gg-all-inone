#ifndef RATE_CTRL_PID_H__
#define RATE_CTRL_PID_H__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>

#include "wq_topics.h"

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
};

#endif /* RATE_CTRL_PID_H__ */


