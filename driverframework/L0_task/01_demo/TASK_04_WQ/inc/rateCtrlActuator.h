#ifndef RATE_CTRL_ACTUATOR_H__
#define RATE_CTRL_ACTUATOR_H__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>

#include "wq_topics.h"

class RateCtrlActuator {
public:
    RateCtrlActuator();

    rt_err_t init();

private:
    static void workHandler(struct rt_work* work, void* parameter);
    static void asyncCallback(const void* data, void* user_data);

    void handleWork();
    void applyActuatorOutputs(const rate_ctrl_actuator_cmd_msg_t& cmd);

    struct rt_work work_;
    McnNode_t actuator_node_;
    rate_ctrl_actuator_cmd_msg_t latest_cmd_;
};

#endif /* RATE_CTRL_ACTUATOR_H__ */


