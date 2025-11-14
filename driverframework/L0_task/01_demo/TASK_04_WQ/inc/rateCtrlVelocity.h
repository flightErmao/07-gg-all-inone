#ifndef RATE_CTRL_VELOCITY_H__
#define RATE_CTRL_VELOCITY_H__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>

#include "wq_topics.h"

class RateCtrlVelocity {
public:
    RateCtrlVelocity();

    rt_err_t init();

private:
    static void workHandler(struct rt_work* work, void* parameter);
    static void asyncCallback(const void* data, void* user_data);

    void handleWork();

    struct rt_work work_;
    McnNode_t imu_node_;
    imu_raw_msg_t latest_imu_;
};

#endif /* RATE_CTRL_VELOCITY_H__ */


