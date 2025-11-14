#ifndef RATE_CTRL_VELOCITY_H__
#define RATE_CTRL_VELOCITY_H__

#include <rtthread.h>
#include <uMCN.h>
#include <ipc/workqueue.h>
#include <matrix/math.hpp>
#include "../../L1_middleWare/05_px4_lib/02_mathlib/filter/LowPassFilter2p.hpp"

#include "wq_topics.h"

using namespace matrix;

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
    
    // Low pass filters for velocity calculation
    math::LowPassFilter2p<Vector3f> accel_filter_;
    math::LowPassFilter2p<Vector3f> gyro_filter_;
};

#endif /* RATE_CTRL_VELOCITY_H__ */


