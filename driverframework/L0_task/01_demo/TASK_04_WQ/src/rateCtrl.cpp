extern "C" {
#include <finsh.h>
#include <rtthread.h>

#define LOG_TAG "rate_ctrl"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include "rateCtrlActuator.h"
#include "rateCtrlPid.h"
#include "rateCtrlVelocity.h"
#include "taskImuPub.h"
#include "workqueueManage.h"

static RateCtrlVelocity g_rate_velocity;
static RateCtrlPid g_rate_pid;
static RateCtrlActuator g_rate_actuator;

static int rate_ctrl_entry(int argc, char** argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    static rt_bool_t initialized = RT_FALSE;
    if (initialized) {
        LOG_I("already initialized");
        return RT_EOK;
    }

    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        LOG_E("workqueue init failed (%d)", ret);
        return ret;
    }

    ret = task_imu_pub_start();
    if (ret != RT_EOK) {
        LOG_E("task imu pub start failed (%d)", ret);
        return ret;
    }

    ret = g_rate_velocity.init();
    if (ret != RT_EOK) {
        LOG_E("rate velocity init failed (%d)", ret);
        return ret;
    }

    ret = g_rate_pid.init();
    if (ret != RT_EOK) {
        LOG_E("rate pid init failed (%d)", ret);
        return ret;
    }

    ret = g_rate_actuator.init();
    if (ret != RT_EOK) {
        LOG_E("rate actuator init failed (%d)", ret);
        return ret;
    }

    initialized = RT_TRUE;
    LOG_I("rate control workqueue demo initialized");
    return RT_EOK;
}

MSH_CMD_EXPORT_ALIAS(rate_ctrl_entry, rate_ctrl, start workqueue rate control demo);


