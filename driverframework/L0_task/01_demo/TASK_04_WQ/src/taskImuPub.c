#include "taskImuPub.h"

#include <rtconfig.h>
#include <rtthread.h>

#define LOG_TAG "task_imu_pub"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#include <uMCN.h>

#include "workqueueManage.h"

#ifndef CONFIG_WORK_TASK_WQ_IMU_THREAD_STACK_SIZE
#define CONFIG_WORK_TASK_WQ_IMU_THREAD_STACK_SIZE 1024
#endif

#ifndef CONFIG_WORK_TASK_WQ_IMU_THREAD_PRIORITY
#define CONFIG_WORK_TASK_WQ_IMU_THREAD_PRIORITY (RT_THREAD_PRIORITY_MAX / 2)
#endif

#ifndef CONFIG_WORK_TASK_WQ_IMU_THREAD_TIMESLICE
#define CONFIG_WORK_TASK_WQ_IMU_THREAD_TIMESLICE 20
#endif

#ifndef CONFIG_WORK_TASK_WQ_IMU_PUBLISH_INTERVAL_MS
#define CONFIG_WORK_TASK_WQ_IMU_PUBLISH_INTERVAL_MS 3000
#endif

MCN_DEFINE(imu_raw, sizeof(imu_raw_msg_t));

static rt_thread_t s_task_thread = RT_NULL;
static imu_raw_msg_t s_imu_msg = { 0 };

static void task_imu_pub_entry(void* parameter)
{
    RT_UNUSED(parameter);

    while (1) {
        ++s_imu_msg.seq;

        s_imu_msg.accel[0] = 0.01f * s_imu_msg.seq;
        s_imu_msg.accel[1] = 0.02f * s_imu_msg.seq;
        s_imu_msg.accel[2] = 0.03f * s_imu_msg.seq;

        s_imu_msg.gyro[0] = 0.001f * s_imu_msg.seq;
        s_imu_msg.gyro[1] = 0.002f * s_imu_msg.seq;
        s_imu_msg.gyro[2] = 0.003f * s_imu_msg.seq;

        if (mcn_publish(MCN_HUB(imu_raw), &s_imu_msg) != RT_EOK) {
            LOG_E("publish imu raw failed");
        } else {
            LOG_I("publish imu seq:%u accel(%.3f, %.3f, %.3f) gyro(%.3f, %.3f, %.3f)",
                s_imu_msg.seq,
                s_imu_msg.accel[0],
                s_imu_msg.accel[1],
                s_imu_msg.accel[2],
                s_imu_msg.gyro[0],
                s_imu_msg.gyro[1],
                s_imu_msg.gyro[2]);
        }

        rt_thread_mdelay(CONFIG_WORK_TASK_WQ_IMU_PUBLISH_INTERVAL_MS);
    }
}

rt_err_t task_imu_pub_start(void)
{
    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        LOG_E("workqueue init failed (%d)", ret);
        return ret;
    }

    ret = mcn_advertise(MCN_HUB(imu_raw), RT_NULL);
    if (ret != RT_EOK && ret != -RT_EBUSY) {
        LOG_E("imu raw advertise failed (%d)", ret);
        return ret;
    }

    if (s_task_thread != RT_NULL) {
        LOG_I("imu publisher already running");
        return RT_EOK;
    }

    s_task_thread = rt_thread_create("imu_pub",
        task_imu_pub_entry,
        RT_NULL,
        CONFIG_WORK_TASK_WQ_IMU_THREAD_STACK_SIZE,
        CONFIG_WORK_TASK_WQ_IMU_THREAD_PRIORITY,
        CONFIG_WORK_TASK_WQ_IMU_THREAD_TIMESLICE);
    if (s_task_thread == RT_NULL) {
        LOG_E("create imu pub thread failed");
        return -RT_ENOMEM;
    }

    rt_thread_startup(s_task_thread);
    LOG_I("imu publisher started");

    return RT_EOK;
}


