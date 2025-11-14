#include "workqueueManage.h"

#include <rtconfig.h>
#include <ipc/workqueue.h>

#define WQ_RATE_CTRL_NAME "wq_rate_ctrl"

#ifndef CONFIG_WORK_TASK_WQ_WORKQUEUE_STACK_SIZE
#define CONFIG_WORK_TASK_WQ_WORKQUEUE_STACK_SIZE 1536
#endif

#ifndef CONFIG_WORK_TASK_WQ_WORKQUEUE_PRIORITY
#define CONFIG_WORK_TASK_WQ_WORKQUEUE_PRIORITY (RT_THREAD_PRIORITY_MAX / 3)
#endif

static struct rt_workqueue* s_rate_ctrl_wq = RT_NULL;

rt_err_t wq_workqueue_manage_init(void)
{
    if (s_rate_ctrl_wq != RT_NULL) {
        return RT_EOK;
    }

    s_rate_ctrl_wq = rt_workqueue_create(
        WQ_RATE_CTRL_NAME,
        CONFIG_WORK_TASK_WQ_WORKQUEUE_STACK_SIZE,
        CONFIG_WORK_TASK_WQ_WORKQUEUE_PRIORITY);
    if (s_rate_ctrl_wq == RT_NULL) {
        return -RT_ENOMEM;
    }

    return RT_EOK;
}

rt_err_t wq_add_work(struct rt_work* work)
{
    if (work == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_err_t ret = wq_workqueue_manage_init();
    if (ret != RT_EOK) {
        return ret;
    }

    return rt_workqueue_dowork(s_rate_ctrl_wq, work);
}

struct rt_workqueue* wq_workqueue_get(void)
{
    return s_rate_ctrl_wq;
}


