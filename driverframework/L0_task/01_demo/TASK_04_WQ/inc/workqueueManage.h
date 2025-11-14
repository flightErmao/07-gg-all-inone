#ifndef WQ_WORKQUEUE_MANAGE_H__
#define WQ_WORKQUEUE_MANAGE_H__

#include <rtthread.h>
#include <ipc/workqueue.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t wq_workqueue_manage_init(void);
rt_err_t wq_add_work(struct rt_work* work);
struct rt_workqueue* wq_workqueue_get(void);

#ifdef __cplusplus
}
#endif

#endif /* WQ_WORKQUEUE_MANAGE_H__ */


