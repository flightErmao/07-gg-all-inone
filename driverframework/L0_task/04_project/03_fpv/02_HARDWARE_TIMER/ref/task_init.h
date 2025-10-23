#ifndef TASK_INIT_H__
#define TASK_INIT_H__

#include <rtthread.h>


#ifdef __cplusplus
extern "C" {
#endif
// #define TIME_PIN_DEBUG_EN


rt_err_t InitCommon(void);
rt_err_t take_sem(const char *name, rt_int32_t timeout);
rt_err_t release_sem(const char *name);
rt_err_t ssend_event(const char *pubname, const char *eventname);
rt_err_t recv_event(const char *pubname, const char *subname, rt_uint32_t opt, rt_int32_t timeout);

#ifdef __cplusplus
}
#endif

#endif  // TASK_COMMON_H__
