#ifndef PID_PARAM_H
#define PID_PARAM_H

#include <stddef.h>
#include "rtdef.h"

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t getParam(const char *name, void *value, size_t value_size);

#ifdef __cplusplus
}
#endif

#endif  // PID_PARAM_H

