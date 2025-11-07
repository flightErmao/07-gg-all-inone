#ifndef PARAM_FYDELIX_H
#define PARAM_FYDELIX_H

#include <stddef.h>

#include "rtdef.h"
#include "uparam.h"

rt_err_t getParam(const char *name, void *value, size_t value_size);
rt_err_t setParam(const char *name, void *value, size_t value_size);

// 新增接口函数，用于获取参数信息（避免使用extern）
size_t getParamCount(void);
const param_list *getParamByIndex(size_t index);

#endif  // PARAM_FYDELIX_H