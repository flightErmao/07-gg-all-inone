#include "acc_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "acc_init"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
#include "timestamp.h"
#include "../common/inc/init_sync.h"
}

// 初始化相关的辅助函数可以在这里添加
// 目前主要的初始化逻辑在 acc_class.cpp 中

