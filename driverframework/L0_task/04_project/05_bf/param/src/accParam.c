#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <rtconfig.h>  // For PROJECT_BF_ACC_EN

#include "bfAccParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_acc_param_default(void *address, uint8_t size);

/* 加速度计滤波器参数 */
#ifdef PROJECT_BF_ACC_EN
static uint16_t acc_filter_cutoff_hz;  // 加速度计 PT2 滤波器截止频率（Hz）
#endif

/* 默认值 - 与 Betaflight 一致 */
#ifdef PROJECT_BF_ACC_EN
static const uint16_t acc_filter_cutoff_hz_default = 50;  // 默认 50Hz
#endif

#ifdef PROJECT_BF_ACC_EN
static const param_default_t bf_acc_defaults[] = {
    {&acc_filter_cutoff_hz, &acc_filter_cutoff_hz_default},
};

static void bf_acc_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_acc_defaults) / sizeof(bf_acc_defaults[0])); i++) {
        if (address == bf_acc_defaults[i].param) {
            memcpy(address, bf_acc_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_acc_params[] = {
    {(void*)&acc_filter_cutoff_hz, sizeof(acc_filter_cutoff_hz), "acc_filter_cutoff_hz", "u16", bf_acc_param_default},
};

param_list *bfAccParam_list(void) { return bf_acc_params; }

size_t bfAccParam_count(void) { return sizeof(bf_acc_params) / sizeof(bf_acc_params[0]); }
#else
param_list *bfAccParam_list(void) { return RT_NULL; }

size_t bfAccParam_count(void) { return 0; }
#endif

