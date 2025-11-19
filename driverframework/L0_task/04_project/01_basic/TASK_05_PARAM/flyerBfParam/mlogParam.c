#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bfMlogParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_mlog_param_default(void *address, uint8_t size);

/* Betaflight 风格 Mlog 参数 */

/* Mlog 陀螺仪数据记录使能 */
static uint8_t mlog_gyro_en;  // 0=禁用, 1=使能

/* 默认值 */
static const uint8_t mlog_gyro_en_default = 1;  // 默认使用

static const param_default_t bf_mlog_defaults[] = {
    {&mlog_gyro_en, &mlog_gyro_en_default},
};

static void bf_mlog_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_mlog_defaults) / sizeof(bf_mlog_defaults[0])); i++) {
        if (address == bf_mlog_defaults[i].param) {
            memcpy(address, bf_mlog_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_mlog_params[] = {
    {(void*)&mlog_gyro_en, sizeof(mlog_gyro_en), "mlog_gyro_en", "u8",
     bf_mlog_param_default},
};

param_list *bfMlogParam_list(void) { return bf_mlog_params; }

size_t bfMlogParam_count(void) { return sizeof(bf_mlog_params) / sizeof(bf_mlog_params[0]); }

