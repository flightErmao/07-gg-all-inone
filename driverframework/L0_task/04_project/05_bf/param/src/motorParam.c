#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "bfMotorParam.h"

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void bf_motor_param_default(void *address, uint8_t size);

/* Betaflight 风格电机混控参数 */

/* Mixer Mode - 混控模式 (1-27, 3=MIXER_QUADX) */
static uint8_t mixer_mode;

/* Mixer Type - 混控器类型 (0=LEGACY, 1=LINEAR, 2=DYNAMIC, 3=EZLANDING) */
static uint8_t mixer_type;

/* 默认值 - Betaflight 典型配置 */
static const uint8_t mixer_mode_default = 3;  // MIXER_QUADX
static const uint8_t mixer_type_default = 0;  // MIXER_LEGACY

static const param_default_t bf_motor_defaults[] = {
    {&mixer_mode, &mixer_mode_default},
    {&mixer_type, &mixer_type_default},
};

static void bf_motor_param_default(void *address, uint8_t size) {
    for (size_t i = 0; i < (sizeof(bf_motor_defaults) / sizeof(bf_motor_defaults[0])); i++) {
        if (address == bf_motor_defaults[i].param) {
            memcpy(address, bf_motor_defaults[i].default_value, size);
            break;
        }
    }
}

static param_list bf_motor_params[] = {
    {(void *)&mixer_mode, sizeof(mixer_mode), "mixer_mode", "u8", bf_motor_param_default},
    {(void *)&mixer_type, sizeof(mixer_type), "mixer_type", "u8", bf_motor_param_default},
};

param_list *bfMotorParam_list(void) { return bf_motor_params; }

size_t bfMotorParam_count(void) { return sizeof(bf_motor_params) / sizeof(bf_motor_params[0]); }

