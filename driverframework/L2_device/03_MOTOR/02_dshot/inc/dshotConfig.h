/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */

#ifndef __DSHOT_CONFIG_RUNTIME_H__
#define __DSHOT_CONFIG_RUNTIME_H__

#include <rtconfig.h>
#include "drv_dma.h"
#include "at32f435_437.h"
#include "dmaConfig.h"
#include "rtconfig.h"
#include "debugPin.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EVENT_DMA_SAMPLING_DONE (1 << 0)

/* DShot frame configuration constants */
#define MOTOR_DSHOT_FRAME_BITS 16
#define MOTOR_DSHOT_STATE_PER_SYMBOL 3
#define MOTOR_DSHOT_BIT_HOLD_STATES 24
#define MOTOR_DSHOT_BIT_BEFORE_STATES 3
#define MOTOR_DSHOT_BUF_LENGTH \
  (MOTOR_DSHOT_FRAME_BITS * MOTOR_DSHOT_STATE_PER_SYMBOL + MOTOR_DSHOT_BIT_HOLD_STATES + MOTOR_DSHOT_BIT_BEFORE_STATES)
#ifndef DSHOT_RECEIVE_BUFFER_NUMBER
#define DSHOT_RECEIVE_BUFFER_NUMBER 140
#endif
#define RECEIVE_BUFFER_NUMBER DSHOT_RECEIVE_BUFFER_NUMBER

typedef struct dmaRegCache_s {
  rt_uint32_t CCR;
  rt_uint32_t CNDTR;
  rt_uint32_t CPAR;
  rt_uint32_t CMAR;
} dmaRegCache_t;

typedef struct {
  uint8_t motor_dir;
  rt_bool_t motor_change_dir_flag;
  uint8_t motor_change_dir_count;
} dshot_dir_control_t;

typedef enum {
  PWM_TYPE_DSHOT150,
  PWM_TYPE_DSHOT300,
  PWM_TYPE_DSHOT600,
} motorPwmProtocolTypes_e;

typedef struct {
  /*about gpio*/
  gpio_type *gpio;
  crm_periph_clock_type gpio_clock;
  uint8_t pin_index_arr[DSHOT_MOTOR_NUMS];

  /*about timer*/
  tmr_type *timer_x;
  crm_periph_clock_type timer_clock;
  tmr_dma_request_type tmr_dma_request;
  uint8_t timer_count_send;
  uint8_t timer_count_rec;

  /*about DMA*/
  struct dma_config *dma_cfg;
  rt_uint16_t dshot_dma_flag;
  rt_event_t event_dma;
  dmaRegCache_t dmaRegOutput;
  dmaRegCache_t dmaRegInput;
  uint8_t dshot_dma_dir;

  /*about buf*/
  uint32_t dshot_encode_cache_buf[MOTOR_DSHOT_BUF_LENGTH * DSHOT_MOTOR_NUMS];
  uint32_t dshot_dma_send_buf[MOTOR_DSHOT_BUF_LENGTH];
  uint16_t dshot_dma_rec_buf[RECEIVE_BUFFER_NUMBER];

  /*other */
  uint8_t bi_dshot_en;
  motorPwmProtocolTypes_e dshot_protocol;
  uint16_t dshot_output_value[DSHOT_MOTOR_NUMS];
  uint16_t dshot_input_rpm[DSHOT_MOTOR_NUMS];
  rt_bool_t act_cmd_en;
  dshot_dir_control_t motor_dir_ctrl[DSHOT_MOTOR_NUMS]; /* motor direction control */
  rt_bool_t ctrl_cmd_en;
} dshot_config_t;

typedef enum {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEACON1,
  DSHOT_CMD_BEACON2,
  DSHOT_CMD_BEACON3,
  DSHOT_CMD_BEACON4,
  DSHOT_CMD_BEACON5,
  DSHOT_CMD_ESC_INFO,  // V2 includes settings
  DSHOT_CMD_SPIN_DIRECTION_1,
  DSHOT_CMD_SPIN_DIRECTION_2,
  DSHOT_CMD_3D_MODE_OFF,
  DSHOT_CMD_3D_MODE_ON,
  DSHOT_CMD_SETTINGS_REQUEST,  // Currently not implemented
  DSHOT_CMD_SAVE_SETTINGS,
  DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
  DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,  // 14
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
  DSHOT_CMD_LED0_ON,                        // BLHeli32 only
  DSHOT_CMD_LED1_ON,                        // BLHeli32 only
  DSHOT_CMD_LED2_ON,                        // BLHeli32 only
  DSHOT_CMD_LED3_ON,                        // BLHeli32 only
  DSHOT_CMD_LED0_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED1_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED2_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED3_OFF,                       // BLHeli32 only
  DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,  // KISS audio Stream mode on/Off
  DSHOT_CMD_SILENT_MODE_ON_OFF = 31,        // KISS silent Mode on/Off
  DSHOT_CMD_MAX = 47
} dshotCommands_e;

/* Global runtime configuration object */
extern dshot_config_t dshot_config_;

/* Public init APIs used by dshotReg.c */
rt_err_t dshotConfigInit(void);
void dshotGpioInit(void);
void dshotTimerInit(void);
void dshotDmaConfigureInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_CONFIG_RUNTIME_H__ */