/*
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */
#include <rtthread.h>
#include "actuator.h"
#include "dshotControl.h"
#include "dshotConfig.h"
#include "dshotHwOpt.h"
#include <stdbool.h>
#include "dshotEncodeDecode.h"

extern dshot_config_t dshot_config_;
struct actuator_device act_dev_ = {0};

/* DShot configuration function */
static rt_err_t dshot_config(actuator_dev_t dev, const struct actuator_configure *cfg) { return RT_EOK; }

/* DShot control function */
static rt_err_t dshot_control(actuator_dev_t dev, int cmd, void *arg) {
  rt_err_t ret = RT_EOK;
  motor_stop();

  switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
      tmr_counter_enable(dshot_config_.timer_x, TRUE);
      break;
    case ACT_CMD_CHANNEL_DISABLE:
      tmr_counter_enable(dshot_config_.timer_x, FALSE);
      break;
    case ACT_CMD_SET_PROTOCOL:
      ret = RT_EINVAL;
      break;
    case ACT_CMD_TEST_ENABLE:
      dshot_config_.act_cmd_en = RT_TRUE;
      break;
    case ACT_CMD_TEST_DISABLE:
      dshot_config_.act_cmd_en = RT_FALSE;
      break;
    // case ACT_CMD_CHANGE_DIR:
    //   ret = motor_reverse(arg);
    //   break;
    default:
      ret = RT_EINVAL;
      break;
  }
  return ret;
}

/* DShot read function */
static rt_size_t dshot_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t *chan_val, rt_size_t size) {
  DEBUG_PIN_DEBUG0_HIGH();
  rt_event_recv(dshot_config_.event_dma, EVENT_DMA_SAMPLING_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER, NULL);
  DEBUG_PIN_DEBUG0_LOW();
  
  /*about 36us in at32f437vm O1*/
  DEBUG_PIN_DEBUG3_HIGH();
  decodeDShot();
  DEBUG_PIN_DEBUG3_LOW();

  // Read all motors without channel selection mask
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    chan_val[i] = dshot_config_.dshot_input_rpm[i];
  }
  return size;
}

/* DShot write function */
static rt_size_t dshot_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size) {
  if ((dshot_config_.act_cmd_en == RT_TRUE && size == 1) ||
      (dshot_config_.act_cmd_en == RT_FALSE && size == DSHOT_MOTOR_NUMS)) {
    // Process each motor individually
    for (uint8_t motor_index = 0; motor_index < DSHOT_MOTOR_NUMS; motor_index++) {
      // Check if this motor is changing direction
      if (dev->direction_changing[motor_index]) {
        dshot_handle_dir_change(motor_index);
      } else {
        // Normal motor value processing
        writeDshotValueSingleChannle(motor_index, chan_val[motor_index]);
      }
    }
  }
  setDshotValue();
  return size;
}

const static struct actuator_ops dshot_ops = {
    .act_config = dshot_config,
    .act_control = dshot_control,
    .act_read = dshot_read,
    .act_write = dshot_write,
};

/* Initialize DShot device registration */
void dshotDeviceRegInit(void) {
  act_dev_.ops = &dshot_ops;
  act_dev_.config.protocol = ACT_PROTOCOL_DSHOT;
  act_dev_.config.chan_num = DSHOT_MOTOR_NUMS;
  act_dev_.config.dshot_config.dshot_protol = ACT_PROTOCOL_DSHOT;
  act_dev_.config.dshot_config.bi_dshot = RT_TRUE;
  act_dev_.chan_mask = 0x3FF;
  act_dev_.range[0] = 0;
  act_dev_.range[1] = 2047;

  // Initialize direction_changing array
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    act_dev_.direction_changing[i] = RT_FALSE;
  }
}

/* Main DShot driver initialization */
static int dshotAutoInit(void) {
  rt_err_t ret = RT_EOK;
  dshotConfigInit();
  dshotGpioInit();
  dshotTimerInit();
  dshotDmaConfigureInit();
  dshotDeviceRegInit();
  ret = hal_actuator_register(&act_dev_, DSHOT_DEVICE_NAME, RT_DEVICE_FLAG_RDWR, NULL);
  if (ret != RT_EOK) {
    return -1;
  }
  return 0;
}

#ifdef L2_DEVICE_03_MOTOR_02_DSHOT_EN
INIT_ENV_EXPORT(dshotAutoInit);
#endif
