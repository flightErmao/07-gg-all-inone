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

#ifdef DSHOT_CMD_MOTOR_EN

#include <rtthread.h>
#include "actuator.h"
#include <stdlib.h>
/* update delay for telemetry readback */
#define MOTOR_CMD_UPDATE_DELAY_MS 50

/* Helper: test mode control via actuator device */
static rt_device_t find_dshot_device(void) { return rt_device_find(DSHOT_DEVICE_NAME); }

static void dshot_test_mode_enable(void) {
  rt_device_t dev = find_dshot_device();
  if (dev) {
    rt_device_control(dev, ACT_CMD_TEST_ENABLE, NULL);
  }
}

static void dshot_test_mode_disable(void) {
  rt_device_t dev = find_dshot_device();
  if (dev) {
    rt_device_control(dev, ACT_CMD_TEST_DISABLE, NULL);
  }
}

/* Helper: override all motor values in test mode (per-channel write) */
static void setMotorTestOverride(const uint16_t *values) {
  rt_device_t dev = find_dshot_device();
  if (!dev) {
    rt_kprintf("[cmdDshot] DShot device not found\n");
    return;
  }
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    rt_uint16_t mask = (rt_uint16_t)(1u << i);
    rt_uint16_t val = values[i];
    rt_size_t written = rt_device_write(dev, mask, &val, 1);
    if (written != 1) {
      rt_kprintf("[cmdDshot] write motor %d failed, ret=%d\n", i + 1, written);
    }
  }
}

/* Helper: convert 0.0-1.0 duty to DShot value 0-2047 */
static uint16_t duty_to_dshot_value(float duty) {
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  return (uint16_t)(duty * 2047.0f + 0.5f);
}

/* Single motor test: motor_id[1-4], duty[0.0-1.0], duration_ms */
static void cmdMotorTest(int argc, char **argv) {
  if (argc < 4) {
    rt_kprintf("Usage: cmdMotorTest [motor_id 1-4] [duty 0.0-1.0] [duration_ms 0=continuous]\n");
    return;
  }

  int motor_id = atoi(argv[1]);
  float duty = atof(argv[2]);
  int duration_ms = atoi(argv[3]);

  if (motor_id < 1 || motor_id > DSHOT_MOTOR_NUMS) {
    rt_kprintf("[cmdDshot] Error: motor_id must be 1-%d\n", DSHOT_MOTOR_NUMS);
    return;
  }

  uint16_t values[DSHOT_MOTOR_NUMS] = {0};
  values[motor_id - 1] = duty_to_dshot_value(duty);

  dshot_test_mode_enable();
  setMotorTestOverride(values);
  rt_kprintf("[cmdDshot] Motor %d set duty %.2f (val=%d)\n", motor_id, duty, values[motor_id - 1]);

  if (duration_ms > 0) {
    rt_kprintf("[cmdDshot] running %d ms then stop...\n", duration_ms);
    rt_thread_mdelay(duration_ms);
    values[motor_id - 1] = 0;
    setMotorTestOverride(values);
    rt_kprintf("[cmdDshot] Motor %d stopped\n", motor_id);
  }

  dshot_test_mode_disable();
}

/* Motors order test: run each motor sequentially */
static void cmdMortorOrder(int argc, char **argv) {
  int duration_ms = 2000;
  float duty = 0.10f;
  if (argc == 3) {
    duty = atof(argv[1]);
    duration_ms = atoi(argv[2]);
  } else if (argc != 1) {
    rt_kprintf("Usage: cmdMortorOrder [duty 0.0-1.0] [duration_ms]\n");
    return;
  }

  dshot_test_mode_enable();
  rt_kprintf("[cmdDshot] sequence test: duty=%.2f duration=%d ms\n", duty, duration_ms);

  for (int m = 0; m < DSHOT_MOTOR_NUMS; m++) {
    uint16_t values[DSHOT_MOTOR_NUMS] = {0};
    values[m] = duty_to_dshot_value(duty);
    rt_kprintf("[cmdDshot] testing motor %d...\n", m + 1);
    setMotorTestOverride(values);
    rt_thread_mdelay(duration_ms);
    values[m] = 0;
    setMotorTestOverride(values);
    if (m < DSHOT_MOTOR_NUMS - 1) {
      rt_thread_mdelay(500);
    }
  }

  rt_kprintf("[cmdDshot] sequence test done.\n");
  dshot_test_mode_disable();
}

MSH_CMD_EXPORT_ALIAS(cmdMotorTest, cmdMotorTest, dshot single motor test command);
MSH_CMD_EXPORT_ALIAS(cmdMortorOrder, cmdMortorOrder, dshot motor sequence test command);

#endif  // CONFIG_DSHOT_CMD_MOTOR_EN
