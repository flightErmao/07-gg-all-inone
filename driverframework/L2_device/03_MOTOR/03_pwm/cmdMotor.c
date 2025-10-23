#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>
#include <stdbool.h>
#include "actuator.h"
#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(L2_DEVICE_03_MOTOR_03_PWM_DEBUGPIN_EN)
#include "debugPin.h"
#endif

#ifdef RT_USING_FINSH

#define MOTOR_DEVICE_NAME "aux_out"
#define MAX_MOTOR_CHANNELS 4
#define DUTY_CYCLE_MIN 0.0f
#define DUTY_CYCLE_MAX 1.0f
#define MOTOR_ORDER_TEST_DURATION_MS 2000
#define MOTOR_ORDER_TEST_DUTY_CYCLE 0.1f
#define MOTOR_CALIB_DURATION_MS 10000

static rt_device_t motor_device = RT_NULL;
static bool motor_device_initialized = false;

/* Initialize motor device */
static int motor_device_init(void)
{
    if (motor_device_initialized) {
        return 0;
    }

    motor_device = rt_device_find(MOTOR_DEVICE_NAME);
    if (motor_device == RT_NULL) {
        rt_kprintf("[cmdMotor] can't find device: %s\n", MOTOR_DEVICE_NAME);
        return -1;
    }

    rt_err_t ret = rt_device_open(motor_device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("[cmdMotor] open device failed: %d\n", ret);
        return -1;
    }

    motor_device_initialized = true;
    rt_kprintf("[cmdMotor] device initialized successfully\n");
    return 0;
}

/* Set motor duty cycle */
static int motor_set_duty_cycle(int motor_id, float duty_cycle, int duration_ms)
{
    if (!motor_device_initialized) {
        if (motor_device_init() != 0) {
            return -1;
        }
    }

    if (motor_id < 1 || motor_id > MAX_MOTOR_CHANNELS) {
        rt_kprintf("[cmdMotor] Error: Motor ID must be between 1-%d\n", MAX_MOTOR_CHANNELS);
        return -1;
    }

    if (duty_cycle < DUTY_CYCLE_MIN || duty_cycle > DUTY_CYCLE_MAX) {
        rt_kprintf("[cmdMotor] Error: Duty cycle must be between %.1f-%.1f\n", DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
        return -1;
    }

    /* Convert motor_id to channel index (0-based) */
    int channel = motor_id - 1;
    rt_uint16_t chan_sel = 1 << channel;
    
    /* Convert duty cycle to 16-bit PWM value (0-65535 range) */
    rt_uint16_t chan_val = (rt_uint16_t)(duty_cycle * 65535.0f);

    /* Write PWM value to specific channel */
#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(L2_DEVICE_03_MOTOR_03_PWM_DEBUGPIN_EN)
    DEBUG_PIN_DEBUG0_HIGH();
#endif
    // rt_device_control(motor_device, ACT_CMD_CHANNEL_ENABLE, NULL);
    // uint16_t start = 0;
    // rt_device_write(motor_device, chan_sel, &start, sizeof(chan_val));
    // rt_thread_mdelay(2000);

    rt_size_t written = rt_device_write(motor_device, chan_sel, &chan_val, sizeof(chan_val));
    if (written != sizeof(chan_val)) {
        rt_kprintf("[cmdMotor] write failed, written: %d\n", written);
        return -1;
    }

    rt_kprintf("[cmdMotor] Motor %d set to duty cycle %.2f (16-bit PWM %d)\n", motor_id, duty_cycle, chan_val);

    /* Auto stop after duration if specified */
    if (duration_ms > 0) {
        rt_kprintf("[cmdMotor] Running for %d ms then auto stop...\n", duration_ms);
        rt_thread_mdelay(duration_ms);

#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(L2_DEVICE_03_MOTOR_03_PWM_DEBUGPIN_EN)
        DEBUG_PIN_DEBUG0_LOW();
#endif
        /* Stop motor (set to 0 PWM value) */
        chan_val = 0;
        written = rt_device_write(motor_device, chan_sel, &chan_val, sizeof(chan_val));
        // rt_device_control(motor_device, ACT_CMD_CHANNEL_DISABLE, NULL);
        if (written == sizeof(chan_val)) {
            rt_kprintf("[cmdMotor] Motor %d auto stopped\n", motor_id);
        }
    }

    return 0;
}

/* Test all motors sequentially */
static int motor_test_sequence(void) {
  if (!motor_device_initialized) {
    if (motor_device_init() != 0) {
      return -1;
    }
  }

  rt_kprintf("[cmdMotor] Starting motor sequence test...\n");
  rt_kprintf("[cmdMotor] Each motor will run for %d ms\n", MOTOR_ORDER_TEST_DURATION_MS);

  for (int motor_id = 1; motor_id <= MAX_MOTOR_CHANNELS; motor_id++) {
    rt_kprintf("[cmdMotor] Testing Motor %d...\n", motor_id);

    /* Set motor to 50% duty cycle for testing */
    int result = motor_set_duty_cycle(motor_id, MOTOR_ORDER_TEST_DUTY_CYCLE, MOTOR_ORDER_TEST_DURATION_MS);
    if (result != 0) {
      rt_kprintf("[cmdMotor] Error: Motor %d test failed\n", motor_id);
      return -1;
    }

    /* Small delay between motors */
    if (motor_id < MAX_MOTOR_CHANNELS) {
      rt_kprintf("[cmdMotor] Waiting 500ms before next motor...\n");
      rt_thread_mdelay(500);
    }
  }

  rt_kprintf("[cmdMotor] Motor sequence test completed!\n");
  return 0;
}

/* Motor calibration function */
static int motor_calibration(void) {
  if (!motor_device_initialized) {
    if (motor_device_init() != 0) {
      return -1;
    }
  }

  rt_kprintf("[cmdMotor] Starting motor calibration...\n");
  rt_kprintf("[cmdMotor] Step 1: Setting all motors to maximum duty cycle (%.1f%%) for %d ms\n", DUTY_CYCLE_MAX * 100,
             MOTOR_CALIB_DURATION_MS);

  /* Step 1: Set all motors to maximum duty cycle */
  for (int motor_id = 1; motor_id <= MAX_MOTOR_CHANNELS; motor_id++) {
    int result = motor_set_duty_cycle(motor_id, DUTY_CYCLE_MAX, 0);  // 0 = continuous
    if (result != 0) {
      rt_kprintf("[cmdMotor] Error: Motor %d calibration step 1 failed\n", motor_id);
      return -1;
    }
  }

  /* Wait for calibration duration */
  rt_thread_mdelay(MOTOR_CALIB_DURATION_MS);

  rt_kprintf("[cmdMotor] Step 2: Setting all motors to minimum duty cycle (%.1f%%) for %d ms\n", DUTY_CYCLE_MIN * 100,
             MOTOR_CALIB_DURATION_MS);

  /* Step 2: Set all motors to minimum duty cycle */
  for (int motor_id = 1; motor_id <= MAX_MOTOR_CHANNELS; motor_id++) {
    int result = motor_set_duty_cycle(motor_id, DUTY_CYCLE_MIN, 0);  // 0 = continuous
    if (result != 0) {
      rt_kprintf("[cmdMotor] Error: Motor %d calibration step 2 failed\n", motor_id);
      return -1;
    }
  }

  /* Wait for calibration duration */
  rt_thread_mdelay(MOTOR_CALIB_DURATION_MS);

  rt_kprintf("[cmdMotor] Motor calibration completed!\n");
  return 0;
}

/* Motor control command */
static void cmdMotorTest(int argc, char **argv) {
  int motor_id = 0;
  float duty_cycle = 0.0f;
  int duration_ms = 0;

  if (argc < 4) {
    rt_kprintf("Usage: motor [motor_id] [duty_cycle] [duration_ms]\n");
    rt_kprintf("  motor_id: 1-%d (motor number)\n", MAX_MOTOR_CHANNELS);
    rt_kprintf("  duty_cycle: %.1f-%.1f (duty cycle 0.0-1.0, converts to 16-bit PWM 0-65535)\n", DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    rt_kprintf("  duration_ms: Duration (ms), 0=continuous\n");
    rt_kprintf("Examples:\n");
    rt_kprintf("  motor 1 0.5 1000     # Motor 1, 50%% duty cycle (32767), run 1s then stop\n");
    rt_kprintf("  motor 2 0.2 0        # Motor 2, 20%% duty cycle (13107), continuous\n");
    rt_kprintf("  motor 3 0.0 0        # Motor 3 stop (0)\n");
    return;
  }

  /* Parse parameters */
  motor_id = atoi(argv[1]);
  duty_cycle = atof(argv[2]);
  duration_ms = atoi(argv[3]);

  /* Execute motor control */
  motor_set_duty_cycle(motor_id, duty_cycle, duration_ms);
}

/* Motor test command */
static void cmdMotorOrder(int argc, char **argv) {
  if (argc > 1) {
    rt_kprintf("Usage: motor_order\n");
    rt_kprintf("  Test all motors sequentially (1-4) for %d ms each\n", MOTOR_ORDER_TEST_DURATION_MS);
    rt_kprintf("  Each motor runs at %.2f%% duty cycle (16-bit PWM %d)\n", 
               MOTOR_ORDER_TEST_DUTY_CYCLE * 100, 
               (int)(MOTOR_ORDER_TEST_DUTY_CYCLE * 65535.0f));
    return;
  }

  /* Execute motor sequence test */
  motor_test_sequence();
}

/* Motor calibration command */
static void cmdMotorCalib(int argc, char **argv) {
  if (argc > 1) {
    rt_kprintf("Usage: motor_calib\n");
    rt_kprintf("  Calibrate all motors (1-4) with the following sequence:\n");
    rt_kprintf("  Step 1: Set all motors to %.1f%% duty cycle (16-bit PWM %d) for %d ms\n", 
               DUTY_CYCLE_MAX * 100, (int)(DUTY_CYCLE_MAX * 65535.0f), MOTOR_CALIB_DURATION_MS);
    rt_kprintf("  Step 2: Set all motors to %.1f%% duty cycle (16-bit PWM %d) for %d ms\n", 
               DUTY_CYCLE_MIN * 100, (int)(DUTY_CYCLE_MIN * 65535.0f), MOTOR_CALIB_DURATION_MS);
    rt_kprintf("  Total calibration time: %d ms\n", MOTOR_CALIB_DURATION_MS * 2);
    return;
  }

  /* Execute motor calibration */
  motor_calibration();
}

/* Export command to MSH */
MSH_CMD_EXPORT_ALIAS(cmdMotorTest, cmdMotorTest, motor single test command);
MSH_CMD_EXPORT_ALIAS(cmdMotorOrder, cmdMotorOrder, motor sequence test command);
MSH_CMD_EXPORT_ALIAS(cmdMotorCalib, cmdMotorCalib, motor calibration command);

#endif /* RT_USING_FINSH */
