#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>
#include <stdbool.h>
#include "actuator.h"
#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(MOTOR_USING_PWM_DEBUGPIN_EN)
#include "debugPin.h"
#endif

#ifdef RT_USING_FINSH

#define MOTOR_DEVICE_NAME "aux_out"
#define MAX_MOTOR_CHANNELS 4
#define DUTY_CYCLE_MIN 0.0f
#define DUTY_CYCLE_MAX 1.0f

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
    
    /* Convert duty cycle to PWM value (1000-2000 range) */
    rt_uint16_t chan_val = (rt_uint16_t)(1000.0f + duty_cycle * 1000.0f);

    /* Write PWM value to specific channel */
#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(MOTOR_USING_PWM_DEBUGPIN_EN)
    DEBUG_PIN_DEBUG0_HIGH();
#endif
    rt_device_control(motor_device, ACT_CMD_CHANNEL_ENABLE, NULL);
    rt_size_t written = rt_device_write(motor_device, chan_sel, &chan_val, sizeof(chan_val));
    if (written != sizeof(chan_val)) {
        rt_kprintf("[cmdMotor] write failed, written: %d\n", written);
        return -1;
    }

    rt_kprintf("[cmdMotor] Motor %d set to duty cycle %.2f (PWM %d)\n", motor_id, duty_cycle, chan_val);

    /* Auto stop after duration if specified */
    if (duration_ms > 0) {
        rt_kprintf("[cmdMotor] Running for %d ms then auto stop...\n", duration_ms);
        rt_thread_mdelay(duration_ms);

#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(MOTOR_USING_PWM_DEBUGPIN_EN)
    DEBUG_PIN_DEBUG0_LOW();
#endif
        /* Stop motor */
        chan_val = 0;
        written = rt_device_write(motor_device, chan_sel, &chan_val, sizeof(chan_val));
        rt_device_control(motor_device, ACT_CMD_CHANNEL_DISABLE, NULL);
        if (written == sizeof(chan_val)) {
            rt_kprintf("[cmdMotor] Motor %d auto stopped\n", motor_id);
        }
    }

    return 0;
}


/* Motor control command */
static void cmdMotor(int argc, char **argv)
{
    int motor_id = 0;
    float duty_cycle = 0.0f;
    int duration_ms = 0;

    if (argc < 4) {
        rt_kprintf("Usage: motor [motor_id] [duty_cycle] [duration_ms]\n");
        rt_kprintf("  motor_id: 1-%d (motor number)\n", MAX_MOTOR_CHANNELS);
        rt_kprintf("  duty_cycle: %.1f-%.1f (duty cycle 0.0-1.0)\n", DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
        rt_kprintf("  duration_ms: Duration (ms), 0=continuous\n");
        rt_kprintf("Examples:\n");
        rt_kprintf("  motor 1 0.5 1000     # Motor 1, 50%% duty cycle, run 1s then stop\n");
        rt_kprintf("  motor 2 0.2 0        # Motor 2, 20%% duty cycle, continuous\n");
        rt_kprintf("  motor 3 0.0 0        # Motor 3 stop\n");
        return;
    }

    /* Parse parameters */
    motor_id = atoi(argv[1]);
    duty_cycle = atof(argv[2]);
    duration_ms = atoi(argv[3]);

    /* Execute motor control */
    motor_set_duty_cycle(motor_id, duty_cycle, duration_ms);
}


#ifdef MOTOR_USING_PWM_CMD_EN
/* Export command to MSH */
MSH_CMD_EXPORT_ALIAS(cmdMotor, cmdMotor, motor control command);
#endif

#endif /* RT_USING_FINSH */
