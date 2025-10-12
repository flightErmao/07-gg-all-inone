#include "rtthread.h"
#include <rtdevice.h>
#include "taskDshot.h"
#include "aMcnStabilize.h"
#include "maths.h"
#include "rtconfig.h"
#include "string.h"
#include "aMlogDshot.h"
#include "motorFilter.h"

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
#include "aMcnDshot.h"
#endif

#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(PROJECT_MINIFLY_TASK_DSHOT_DEBUG_PIN_EN)
#include "debugPin.h"
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
#include "taskStabilizer.h"
#endif

#ifndef DSHOT_DEVICE_NAME
#define DSHOT_DEVICE_NAME "dshot"
#endif

/* task definition */
#define THREAD_PRIORITY 6
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

#define INVALID_RPM_MCN 2000.0f  // Invalid RPM value to indicate motor stopped

static struct rt_thread task_tid_dshot;
static rt_uint8_t task_stack_dshot[THREAD_STACK_SIZE];
static struct rt_event dshot_event;
static rt_timer_t dshot_timer = RT_NULL;
static rt_device_t dshot_dev = RT_NULL;

/* global mapped motor values for external access */
static uint16_t dshot_mapped[4] = {0, 0, 0, 0};

/* motor filter instance */
static motor_filter_t motor_filter;

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
/* motor pole pairs, default to 7 for 14-pole motors */
// TODO: should be read from config
static uint8_t motor_pole_pairs = 7;
static float dshot_rpm_pub[DSHOT_MOTOR_NUMS];
#endif

/* map motor value to DShot throttle range with armed state check */
static inline uint16_t map_motor_to_dshot(uint16_t motor_val) {
  float fv = (float)motor_val;
  float out = scaleRangef(fv, 0.0f, 65535.0f, 48.0f, 2048.0f);
  return (uint16_t)(out);
}

static void dshot_timer_cb(void* parameter) {
  RT_UNUSED(parameter);
  rt_event_send(&dshot_event, 1u);
}

/* API to get mapped motor values for external access */
void task_dshot_get_mapped(uint16_t* m1, uint16_t* m2, uint16_t* m3, uint16_t* m4) {
  if (m1) *m1 = dshot_mapped[0];
  if (m2) *m2 = dshot_mapped[1];
  if (m3) *m3 = dshot_mapped[2];
  if (m4) *m4 = dshot_mapped[3];
}

static void device_init(void) {
  dshot_dev = rt_device_find(DSHOT_DEVICE_NAME);
  if (dshot_dev) {
    rt_device_open(dshot_dev, RT_DEVICE_OFLAG_RDWR);
  } else {
    rt_kprintf("[taskDshot] device %s not found\n", DSHOT_DEVICE_NAME);
  }

  /* Initialize mlog DShot functionality */
  mlogDshotInit();

  /* Initialize motor filter */
  motor_filter_init(&motor_filter);
  rt_kprintf("[taskDshot] motor filter initialized\n");
}

static void rtos_init(void) {
  rt_event_init(&dshot_event, "dshot_evt", RT_IPC_FLAG_PRIO);

  /* timer period from Kconfig: PROJECT_MINIFLY_TASK_DSHOT_TIMER_HZ */
#ifdef PROJECT_MINIFLY_TASK_DSHOT_TIMER_HZ
  int freq = PROJECT_MINIFLY_TASK_DSHOT_TIMER_HZ;
#else
  int freq = 500;
#endif
  if (freq <= 0) freq = 500;
  rt_tick_t period_ticks = rt_tick_from_millisecond(1000 / freq);
  if (period_ticks == 0) period_ticks = 1;
  dshot_timer = rt_timer_create("dshot_tmr", dshot_timer_cb, RT_NULL, period_ticks,
                                RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
  if (dshot_timer) {
    rt_timer_start(dshot_timer);
  } else {
    rt_kprintf("[taskDshot] create timer failed\n");
  }
}

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
static void dshotReadRpm(void) {
  uint16_t rpm_read_temp[DSHOT_MOTOR_NUMS];
  uint16_t dshot_erpm_raw[DSHOT_MOTOR_NUMS];
  // Read RPM data from all physical motors
  rt_device_read(dshot_dev, 0, rpm_read_temp, 0);

  // Apply reverse mapping to get logical motor RPM values
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    dshot_erpm_raw[i] = rpm_read_temp[i];
    if (dshot_erpm_raw[i] == 0) {
      // TODO:we need safe guard to lock to disarm when in the air and bidir-dshot link invalid
      dshot_rpm_pub[i] = INVALID_RPM_MCN;
    } else {
      dshot_rpm_pub[i] = (float)((float)(1000000 / motor_pole_pairs) / (dshot_erpm_raw[i] * 1.0f));
    }
  }
}
#endif

static void task_dshot_entry(void* parameter) {
  device_init();
  rtos_init();

  dshot_cmd_bus_t latest = {0};
  dshot_cmd_bus_t filtered = {0};
  state_t state = {0};
  bool armed = false;

  while (1) {
    rt_uint32_t rec = 0;
    rt_event_recv(&dshot_event, 1u, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);

#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(PROJECT_MINIFLY_TASK_DSHOT_DEBUG_PIN_EN)
    DEBUG_PIN_DEBUG0_TOGGLE();
#endif

    /* Acquire DShot command from MCN */
    if (mcnDshotCmdAcquire(&latest) != RT_EOK) {
      // No new data available, skip this iteration
      continue;
    }

    /* Apply motor filter to reduce noise */
    motor_filter_apply(&motor_filter, latest.motor_val, filtered.motor_val);
    filtered.timestamp = latest.timestamp;

    /* Optional: Print filter debug info (can be disabled in production) */
#ifdef PROJECT_MINIFLY_TASK_DSHOT_FILTER_DEBUG_EN
    static int debug_counter = 0;
    if (++debug_counter >= 100) { /* Print every 100 iterations */
      debug_counter = 0;
      rt_kprintf("[taskDshot] Raw: %d,%d,%d,%d -> Filtered: %d,%d,%d,%d\n", latest.motor_val[0], latest.motor_val[1],
                 latest.motor_val[2], latest.motor_val[3], filtered.motor_val[0], filtered.motor_val[1],
                 filtered.motor_val[2], filtered.motor_val[3]);
    }
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
    stabilizerGetState(&state);
    armed = state.armed;
#else
    armed = true;
#endif

    if (dshot_dev) {
      if (armed) {
        /* Use filtered motor values instead of raw values */
        dshot_mapped[0] = map_motor_to_dshot(filtered.motor_val[0]);
        dshot_mapped[1] = map_motor_to_dshot(filtered.motor_val[1]);
        dshot_mapped[2] = map_motor_to_dshot(filtered.motor_val[2]);
        dshot_mapped[3] = map_motor_to_dshot(filtered.motor_val[3]);
      } else {
        memset(dshot_mapped, 0, sizeof(dshot_mapped));
      }

      /* write all 4 channels; DShot driver expects size == motor count */
      rt_device_write(dshot_dev, 0x0F, dshot_mapped, 4);

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
      /* Read RPM and publish to MCN */
      dshotReadRpm();
      rpm_data_bus_t rpm_msg;
      memcpy(rpm_msg.rpm, dshot_rpm_pub, sizeof(dshot_rpm_pub));
      rpm_msg.timestamp = rt_tick_get();
      mcnRpmDataPublish(&rpm_msg);
#endif
    }

    /* Push data to mlog */
    mlogDshotPush(dshot_mapped, rt_tick_get());
  }
}

static int task_dshot_autostart(void) {
  rt_thread_init(&task_tid_dshot, "dshot", task_dshot_entry, RT_NULL, task_stack_dshot, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_dshot);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_DSHOT_EN
INIT_APP_EXPORT(task_dshot_autostart);
#endif