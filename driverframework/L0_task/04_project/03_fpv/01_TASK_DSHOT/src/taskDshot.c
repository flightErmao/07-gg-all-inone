#include "rtthread.h"
#include <rtdevice.h>
#include "uMCN.h"
#include "taskDshot.h"
#include "maths.h"
#include "rtconfig.h"
#include "floatConvert.h"
#include "string.h"
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
#include "taskMiniflyStabilizer.h"
#endif

/* task definition */
#define THREAD_PRIORITY 6
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(PROJECT_MINIFLY_TASK_DSHOT_DEBUG_PIN_EN)
#include "debugPin.h"
#endif

#ifndef DSHOT_DEVICE_NAME
#define DSHOT_DEVICE_NAME "dshot"
#endif

/* mcn topic */
MCN_DEFINE(dshot, sizeof(dshot_cmd_bus_t));

static struct rt_thread task_tid_dshot;
static rt_uint8_t task_stack_dshot[THREAD_STACK_SIZE];
static McnNode_t dshot_cmd_sub = RT_NULL;
static struct rt_event dshot_event;
static rt_timer_t dshot_timer = RT_NULL;
static rt_device_t dshot_dev = RT_NULL;

/* global mapped motor values for external access */
static uint16_t dshot_mapped[4] = {0, 0, 0, 0};

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

/* publisher API for mixerControl */
void task_dshot_publish_raw(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  dshot_cmd_bus_t msg = {0};
  msg.motor_val[0] = m1;
  msg.motor_val[1] = m2;
  msg.motor_val[2] = m3;
  msg.motor_val[3] = m4;
  msg.timestamp = rt_tick_get();
  mcn_publish(MCN_HUB(dshot), &msg);
}

/* API to get mapped motor values for external access */
void task_dshot_get_mapped(uint16_t* m1, uint16_t* m2, uint16_t* m3, uint16_t* m4) {
  if (m1) *m1 = dshot_mapped[0];
  if (m2) *m2 = dshot_mapped[1];
  if (m3) *m3 = dshot_mapped[2];
  if (m4) *m4 = dshot_mapped[3];
}

static int dshot_cmd_echo(void* parameter) {
  dshot_cmd_bus_t data;

  if (mcn_copy_from_hub((McnHub*)parameter, &data) != RT_EOK) {
    return -1;
  }
  char m1[10];
  char m2[10];
  char m3[10];
  char m4[10];
  float_to_string(data.motor_val[0], m1, sizeof(m1));
  float_to_string(data.motor_val[1], m2, sizeof(m2));
  float_to_string(data.motor_val[2], m3, sizeof(m3));
  float_to_string(data.motor_val[3], m4, sizeof(m4));
  rt_kprintf("[taskDshot] motor_val: %s, %s, %s, %s\n", m1, m2, m3, m4);
  return 0;
}

static void device_init(void) {
  dshot_dev = rt_device_find(DSHOT_DEVICE_NAME);
  if (dshot_dev) {
    rt_device_open(dshot_dev, RT_DEVICE_OFLAG_RDWR);
  } else {
    rt_kprintf("[taskDshot] device %s not found\n", DSHOT_DEVICE_NAME);
  }
}

static void rtos_init(void) {
  mcn_advertise(MCN_HUB(dshot), dshot_cmd_echo);
  dshot_cmd_sub = mcn_subscribe(MCN_HUB(dshot), RT_NULL, RT_NULL);

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

static void task_dshot_entry(void* parameter) {
  device_init();
  rtos_init();

  dshot_cmd_bus_t latest = {0};
  state_t state = {0};
  bool armed = false;

  while (1) {
    rt_uint32_t rec = 0;
    rt_event_recv(&dshot_event, 1u, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);

#if defined(L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN) && defined(PROJECT_MINIFLY_TASK_DSHOT_DEBUG_PIN_EN)
    DEBUG_PIN_DEBUG0_TOGGLE();
#endif

    mcn_copy(MCN_HUB(dshot), dshot_cmd_sub, &latest);

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
    stabilizerGetState(&state);
    armed = state.armed;
#else
    armed = true;
#endif

    if (dshot_dev) {
      if (armed) {
        dshot_mapped[0] = map_motor_to_dshot(latest.motor_val[0]);
        dshot_mapped[1] = map_motor_to_dshot(latest.motor_val[1]);
        dshot_mapped[2] = map_motor_to_dshot(latest.motor_val[2]);
        dshot_mapped[3] = map_motor_to_dshot(latest.motor_val[3]);
      } else {
        memset(dshot_mapped, 0, sizeof(dshot_mapped));
      }

      /* write all 4 channels; DShot driver expects size == motor count */
      rt_device_write(dshot_dev, 0x0F, dshot_mapped, 4);
    }
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
