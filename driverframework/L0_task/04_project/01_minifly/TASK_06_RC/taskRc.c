#include "rtthread.h"
#include <rtdevice.h>
#include "uMCN.h"
#include "taskRc.h"
#include "anlRemote.h"

/*task definition*/
#define THREAD_PRIORITY 6
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

#define TIMER_PERIOD 20

// Failsafe timeout configuration
#ifdef PROJECT_MINIFLY_TASK06_RC_FAILSAFE_TIMEOUT_MS
#define RC_FAILSAFE_TIMEOUT_MS PROJECT_MINIFLY_TASK06_RC_FAILSAFE_TIMEOUT_MS
#else
#define RC_FAILSAFE_TIMEOUT_MS 2000  // Default 2 seconds
#endif

static struct rt_thread task_tid_rc_minifly;
static rt_uint8_t task_stack_rc_minifly[THREAD_STACK_SIZE];

static rt_timer_t rc_timer = RT_NULL;
static rt_event_t rc_event = RT_NULL;
#define RC_EVENT_TIMER (1 << 0)

MCN_DECLARE(minifly_rc_pilot_cmd);
MCN_DEFINE(minifly_rc_pilot_cmd, sizeof(pilot_cmd_bus_t));

static McnNode_t rc_sub_node = RT_NULL;

static void rc_timer_callback(void *parameter) { rt_event_send(rc_event, RC_EVENT_TIMER); }

static int rc_pilot_cmd_echo(void *parameter) {
  pilot_cmd_bus_t rc_data;

  if (mcn_copy_from_hub((McnHub *)parameter, &rc_data) != RT_EOK) {
    return -1;
  }

  rt_kprintf(
      "RC Pilot Cmd Echo - yaw: %.2f, throttle: %.2f, roll: %.2f, pitch: %.2f, arm: %d, mode: %d, timestamp: %lu\n",
      rc_data.stick_yaw, rc_data.stick_throttle, rc_data.stick_roll, rc_data.stick_pitch, rc_data.ram_status,
      rc_data.ctrl_mode, rc_data.timestamp);

  return 0;
}

static void mcnTopicInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(minifly_rc_pilot_cmd), rc_pilot_cmd_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise minifly_rc_pilot_cmd topic: %d\n", result);
  }

  rc_sub_node = mcn_subscribe(MCN_HUB(minifly_rc_pilot_cmd), RT_NULL, RT_NULL);
  if (rc_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to minifly_rc_pilot_cmd topic\n");
  }
}

static void rcTimerInit(void) {
  rc_event = rt_event_create("rc_event", RT_IPC_FLAG_FIFO);
  if (rc_event == RT_NULL) {
    rt_kprintf("Failed to create rc_event\n");
    return;
  }

  rc_timer = rt_timer_create("rc_timer", rc_timer_callback, RT_NULL, rt_tick_from_millisecond(TIMER_PERIOD),
                             RT_TIMER_FLAG_PERIODIC);
  if (rc_timer == RT_NULL) {
    rt_kprintf("Failed to create rc_timer\n");
    return;
  }

  if (rt_timer_start(rc_timer) != RT_EOK) {
    rt_kprintf("Failed to start rc_timer\n");
    return;
  }

  rt_kprintf("RC timer initialized successfully, period: %dms\n", TIMER_PERIOD);
}

/**
 * @brief Failsafe check function
 * @param current_timestamp Current timestamp
 * @param rc_timestamp RC data timestamp
 * @return true: Failsafe triggered, false: Normal
 */
static bool rc_failsafe_check(uint32_t current_timestamp, uint32_t rc_timestamp) {
  uint32_t time_diff = current_timestamp - rc_timestamp;

  // Check if time difference exceeds failsafe timeout
  if (time_diff > rt_tick_from_millisecond(RC_FAILSAFE_TIMEOUT_MS)) {
    return true;  // Trigger failsafe
  }

  return false;  // Normal
}

/**
 * @brief Apply failsafe measures
 * @param rc_data RC data pointer
 */
static void rc_apply_failsafe(pilot_cmd_bus_t *rc_data) {
  if (!rc_data) return;

  // Reset all axis data to zero
  rc_data->stick_yaw = 0.0f;
  rc_data->stick_throttle = 0.0f;
  rc_data->stick_roll = 0.0f;
  rc_data->stick_pitch = 0.0f;

  // Set to disarm status
  rc_data->ram_status = ARM_STATUS_DISARM;

  // Keep control mode unchanged
  // rc_data->ctrl_mode remains unchanged
}

static void rc_minifly_thread_entry(void *parameter) {
  mcnTopicInit();
  rcTimerInit();

  pilot_cmd_bus_t rc_data = {0};
  rt_uint32_t recv_event = 0;

  rc_data.stick_yaw = 0.0f;
  rc_data.stick_throttle = 0.0f;
  rc_data.stick_roll = 0.0f;
  rc_data.stick_pitch = 0.0f;
  rc_data.ram_status = ARM_STATUS_DISARM;
  rc_data.ctrl_mode = CTRL_MODE_ANGLE;

  while (1) {
    if (rt_event_recv(rc_event, RC_EVENT_TIMER, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
                      &recv_event) == RT_EOK) {
      uint32_t timestamp = rt_tick_get();
      rc_data.timestamp = timestamp;
      rcRawData_t rcRawData = getRcRawData();

      // Check failsafe
      if (rc_failsafe_check(timestamp, rcRawData.timestamp)) {
        // Trigger failsafe
        rc_apply_failsafe(&rc_data);

#ifdef PROJECT_MINIFLY_TASK06_RC_DEBUGPIN_EN
        rt_kprintf("RC failsafe triggered! Time diff: %lu ms\n",
                   (timestamp - rcRawData.timestamp) * (1000 / RT_TICK_PER_SECOND));
#endif
      } else {
        // Normal case, update RC data
        rc_data.stick_yaw = rcRawData.roll;
        rc_data.stick_throttle = rcRawData.thrust;
        rc_data.stick_roll = rcRawData.pitch;
        rc_data.stick_pitch = rcRawData.yaw;
        rc_data.ram_status = rcRawData.arm_status ? ARM_STATUS_ARM : ARM_STATUS_DISARM;
      }

      mcn_publish(MCN_HUB(minifly_rc_pilot_cmd), &rc_data);
    }
  }
}

void rcPilotCmdAcquire(pilot_cmd_bus_t *rc_data) {
  if (!rc_data) return;
  // if (mcn_poll(rc_sub_node)) {
  mcn_copy(MCN_HUB(minifly_rc_pilot_cmd), rc_sub_node, rc_data);
  // }
}

static int taskRcThreadAutoStart(void) {
  rt_thread_init(&task_tid_rc_minifly, "L0_minifly_rc", rc_minifly_thread_entry, RT_NULL, task_stack_rc_minifly,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_rc_minifly);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK06_RC_EN
INIT_APP_EXPORT(taskRcThreadAutoStart);
#endif