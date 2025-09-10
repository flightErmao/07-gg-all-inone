#include <rtdevice.h>
#include <rtthread.h>
#include "taskRc.h"
#include "rtconfig.h"
#include "rc.h"
#include "rcConvert.h"
#include "uMCN.h"
#include "floatConvert.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

static rt_device_t rc_test_dev = RT_NULL;
static rt_uint16_t rc_channels[MAX_RC_CHANNEL_NUM] = {0};

MCN_DECLARE(minifly_rc_pilot_cmd);
MCN_DEFINE(minifly_rc_pilot_cmd, sizeof(pilot_cmd_bus_t));

static McnNode_t rc_sub_node = RT_NULL;

static int rc_pilot_cmd_echo(void* parameter) {
  pilot_cmd_bus_t rc_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &rc_data) != RT_EOK) {
    return -1;
  }

  char yaw_str[16], throttle_str[16], roll_str[16], pitch_str[16];

  float_to_string(rc_data.stick_yaw, yaw_str, sizeof(yaw_str));
  float_to_string(rc_data.stick_throttle, throttle_str, sizeof(throttle_str));
  float_to_string(rc_data.stick_roll, roll_str, sizeof(roll_str));
  float_to_string(rc_data.stick_pitch, pitch_str, sizeof(pitch_str));

  rt_kprintf("RC Pilot Cmd Echo - yaw: %s, throttle: %s, roll: %s, pitch: %s, arm: %d, mode: %d, timestamp: %lu\n",
             yaw_str, throttle_str, roll_str, pitch_str, rc_data.ram_status, rc_data.ctrl_mode, rc_data.timestamp);

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

static void remapValue2PilotCmdBus(const rcValue_t rc_value_remap, const rt_uint32_t rc_timestamp,
                                   pilot_cmd_bus_t* rc_data) {
  rc_data->timestamp = rc_timestamp;
  rc_data->stick_yaw = rc_value_remap.yaw;
  rc_data->stick_throttle = rc_value_remap.thrust * 655.35;
  rc_data->stick_roll = rc_value_remap.roll;
  rc_data->stick_pitch = rc_value_remap.pitch;
  rc_data->ram_status = ARM_STATUS_ARM;
  rc_data->ctrl_mode = CTRL_MODE_ANGLE;
}

void rcTestTask(void* param) {
  rt_uint16_t channel_mask = 0xFF;

  rt_uint32_t rc_timestamp = 0;
  joystickPercent rc_joystick_percent = {0};
  flightLimit_t rc_flight_limit = {0};
  static rcValue_t rc_value_remap = {0};
  getLimitFromConfig(&rc_flight_limit);
  pilot_cmd_bus_t rc_data = {0};

  while (1) {
    if (rc_test_dev != RT_NULL) {
      rt_size_t size = rt_device_read(rc_test_dev, channel_mask, rc_channels, 16);
      if (size > 0) {
        rc_timestamp = rt_tick_get();
        normalizationRcChannels(&rc_joystick_percent, rc_channels);
        remapRcThrottlePitchRoll(rc_joystick_percent, rc_flight_limit, &rc_value_remap);
        remapValue2PilotCmdBus(rc_value_remap, rc_timestamp, &rc_data);
        mcn_publish(MCN_HUB(minifly_rc_pilot_cmd), &rc_data);
      } else {
        rt_kprintf("Read RC data failed!\n");
        rt_thread_mdelay(1000);
      }
    } else {
      rt_kprintf("RC device not found!\n");
      rt_thread_mdelay(1000);
    }
  }
}

static int taskRcTestInit(void) {
  rt_err_t ret = RT_EOK;
  char rc_name[RT_NAME_MAX];

  rt_strncpy(rc_name, PROJECT_FMT_TASK01_RC_DEVICE_DEFAULT, RT_NAME_MAX);

  rc_test_dev = rt_device_find(rc_name);
  if (!rc_test_dev) {
    rt_kprintf("Find RC device %s failed!\n", rc_name);
    return -RT_ERROR;
  }

  ret = rt_device_open(rc_test_dev, RT_DEVICE_FLAG_RDONLY);
  if (ret != RT_EOK) {
    rt_kprintf("Open RC device %s failed!\n", rc_name);
    return ret;
  }

  mcnTopicInit();

  rt_thread_t thread =
      rt_thread_create("L0_fmt_rc", rcTestTask, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (thread != RT_NULL) {
    rt_thread_startup(thread);
    rt_kprintf("RC test task started on %s\n", rc_name);
  } else {
    rt_kprintf("Create RC test thread failed!\n");
    ret = -RT_ERROR;
  }

  return ret;
}

rt_uint16_t* getRcChannels(void) { return rc_channels; }

void rcPilotCmdAcquire(pilot_cmd_bus_t* rc_data) {
  if (!rc_data) return;
#ifdef PROJECT_FMT_TASK01_RC_EN
  if (rc_sub_node != NULL) {
    mcn_copy(MCN_HUB(minifly_rc_pilot_cmd), rc_sub_node, rc_data);
  }
#endif
}

#ifdef PROJECT_FMT_TASK01_RC_EN
INIT_APP_EXPORT(taskRcTestInit);
#endif