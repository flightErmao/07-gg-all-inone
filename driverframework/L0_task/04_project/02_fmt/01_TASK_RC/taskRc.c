#include <rtdevice.h>
#include <rtthread.h>
#include "taskRc.h"
#include "rtconfig.h"
#include "rc.h"
#include "rcConvert.h"
#include "uMCN.h"
#include "floatConvert.h"
#include "rcDef.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

MCN_DECLARE(minifly_rc_pilot_cmd);
MCN_DEFINE(minifly_rc_pilot_cmd, sizeof(pilot_cmd_bus_t));

static rt_device_t rc_test_dev_ = RT_NULL;
static rt_uint16_t rc_channels_[MAX_RC_CHANNEL_NUM] = {0};
static McnNode_t rc_sub_node_ = RT_NULL;
static rt_bool_t cmdPrintf = RT_FALSE;

static int rc_pilot_cmd_echo(void* parameter) {
  pilot_cmd_bus_t pilot_cmd_bus;

  if (mcn_copy_from_hub((McnHub*)parameter, &pilot_cmd_bus) != RT_EOK) {
    return -1;
  }

  char yaw_str[16], throttle_str[16], roll_str[16], pitch_str[16];

  float_to_string(pilot_cmd_bus.stick_yaw, yaw_str, sizeof(yaw_str));
  float_to_string(pilot_cmd_bus.stick_throttle, throttle_str, sizeof(throttle_str));
  float_to_string(pilot_cmd_bus.stick_roll, roll_str, sizeof(roll_str));
  float_to_string(pilot_cmd_bus.stick_pitch, pitch_str, sizeof(pitch_str));

  rt_kprintf("RC Pilot Cmd Echo - yaw: %s, throttle: %s, roll: %s, pitch: %s, arm: %d, mode: %d, timestamp: %lu\n",
             yaw_str, throttle_str, roll_str, pitch_str, pilot_cmd_bus.ram_status, pilot_cmd_bus.ctrl_mode,
             pilot_cmd_bus.timestamp);

  return 0;
}

static void remapValue2PilotCmdBus(pilot_cmd_bus_t* pilot_cmd_bus, const rcValue_t rc_value_remap,
                                   const rt_uint32_t rc_timestamp) {
  pilot_cmd_bus->timestamp = rc_timestamp;
  pilot_cmd_bus->stick_yaw = rc_value_remap.yaw;
  pilot_cmd_bus->stick_throttle = rc_value_remap.thrust * 655.35;
  pilot_cmd_bus->stick_roll = rc_value_remap.roll;
  pilot_cmd_bus->stick_pitch = rc_value_remap.pitch;
}

static void generateCmd(pilot_cmd_bus_t* pilot_cmd_bus, const rt_uint16_t* rc_channels_temp) {
  uint16_t ch_throttle = rc_channels_temp[2];
  rt_bool_t isThrottleAtMin = ch_throttle <= (RC_THROTTLE_MIN + 100);
  rt_bool_t armStick = rc_channels_temp[5] < RC_KEY_2WAY_THRESHOLD;

  if (armStick && isThrottleAtMin) {
    pilot_cmd_bus->ram_status = ARM_STATUS_ARM;
  } else if (!armStick) {
    pilot_cmd_bus->ram_status = ARM_STATUS_DISARM;
  }
  // TODO: need use joystick control crtl_mode
  pilot_cmd_bus->ctrl_mode = CTRL_MODE_ANGLE;
}

static void rcLossHandler(pilot_cmd_bus_t* pilot_cmd_bus, const rt_uint32_t rc_timestamp) {
  pilot_cmd_bus->timestamp = rc_timestamp;
  pilot_cmd_bus->ram_status = ARM_STATUS_DISARM;
  pilot_cmd_bus->ctrl_mode = CTRL_MODE_ANGLE;
  pilot_cmd_bus->stick_yaw = 0;
  pilot_cmd_bus->stick_throttle = 0;
  pilot_cmd_bus->stick_roll = 0;
  pilot_cmd_bus->stick_pitch = 0;
}

static void rcThreadEntry(void* param) {
  rt_uint16_t channel_mask = 0xFF;
  rt_uint32_t rc_timestamp = 0;
  joystickPercent rc_joystick_percent = {0};
  flightLimit_t rc_flight_limit = {0};
  static rcValue_t rc_value_remap = {0};
  getLimitFromConfig(&rc_flight_limit);
  pilot_cmd_bus_t pilot_cmd_bus = {0};
  uint16_t rc_loss_count = 0;
  rt_bool_t is_rc_loss = RT_FALSE;

  while (1) {
    if (rc_test_dev_ != RT_NULL) {
      rt_size_t size = rt_device_read(rc_test_dev_, channel_mask, rc_channels_, 16);
      rc_timestamp = rt_tick_get();
      if (size > 0) {
        rc_loss_count = 0;
        is_rc_loss = RT_FALSE;
        normalizationRcChannels(&rc_joystick_percent, rc_channels_);
        remapRcThrottlePitchRoll(rc_joystick_percent, rc_flight_limit, &rc_value_remap);
        remapValue2PilotCmdBus(&pilot_cmd_bus, rc_value_remap, rc_timestamp);
        generateCmd(&pilot_cmd_bus, rc_channels_);
        mcn_publish(MCN_HUB(minifly_rc_pilot_cmd), &pilot_cmd_bus);
      } else {
        rc_loss_count++;
        if (rc_loss_count % 10 == 0) {
          if(cmdPrintf) {
            rt_kprintf("RC loss count: %d \n", rc_loss_count);
          }
          is_rc_loss = RT_TRUE;
        }
        if (is_rc_loss) {
          rcLossHandler(&pilot_cmd_bus, rc_timestamp);
          mcn_publish(MCN_HUB(minifly_rc_pilot_cmd), &pilot_cmd_bus);
        }
      }
    } else {
      rt_kprintf("RC device not found!\n");
      rt_thread_mdelay(1000);
    }
  }
}

static rt_err_t rcDeviceInit(void) {
  rt_err_t ret = RT_EOK;
  char rc_name[RT_NAME_MAX];
  rt_strncpy(rc_name, PROJECT_FMT_TASK01_RC_DEVICE_DEFAULT, RT_NAME_MAX);

  rc_test_dev_ = rt_device_find(rc_name);
  if (!rc_test_dev_) {
    rt_kprintf("Find RC device %s failed!\n", rc_name);
    return -RT_ERROR;
  }

  ret = rt_device_open(rc_test_dev_, RT_DEVICE_FLAG_RDONLY);
  if (ret != RT_EOK) {
    rt_kprintf("Open RC device %s failed!\n", rc_name);
    return -RT_ERROR;
  }

  return ret;
}

static rt_err_t mcnTopicInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(minifly_rc_pilot_cmd), rc_pilot_cmd_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise minifly_rc_pilot_cmd topic: %d\n", result);
    return -RT_ERROR;
  }

  rc_sub_node_ = mcn_subscribe(MCN_HUB(minifly_rc_pilot_cmd), RT_NULL, RT_NULL);
  if (rc_sub_node_ == RT_NULL) {
    rt_kprintf("Failed to subscribe to minifly_rc_pilot_cmd topic\n");
    return -RT_ERROR;
  }
  return result;
}

static int taskRcThreadInit(void) {
  static struct rt_thread task_tid_rc_fmt;
  static rt_uint8_t task_stack_rc_fmt[THREAD_STACK_SIZE] = {0};

  rt_err_t ret = rt_thread_init(&task_tid_rc_fmt, "L0_fmt_rc", rcThreadEntry, RT_NULL, task_stack_rc_fmt,
                                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (ret != RT_EOK) {
    rt_kprintf("Init RC thread failed: %d\n", ret);
    return -RT_ERROR;
  }
  
  ret = rt_thread_startup(&task_tid_rc_fmt);
  if (ret != RT_EOK) {
    rt_kprintf("Start RC thread failed: %d\n", ret);
    return -RT_ERROR;
  }
  
  rt_kprintf("RC task started\n");
  return RT_EOK;
}

static int taskRcAutoStartInit(void) {
  rt_err_t ret = RT_EOK;

  ret = rcDeviceInit();
  if (ret != RT_EOK) {
    rt_kprintf("RC device init failed!\n");
    return ret;
  }

  ret = mcnTopicInit();
  if (ret != RT_EOK) {
    rt_kprintf("MCN topic init failed!\n");
    return ret;
  }

  ret = taskRcThreadInit();
  if (ret != RT_EOK) {
    rt_kprintf("RC thread auto start failed!\n");
    return ret;
  }

  return ret;
}

#ifdef PROJECT_FMT_TASK01_RC_EN
INIT_APP_EXPORT(taskRcAutoStartInit);
#endif

rt_uint16_t* getRcChannels(void) { return rc_channels_; }

void rcPilotCmdAcquire(pilot_cmd_bus_t* pilot_cmd_bus) {
  if (!pilot_cmd_bus) return;
#ifdef PROJECT_FMT_TASK01_RC_EN
  if (rc_sub_node_ != NULL) {
    mcn_copy(MCN_HUB(minifly_rc_pilot_cmd), rc_sub_node_, pilot_cmd_bus);
  }
#endif
}

static int cmdRcPri(int argc, char **argv) {

  if (argc < 2) {
    rt_kprintf("RC printf enable command usage: rc_printf_enable <enable|disable>\n");
    return -1;
  }

  if (!rt_strcmp(argv[1], "enable")) {
    cmdPrintf = RT_TRUE;
  } else if (!rt_strcmp(argv[1], "disable")) {
    cmdPrintf = RT_FALSE;
  } else {
    rt_kprintf("Invalid command: %s\n", argv[1]);
    rt_kprintf("Usage: rc_printf_enable <enable|disable>\n");
    return -1;
  }

  return 0;
}

MSH_CMD_EXPORT_ALIAS(cmdRcPri, cmdRcPri, rc printf enable command);
