#include "aMcnStabilize.h"
#include "floatConvert.h"
#include "rtconfig.h"

/* ==================== State MCN ==================== */
MCN_DECLARE(state);
MCN_DEFINE(state, sizeof(state_t));
static McnNode_t state_sub_node = RT_NULL;

static int state_echo(void* parameter) {
  state_t state;
  if (mcn_copy_from_hub((McnHub*)parameter, &state) != RT_EOK) {
    return -1;
  }
  char roll[16], pitch[16], yaw[16];
  float_to_string(state.attitude.roll, roll, sizeof(roll));
  float_to_string(state.attitude.pitch, pitch, sizeof(pitch));
  float_to_string(state.attitude.yaw, yaw, sizeof(yaw));
  rt_kprintf("att(r,p,y): %s, %s, %s, mode: %d, armed: %d, ts: %lu\n", roll, pitch, yaw, (int)state.fly_mode,
             (int)state.armed, state.attitude.timestamp);
  return 0;
}

int mcnStateInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(state), state_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise state topic: %d\n", result);
    return -1;
  }

  state_sub_node = mcn_subscribe(MCN_HUB(state), RT_NULL, RT_NULL);
  if (state_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to state topic\n");
    return -1;
  }

  return 0;
}

int mcnStatePub(const state_t* state_data) {
  if (!state_data) {
    return -1;
  }

  return mcn_publish(MCN_HUB(state), state_data);
}

void mcnStateAcquire(state_t* state_data) {
  if (!state_data) {
    return;
  }

  if (state_sub_node != RT_NULL) {
    mcn_copy(MCN_HUB(state), state_sub_node, state_data);
  }
}
INIT_COMPONENT_EXPORT(mcnStateInit);

/* ==================== Mixer MCN ==================== */

MCN_DECLARE(mixer_cmd);
MCN_DEFINE(mixer, sizeof(mixer_data_t));
static McnNode_t mixer_cmd_sub_node = RT_NULL;

static int mixer_cmd_echo(void* parameter) {
  mixer_data_t data;
  if (mcn_copy_from_hub((McnHub*)parameter, &data) != RT_EOK) {
    return -1;
  }
  char m1[10], m2[10], m3[10], m4[10];
  float_to_string(data.motor_val[0], m1, sizeof(m1));
  float_to_string(data.motor_val[1], m2, sizeof(m2));
  float_to_string(data.motor_val[2], m3, sizeof(m3));
  float_to_string(data.motor_val[3], m4, sizeof(m4));
  rt_kprintf("[aMcnStabilize] mixer: %s, %s, %s, %s, ts: %lu\n", m1, m2, m3, m4, data.timestamp);
  return 0;
}

static int mcnMixerInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(mixer), mixer_cmd_echo);
  if (result != RT_EOK) {
    rt_kprintf("[aMcnStabilize] Failed to advertise mixer topic: %d\n", result);
    return -1;
  }
  static rt_sem_t event = RT_NULL;
  event = rt_sem_create("mixer_node_event", 0, RT_IPC_FLAG_FIFO);
  mixer_cmd_sub_node = mcn_subscribe(MCN_HUB(mixer), event, RT_NULL);
  if (mixer_cmd_sub_node == RT_NULL) {
    rt_kprintf("[aMcnStabilize] Failed to subscribe to mixer topic\n");
    return -1;
  }

  rt_kprintf("[aMcnStabilize] DShot command MCN initialized\n");
  return 0;
}

int mcnMixerPublish(const mixer_data_t* mixer_data) {
  if (!mixer_data) {
    return -1;
  }
  return mcn_publish(MCN_HUB(mixer), mixer_data);
}

int mcnMixerAcquire(mixer_data_t* mixer_data) {
  if (!mixer_data) {
    return -1;
  }
  if (mixer_cmd_sub_node == RT_NULL) {
    return -1;
  }
  return mcn_copy(MCN_HUB(mixer), mixer_cmd_sub_node, mixer_data);
}

void mcnWaitMixerPub() {
  if (mixer_cmd_sub_node != RT_NULL) {
    mcn_poll_sync(mixer_cmd_sub_node, RT_WAITING_FOREVER);
  }
}

INIT_COMPONENT_EXPORT(mcnMixerInit);
