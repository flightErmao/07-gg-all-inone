#include "aMcnStabilize.h"
#include "floatConvert.h"
#include "rtconfig.h"

MCN_DECLARE(state);
/* MCN topic definition */
MCN_DEFINE(state, sizeof(state_t));
/* MCN subscriber node */
static McnNode_t state_sub_node = RT_NULL;

/* Echo function for state data */
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

/* Initialize MCN state reporting */
int mcnStateReportInit(void) {
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

/* Publish state data to MCN */
int mcnStateReportPublish(const state_t* state_data) {
  if (!state_data) {
    return -1;
  }

  return mcn_publish(MCN_HUB(state), state_data);
}

/* Acquire state data from MCN */
static int mcnStateReportAcquire(state_t* state_data) {
  if (!state_data) {
    return -1;
  }

  if (state_sub_node != RT_NULL) {
    mcn_copy(MCN_HUB(state), state_sub_node, state_data);
  }

  return 0;
}

void stabilizerGetState(state_t* state) {
  if (!state) return;
  mcnStateReportAcquire(state);
}
