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

/* ==================== DShot Command MCN ==================== */

#ifdef PROJECT_MINIFLY_TASK_DSHOT_EN

/* MCN topic definition */
MCN_DEFINE(dshot_cmd, sizeof(dshot_cmd_bus_t));

/* MCN subscriber node */
static McnNode_t dshot_cmd_sub_node = RT_NULL;

/* Echo function for dshot command data */
static int dshot_cmd_echo(void* parameter) {
  dshot_cmd_bus_t data;
  if (mcn_copy_from_hub((McnHub*)parameter, &data) != RT_EOK) {
    return -1;
  }

  char m1[10], m2[10], m3[10], m4[10];
  float_to_string(data.motor_val[0], m1, sizeof(m1));
  float_to_string(data.motor_val[1], m2, sizeof(m2));
  float_to_string(data.motor_val[2], m3, sizeof(m3));
  float_to_string(data.motor_val[3], m4, sizeof(m4));

  rt_kprintf("[aMcnStabilize] dshot_cmd: %s, %s, %s, %s, ts: %lu\n", m1, m2, m3, m4, data.timestamp);

  return 0;
}

/* Initialize MCN dshot command */
int mcnDshotCmdInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(dshot_cmd), dshot_cmd_echo);
  if (result != RT_EOK) {
    rt_kprintf("[aMcnStabilize] Failed to advertise dshot_cmd topic: %d\n", result);
    return -1;
  }

  dshot_cmd_sub_node = mcn_subscribe(MCN_HUB(dshot_cmd), RT_NULL, RT_NULL);
  if (dshot_cmd_sub_node == RT_NULL) {
    rt_kprintf("[aMcnStabilize] Failed to subscribe to dshot_cmd topic\n");
    return -1;
  }

  rt_kprintf("[aMcnStabilize] DShot command MCN initialized\n");
  return 0;
}

/* Publish dshot command data to MCN */
int mcnDshotCmdPublish(const dshot_cmd_bus_t* cmd_data) {
  if (!cmd_data) {
    return -1;
  }

  return mcn_publish(MCN_HUB(dshot_cmd), cmd_data);
}

/* Acquire dshot command data from MCN */
int mcnDshotCmdAcquire(dshot_cmd_bus_t* cmd_data) {
  if (!cmd_data) {
    return -1;
  }

  if (dshot_cmd_sub_node == RT_NULL) {
    return -1;
  }

  if (mcn_poll(dshot_cmd_sub_node)) {
    return mcn_copy(MCN_HUB(dshot_cmd), dshot_cmd_sub_node, cmd_data);
  }

  return -1;
}

#endif /* PROJECT_MINIFLY_TASK_DSHOT_EN */