#ifndef __TASK_RC_H__
#define __TASK_RC_H__

#include <rtthread.h>

typedef enum {
  ARM_STATUS_DISARM = 0,
  ARM_STATUS_ARM = 1,
} ARM_STATUS_e;

typedef enum {
  CTRL_MODE_ANGLERATE = 0,
  CTRL_MODE_ANGLE = 1,
} CTRL_MODE_e;

typedef struct {
  uint32_t timestamp;
  float stick_yaw;
  float stick_throttle;
  float stick_roll;
  float stick_pitch;
  ARM_STATUS_e ram_status;
  CTRL_MODE_e ctrl_mode;
} pilot_cmd_bus_t;

void rcPilotCmdAcquire(pilot_cmd_bus_t* rc_data);
rt_uint16_t* getRcChannels(void);

#endif