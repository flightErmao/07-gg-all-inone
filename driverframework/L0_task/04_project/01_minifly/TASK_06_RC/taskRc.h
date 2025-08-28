#ifndef TASK_RC_MINIFLY_H
#define TASK_RC_MINIFLY_H

#include "stdint.h"

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
} Pilot_Cmd_Bus;

/* RC数据获取接口 */
void rcPilotCmdAcquire(Pilot_Cmd_Bus *rc_data);

#endif
