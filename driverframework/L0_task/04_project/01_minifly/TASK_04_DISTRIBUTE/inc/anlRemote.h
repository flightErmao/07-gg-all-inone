#ifndef ANL_REMOTE_H
#define ANL_REMOTE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocolAtkpInterface.h"

typedef struct {
  uint32_t timestamp;
  float roll;
  float pitch;
  float yaw;
  float thrust;
  bool arm_status;
  bool ctrl_mode;
} rcRawData_t;

void anlRemote(atkp_t *anlPacket);
rcRawData_t getRcRawData(void);

#endif /* ANL_REMOTE_H */