#include "anlRcData.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_RCDATA_EN

typedef struct {
  uint16_t ch[8];
} joystickFlyui16_t;

extern joystickFlyui16_t rcdata;

void anlRcData(atkp_t *anlPacket) { rcdata = *((joystickFlyui16_t *)anlPacket->data); }

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_RCDATA_EN */