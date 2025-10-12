#include "anlPower.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_POWER_EN

extern void pmSyslinkUpdate(atkp_t *p);

void anlPower(atkp_t *anlPacket) { pmSyslinkUpdate(anlPacket); }

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_POWER_EN */