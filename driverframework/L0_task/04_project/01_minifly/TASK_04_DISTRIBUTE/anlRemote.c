#include "anlRemote.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_REMOTE_EN

extern void remoterCtrlProcess(atkp_t *p);

void anlRemote(atkp_t *anlPacket) { remoterCtrlProcess(anlPacket); }

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_REMOTE_EN */
