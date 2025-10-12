#include "anlRemote.h"
#include "rtthread.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_REMOTE_EN

typedef enum {
  REMOTER_CMD,
  REMOTER_DATA,
} remoterType_e;

typedef rt_packed(struct {
  float roll;
  float pitch;
  float yaw;
  float thrust;
  float trimPitch;
  float trimRoll;
  uint8_t ctrlMode;
  bool flightMode;
  bool RCLock;
}) remoterSendData_t;

rcRawData_t rcRawData_;

void anlRemote(atkp_t *anlPacket) {
  if (anlPacket->data[0] == REMOTER_DATA) {
    remoterSendData_t remoterSendData = *(remoterSendData_t *)(anlPacket->data + 1);
    rcRawData_.timestamp = rt_tick_get();
    rcRawData_.roll = remoterSendData.roll;
    rcRawData_.pitch = remoterSendData.pitch;
    rcRawData_.yaw = remoterSendData.yaw;
    rcRawData_.thrust = remoterSendData.thrust;
    rcRawData_.arm_status = !remoterSendData.RCLock;
    rcRawData_.ctrl_mode = remoterSendData.ctrlMode;
  }
}

rcRawData_t getRcRawData(void) { return rcRawData_; }

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_REMOTE_EN */
