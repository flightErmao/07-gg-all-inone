#include "anlPid.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN

extern void packCheck(uint8_t msgId, uint8_t cksum);
extern void setMotorPWM(int16_t enable, int16_t m1_set, int16_t m2_set, int16_t m3_set, int16_t m4_set);
extern void attitudePIDwriteToConfigParam(void);
extern void positionPIDwriteToConfigParam(void);

extern pid_param_t pidRateRoll, pidRatePitch, pidRateYaw;
extern pid_param_t pidAngleRoll, pidAnglePitch, pidAngleYaw;
extern pid_param_t pidVZ, pidZ, pidVX, pidVY, pidX, pidY;

static uint8_t atkpCheckSum(atkp_t *packet) {
  uint8_t sum;
  sum = DOWN_BYTE1;
  sum += DOWN_BYTE2;
  sum += packet->msgID;
  sum += packet->dataLen;
  for (int i = 0; i < packet->dataLen; i++) {
    sum += packet->data[i];
  }
  return sum;
}

void anlPid1(atkp_t *anlPacket) {
  pidRateRoll.kp = 0.1 * ((int16_t)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
  pidRateRoll.ki = 0.1 * ((int16_t)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
  pidRateRoll.kd = 0.1 * ((int16_t)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
  pidRatePitch.kp = 0.1 * ((int16_t)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
  pidRatePitch.ki = 0.1 * ((int16_t)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
  pidRatePitch.kd = 0.1 * ((int16_t)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
  pidRateYaw.kp = 0.1 * ((int16_t)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
  pidRateYaw.ki = 0.1 * ((int16_t)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
  pidRateYaw.kd = 0.1 * ((int16_t)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

  uint8_t cksum = atkpCheckSum(anlPacket);
  packCheck(anlPacket->msgID, cksum);
}

void anlPid2(atkp_t *anlPacket) {
  pidAngleRoll.kp = 0.1 * ((int16_t)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
  pidAngleRoll.ki = 0.1 * ((int16_t)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
  pidAngleRoll.kd = 0.1 * ((int16_t)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
  pidAnglePitch.kp = 0.1 * ((int16_t)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
  pidAnglePitch.ki = 0.1 * ((int16_t)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
  pidAnglePitch.kd = 0.1 * ((int16_t)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
  pidAngleYaw.kp = 0.1 * ((int16_t)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
  pidAngleYaw.ki = 0.1 * ((int16_t)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
  pidAngleYaw.kd = 0.1 * ((int16_t)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

  uint8_t cksum = atkpCheckSum(anlPacket);
  packCheck(anlPacket->msgID, cksum);
}

void anlPid3(atkp_t *anlPacket) {
  pidVZ.kp = 0.1 * ((int16_t)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
  pidVZ.ki = 0.1 * ((int16_t)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
  pidVZ.kd = 0.1 * ((int16_t)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

  pidZ.kp = 0.1 * ((int16_t)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
  pidZ.ki = 0.1 * ((int16_t)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
  pidZ.kd = 0.1 * ((int16_t)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));

  pidVX.kp = 0.1 * ((int16_t)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
  pidVX.ki = 0.1 * ((int16_t)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
  pidVX.kd = 0.1 * ((int16_t)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

  pidVY = pidVX;  // 位置速率PID，X\Y方向是一样的

  uint8_t cksum = atkpCheckSum(anlPacket);
  packCheck(anlPacket->msgID, cksum);
}

void anlPid4(atkp_t *anlPacket) {
  pidX.kp = 0.1 * ((int16_t)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
  pidX.ki = 0.1 * ((int16_t)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
  pidX.kd = 0.1 * ((int16_t)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

  pidY = pidX;  // 位置保持PID，X\Y方向是一样的

  uint8_t cksum = atkpCheckSum(anlPacket);
  packCheck(anlPacket->msgID, cksum);
}

void anlPid5(atkp_t *anlPacket) {
  uint8_t cksum = atkpCheckSum(anlPacket);
  packCheck(anlPacket->msgID, cksum);
}

void anlPid6(atkp_t *anlPacket) {
  int16_t enable = ((int16_t)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
  int16_t m1_set = ((int16_t)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
  int16_t m2_set = ((int16_t)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
  int16_t m3_set = ((int16_t)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
  int16_t m4_set = ((int16_t)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));

  setMotorPWM(enable, m1_set, m2_set, m3_set, m4_set);
  attitudePIDwriteToConfigParam();
  positionPIDwriteToConfigParam();

  uint8_t cksum = atkpCheckSum(anlPacket);
  packCheck(anlPacket->msgID, cksum);
}

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN */
