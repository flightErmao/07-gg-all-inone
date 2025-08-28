#include "anlAck.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_ACK_EN

extern void packPid(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p,
                    float p3_i, float p3_d);
extern void resetConfigParamPID(void);
extern void attitudeControlInit(float rate_dt, float angle_dt);
extern void positionControlInit(float vel_dt, float pos_dt);

extern pid_param_t pidRateRoll, pidRatePitch, pidRateYaw;
extern pid_param_t pidAngleRoll, pidAnglePitch, pidAngleYaw;
extern pid_param_t pidVZ, pidZ, pidVX, pidVY, pidX, pidY;

void anlAck(atkp_t *anlPacket) {
  if (anlPacket->data[0] == D_ACK_READ_PID) /*读取PID参数*/
  {
    packPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd, pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
            pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd);
    packPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd, pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
            pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd);
    packPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd, pidZ.kp, pidZ.ki, pidZ.kd, pidVX.kp, pidVX.ki, pidVX.kd);
    packPid(4, pidX.kp, pidX.ki, pidX.kd, 0, 0, 0, 0, 0, 0);
  }
  if (anlPacket->data[0] == D_ACK_RESET_PARAM) /*恢复默认参数*/
  {
    resetConfigParamPID();

    attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT);        /*初始化姿态PID*/
    positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/

    packPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd, pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
            pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd);
    packPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd, pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
            pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd);
    packPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd, pidZ.kp, pidZ.ki, pidZ.kd, pidVX.kp, pidVX.ki, pidVX.kd);
    packPid(4, pidX.kp, pidX.ki, pidX.kd, 0, 0, 0, 0, 0, 0);
  }
}

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_ACK_EN */