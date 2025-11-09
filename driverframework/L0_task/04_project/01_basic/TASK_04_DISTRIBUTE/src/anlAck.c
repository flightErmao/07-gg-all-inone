#include "anlAck.h"
#include "rtconfig.h"

#ifdef PROJECT_MINIFLY_TASK04_DISTRIBUTE_ACK_EN

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
#include "stateControl.h"
#endif

#include "pidMinifly.h"
#include "taskParam.h"

extern void packPid(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p,
                    float p3_i, float p3_d);
// extern void resetConfigParamPID(void);
// extern void attitudeControlInit(float rate_dt, float angle_dt);
// extern void positionControlInit(float vel_dt, float pos_dt);

void anlAck(atkp_t *anlPacket) {
  if (anlPacket->data[0] == D_ACK_READ_PID)
  {
    // Read PID parameters from configParam to ensure we have the current values
    configParam_t configParam_temp;
    getConfigParam(&configParam_temp);
    
    packPid(1, configParam_temp.pidRate.roll.kp, configParam_temp.pidRate.roll.ki, configParam_temp.pidRate.roll.kd,
            configParam_temp.pidRate.pitch.kp, configParam_temp.pidRate.pitch.ki, configParam_temp.pidRate.pitch.kd,
            configParam_temp.pidRate.yaw.kp, configParam_temp.pidRate.yaw.ki, configParam_temp.pidRate.yaw.kd);
    packPid(2, configParam_temp.pidAngle.roll.kp, configParam_temp.pidAngle.roll.ki, configParam_temp.pidAngle.roll.kd,
            configParam_temp.pidAngle.pitch.kp, configParam_temp.pidAngle.pitch.ki, configParam_temp.pidAngle.pitch.kd,
            configParam_temp.pidAngle.yaw.kp, configParam_temp.pidAngle.yaw.ki, configParam_temp.pidAngle.yaw.kd);
    // packPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd, pidZ.kp, pidZ.ki, pidZ.kd, pidVX.kp, pidVX.ki, pidVX.kd);
    // packPid(4, pidX.kp, pidX.ki, pidX.kd, 0, 0, 0, 0, 0, 0);
  }
  //   if (anlPacket->data[0] == D_ACK_RESET_PARAM) /*恢复默认参数*/
  //   {
  //     resetConfigParamPID();

  //     attitudeControlInit(RATE_PID_DT, ANGLE_PID_DT);        /*初始化姿态PID*/
  //     positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/

  //     packPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd, pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
  //             pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd);
  //     packPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd, pidAnglePitch.kp, pidAnglePitch.ki,
  //     pidAnglePitch.kd,
  //             pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd);
  //     packPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd, pidZ.kp, pidZ.ki, pidZ.kd, pidVX.kp, pidVX.ki, pidVX.kd);
  //     packPid(4, pidX.kp, pidX.ki, pidX.kd, 0, 0, 0, 0, 0, 0);
  //   }
}

#endif /* PROJECT_MINIFLY_TASK04_DISTRIBUTE_ACK_EN */