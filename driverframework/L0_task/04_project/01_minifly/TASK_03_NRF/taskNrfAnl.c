#include <rtthread.h>
#include <rtdevice.h>
#include "taskNrfRec.h"
#include "rtconfig.h"
#include "../../../02_tools/TASK_01_ANOTC_TELEM/protocolAtkpInterface.h"
#include "taskNrfDefs.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

/*下行指令、应答宏已移至 taskNrfDefs.h */
/*下行指令ID*/
typedef enum {
  DOWN_COMMAND = 0x01,
  DOWN_ACK = 0x02,
  DOWN_RCDATA = 0x03,
  DOWN_POWER = 0x05,
  DOWN_FLYMODE = 0x0A,
  DOWN_PID1 = 0x10,
  DOWN_PID2 = 0x11,
  DOWN_PID3 = 0x12,
  DOWN_PID4 = 0x13,
  DOWN_PID5 = 0x14,
  DOWN_PID6 = 0x15,
  DOWN_RADIO = 0x40,

  DOWN_REMOTER = 0x50,
} downmsgID_e;

static struct rt_thread taskNrfAnlTid;
static rt_align(RT_ALIGN_SIZE) rt_uint8_t taskNrfAnlStack[THREAD_STACK_SIZE];

extern bool flyable;

typedef unsigned char u8;
typedef short s16;

typedef struct {
  float kp;
  float ki;
  float kd;
} pid_param_t;

extern pid_param_t pidRateRoll, pidRatePitch, pidRateYaw;
extern pid_param_t pidAngleRoll, pidAnglePitch, pidAngleYaw;
extern pid_param_t pidVZ, pidZ, pidVX, pidVY, pidX, pidY;

extern void packPid(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p,
                    float p3_i, float p3_d);
extern void packCheck(uint8_t msgId, uint8_t cksum);
extern u8 atkpCheckSum(atkp_t *p);

extern void resetConfigParamPID(void);
extern void attitudeControlInit(float rate_dt, float angle_dt);
extern void positionControlInit(float vel_dt, float pos_dt);

extern void attitudePIDwriteToConfigParam(void);
extern void positionPIDwriteToConfigParam(void);

extern void setMotorPWM(s16 enable, s16 m1_set, s16 m2_set, s16 m3_set, s16 m4_set);

typedef struct {
  uint16_t ch[8];
} joystickFlyui16_t;
extern joystickFlyui16_t rcdata;
extern void pmSyslinkUpdate(atkp_t *p);
extern void remoterCtrlProcess(atkp_t *p);

static void atkpReceiveAnl(atkp_t *anlPacket) {
  if (anlPacket->msgID == DOWN_COMMAND) {
    switch (anlPacket->data[0]) {
      case D_COMMAND_ACC_CALIB:
        break;

      case D_COMMAND_GYRO_CALIB:
        break;

      case D_COMMAND_MAG_CALIB:
        break;

      case D_COMMAND_BARO_CALIB:
        break;

      case D_COMMAND_ACC_CALIB_STEP1:
        break;
      case D_COMMAND_ACC_CALIB_STEP2:
        break;
      case D_COMMAND_ACC_CALIB_STEP3:
        break;
      case D_COMMAND_ACC_CALIB_STEP4:
        break;
      case D_COMMAND_ACC_CALIB_STEP5:
        break;
      case D_COMMAND_ACC_CALIB_STEP6:
        break;

      case D_COMMAND_FLIGHT_LOCK:
        flyable = false;
        break;

      case D_COMMAND_FLIGHT_ULOCK:
        flyable = true;
    }
  } else if (anlPacket->msgID == DOWN_ACK) {
    if (anlPacket->data[0] == D_ACK_READ_PID) /*读取PID参数*/
    {
      packPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd, pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
              pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd);
      packPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd, pidAnglePitch.kp, pidAnglePitch.ki,
              pidAnglePitch.kd, pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd);
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
      packPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd, pidAnglePitch.kp, pidAnglePitch.ki,
              pidAnglePitch.kd, pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd);
      packPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd, pidZ.kp, pidZ.ki, pidZ.kd, pidVX.kp, pidVX.ki, pidVX.kd);
      packPid(4, pidX.kp, pidX.ki, pidX.kd, 0, 0, 0, 0, 0, 0);
    }
  } else if (anlPacket->msgID == DOWN_RCDATA) {
    rcdata = *((joystickFlyui16_t *)anlPacket->data);
  } else if (anlPacket->msgID == DOWN_POWER) /*nrf51822*/
  {
    pmSyslinkUpdate(anlPacket);
  } else if (anlPacket->msgID == DOWN_REMOTER) /*遥控器*/
  {
    remoterCtrlProcess(anlPacket);
  } else if (anlPacket->msgID == DOWN_PID1) {
    pidRateRoll.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
    pidRateRoll.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
    pidRateRoll.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
    pidRatePitch.kp = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
    pidRatePitch.ki = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
    pidRatePitch.kd = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
    pidRateYaw.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
    pidRateYaw.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
    pidRateYaw.kd = 0.1 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));
    u8 cksum = atkpCheckSum(anlPacket);
    packCheck(anlPacket->msgID, cksum);
  } else if (anlPacket->msgID == DOWN_PID2) {
    pidAngleRoll.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
    pidAngleRoll.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
    pidAngleRoll.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
    pidAnglePitch.kp = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
    pidAnglePitch.ki = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
    pidAnglePitch.kd = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
    pidAngleYaw.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
    pidAngleYaw.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
    pidAngleYaw.kd = 0.1 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));
    u8 cksum = atkpCheckSum(anlPacket);
    packCheck(anlPacket->msgID, cksum);
  } else if (anlPacket->msgID == DOWN_PID3) {
    pidVZ.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
    pidVZ.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
    pidVZ.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

    pidZ.kp = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
    pidZ.ki = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
    pidZ.kd = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));

    pidVX.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
    pidVX.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
    pidVX.kd = 0.1 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

    pidVY = pidVX;  // 位置速率PID，X\Y方向是一样的

    u8 cksum = atkpCheckSum(anlPacket);
    packCheck(anlPacket->msgID, cksum);
  } else if (anlPacket->msgID == DOWN_PID4) {
    pidX.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
    pidX.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
    pidX.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

    pidY = pidX;  // 位置保持PID，X\Y方向是一样的

    u8 cksum = atkpCheckSum(anlPacket);
    packCheck(anlPacket->msgID, cksum);
  } else if (anlPacket->msgID == DOWN_PID5) {
    u8 cksum = atkpCheckSum(anlPacket);
    packCheck(anlPacket->msgID, cksum);
  } else if (anlPacket->msgID == DOWN_PID6) {
    //		s16 temp1  = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
    //		s16 temp2  = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
    //		s16 temp3  = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
    s16 enable = ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
    s16 m1_set = ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
    s16 m2_set = ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
    s16 m3_set = ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
    s16 m4_set = ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
    setMotorPWM(enable, m1_set, m2_set, m3_set, m4_set);
    attitudePIDwriteToConfigParam();
    positionPIDwriteToConfigParam();
    u8 cksum = atkpCheckSum(anlPacket);
    packCheck(anlPacket->msgID, cksum);
  }
}

static void taskNrfAnlEntry(void *parameter) {
  struct rt_messagequeue *recv_mq = NULL;
  while (1) {
    if (recv_mq != NULL) {
      break;
    }
    recv_mq = getNrfRecvMq();
    rt_thread_mdelay(100);
  }

  while (1) {
    atkp_t pkt;
    rt_memset(&pkt, 0, sizeof(pkt));
    if (rt_mq_recv(recv_mq, &pkt, sizeof(pkt), RT_WAITING_FOREVER) == RT_EOK) {
      atkpReceiveAnl(&pkt);
    }
  }
}

static int taskNrfAnlInit(void) {
  rt_thread_init(&taskNrfAnlTid, "L0_minifly_nrfAnl", taskNrfAnlEntry, RT_NULL, taskNrfAnlStack, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&taskNrfAnlTid);
  rt_kprintf("nrf anl task started\n");
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_NRF_EN
INIT_APP_EXPORT(taskNrfAnlInit);
#endif