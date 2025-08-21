#ifndef __TASK_ANOTC_TELEM_H__
#define __TASK_ANOTC_TELEM_H__

#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 线程与消息队列配置 */
#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define MSG_NUM 30
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)

/* 通用字节访问宏 */
#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

/* 协议帧头 */
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

/* ATKP 尺寸相关 */
#define ATKP_MAX_DATA_SIZE 128
#define ATKP_PROTOCOL_HEAD_SIZE 6
#define ATKP_ANOTC_TELEM_BUF_SIZE (ATKP_MAX_DATA_SIZE + ATKP_PROTOCOL_HEAD_SIZE)

/* 波特率（默认） */
#define ANOTC_TELEM_BAUD_RATE 500000

/* 下行命令/ACK 定义 */
#define D_COMMAND_ACC_CALIB 0x01
#define D_COMMAND_GYRO_CALIB 0x02
#define D_COMMAND_MAG_CALIB 0x04
#define D_COMMAND_BARO_CALIB 0x05
#define D_COMMAND_ACC_CALIB_EXIT 0x20
#define D_COMMAND_ACC_CALIB_STEP1 0x21
#define D_COMMAND_ACC_CALIB_STEP2 0x22
#define D_COMMAND_ACC_CALIB_STEP3 0x23
#define D_COMMAND_ACC_CALIB_STEP4 0x24
#define D_COMMAND_ACC_CALIB_STEP5 0x25
#define D_COMMAND_ACC_CALIB_STEP6 0x26
#define D_COMMAND_FLIGHT_LOCK 0xA0
#define D_COMMAND_FLIGHT_ULOCK 0xA1

#define D_ACK_READ_PID 0x01
#define D_ACK_READ_VERSION 0xA0
#define D_ACK_RESET_PARAM 0xA1

/* ATKP 数据结构 */
typedef struct
{
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[ATKP_MAX_DATA_SIZE];
} atkp_t;

/* 上行指令ID */
typedef enum
{
    UP_VERSION = 0x00,
    UP_STATUS = 0x01,
    UP_SENSER = 0x02,
    UP_RCDATA = 0x03,
    UP_GPSDATA = 0x04,
    UP_POWER = 0x05,
    UP_MOTOR = 0x06,
    UP_SENSER2 = 0x07,
    UP_FLYMODE = 0x0A,
    UP_SPEED = 0x0B,
    UP_PID1 = 0x10,
    UP_PID2 = 0x11,
    UP_PID3 = 0x12,
    UP_PID4 = 0x13,
    UP_PID5 = 0x14,
    UP_PID6 = 0x15,
    UP_RADIO = 0x40,
    UP_MSG = 0xEE,
    UP_CHECK = 0xEF,

    UP_REMOTER = 0x50,
    UP_PRINTF = 0x51,

    UP_USER_DATA1 = 0xF1,
    UP_USER_DATA2 = 0xF2,
    UP_USER_DATA3 = 0xF3,
    UP_USER_DATA4 = 0xF4,
    UP_USER_DATA5 = 0xF5,
    UP_USER_DATA6 = 0xF6,
    UP_USER_DATA7 = 0xF7,
    UP_USER_DATA8 = 0xF8,
    UP_USER_DATA9 = 0xF9,
    UP_USER_DATA10 = 0xFA,
} upmsgID_e;

/* 下行指令ID */
typedef enum
{
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

#define USER_FRAME_INDEX1 0
#define USER_FRAME_INDEX2 1
#define USER_FRAME_INDEX3 2
#define USER_FRAME_INDEX4 3
#define USER_FRAME_INDEX5 4
#define USER_FRAME_INDEX6 5
#define USER_FRAME_INDEX7 6
#define USER_FRAME_INDEX8 7
#define USER_FRAME_INDEX9 8
#define USER_FRAME_INDEX10 9
#define USER_FRAME_INDEX11 10
#define USER_FRAME_INDEX12 11

#define IMU_DATA 1
#define BARO_DATA 2
#define MAG_DATA 3

typedef enum {
    MSG_ASYNC = 0,
    MSG_SYNC,
} msg_send_method_e;

int task_anotc_telem(void);

/* moved to module, keep prototypes here for backward include if needed */
void anotc_telem_sendUserDataLine6_float(uint8_t group, float *buf_data_cat, msg_send_method_e method);
void setUserData_float(uint8_t index, float *buf, float value);
void sendUserDatafloat3(uint8_t group, float a, float b, float c);
void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f);
void sendUserDataLine6_int16_sync(uint8_t group, int16_t *buf_data_cat);
void sendUserDataLine6_int16_async(uint8_t group, int16_t *buf_data_cat);

#endif /*ATKP_H*/
