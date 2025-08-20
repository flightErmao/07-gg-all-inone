#ifndef __TASK_ANOTC_TELEM_H__
#define __TASK_ANOTC_TELEM_H__

#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

#endif /*ATKP_H*/
