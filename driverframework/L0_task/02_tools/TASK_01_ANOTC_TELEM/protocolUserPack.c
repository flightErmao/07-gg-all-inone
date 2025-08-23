#include <rtthread.h>
#include "protocolAtkpType.h"
#include "protocolAtkpInterface.h"
#include "deviceManager.h"
#include "taskAnotcTelem.h"

/* 消息发送方式 */
typedef enum {
  MSG_ASYNC = 0,
  MSG_SYNC,
} msg_send_method_e;

/* 通用: 发送 N 个 float 数据，支持同步/异步 */
static void anotc_telem_send_floats(uint8_t group, const float *values, uint8_t count, msg_send_method_e method) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  for (uint8_t i = 0; i < count; i++) {
    float temp = values[i];
    p.data[_cnt++] = BYTE3(temp);
    p.data[_cnt++] = BYTE2(temp);
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
  }

  p.dataLen = _cnt;
  if (method == MSG_ASYNC) {
    anotcMqStash(&p);
  } else {
    anotcDeviceSendDirect(&p);
  }
}

#ifdef UNUSED_FUNCTION_WARNING_SUPPRESS
static void anotc_telem_send_int16(uint8_t group, int16_t *values, uint8_t count, msg_send_method_e method) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  for (uint8_t i = 0; i < count; i++) {
    int16_t temp = values[i];
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
  }
  p.dataLen = _cnt;
  if (method == MSG_ASYNC) {
    anotcMqStash(&p);
  } else {
    anotcDeviceSendDirect(&p);
  }
}
#endif

void sendUserDatafloat3(uint8_t group, float a, float b, float c) {
  float values[3] = {a, b, c};
  anotc_telem_send_floats(group, values, 3, MSG_ASYNC);
}

void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f) {
  float values[6] = {a, b, c, d, e, f};
  anotc_telem_send_floats(group, values, 6, MSG_ASYNC);
}

void sendUserDatafloat6_u32(uint8_t group, float a, float b, float c, float d, float e, float f, uint32_t u32) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  float values[6] = {a, b, c, d, e, f};
  for (uint8_t i = 0; i < 6; i++) {
    float temp = values[i];
    p.data[_cnt++] = BYTE3(temp);
    p.data[_cnt++] = BYTE2(temp);
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
  }

  // 添加 u32 数据
  p.data[_cnt++] = BYTE3(u32);
  p.data[_cnt++] = BYTE2(u32);
  p.data[_cnt++] = BYTE1(u32);
  p.data[_cnt++] = BYTE0(u32);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

void sendUserDatafloat12_u32(uint8_t group, float a, float b, float c, float d, float e, float f, float g, float h,
                             float i, float j, float k, float l, uint32_t u32) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  float values[12] = {a, b, c, d, e, f, g, h, i, j, k, l};
  for (uint8_t i = 0; i < 12; i++) {
    float temp = values[i];
    p.data[_cnt++] = BYTE3(temp);
    p.data[_cnt++] = BYTE2(temp);
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
  }

  // 添加 u32 数据
  p.data[_cnt++] = BYTE3(u32);
  p.data[_cnt++] = BYTE2(u32);
  p.data[_cnt++] = BYTE1(u32);
  p.data[_cnt++] = BYTE0(u32);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}
