#include <rtthread.h>
#include "protocolAtkpType.h"
#include "protocolAtkpInterface.h"
#include "deviceManager.h"
#include "taskAnotcTelem.h"

/* Message send method */
typedef enum {
  MSG_ASYNC = 0,
  MSG_SYNC,
} msg_send_method_e;

/* Generic: send N float data, support sync/async */
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

void sendUserDatafloat9(uint8_t group, float a, float b, float c, float d, float e, float f, float g, float h, float i) {
  float values[9] = {a, b, c, d, e, f, g, h, i};
  anotc_telem_send_floats(group, values, 9, MSG_ASYNC);
}

void sendUserDatafloat6_u32(uint8_t group, float a, float b, float c, float d, float e, float f, uint32_t uint32_t) {
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

  // add uint32_t data
  p.data[_cnt++] = BYTE3(uint32_t);
  p.data[_cnt++] = BYTE2(uint32_t);
  p.data[_cnt++] = BYTE1(uint32_t);
  p.data[_cnt++] = BYTE0(uint32_t);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

void sendUserDatafloat12_u32(uint8_t group, float a, float b, float c, float d, float e, float f, float g, float h,
                             float i, float j, float k, float l, uint32_t uint32_t) {
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

  // add uint32_t data
  p.data[_cnt++] = BYTE3(uint32_t);
  p.data[_cnt++] = BYTE2(uint32_t);
  p.data[_cnt++] = BYTE1(uint32_t);
  p.data[_cnt++] = BYTE0(uint32_t);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

void sendUserDatauint16_4(uint8_t group, uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  uint16_t values[4] = {a, b, c, d};
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t temp = values[i];
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
  }

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

void sendUserDatauint16_8(uint8_t group, uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e, uint16_t f, uint16_t g, uint16_t h) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_USER_DATA1 + group - 1;

  uint16_t values[8] = {a, b, c, d, e, f, g, h};
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t temp = values[i];
    p.data[_cnt++] = BYTE1(temp);
    p.data[_cnt++] = BYTE0(temp);
  }

  p.dataLen = _cnt;
  anotcMqStash(&p);
}
