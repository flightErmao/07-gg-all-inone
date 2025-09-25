
#include <rtthread.h>
#include "protocolAtkpType.h"
#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

/* Pack and enqueue system status */
void packStatus(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed) {
  uint8_t _cnt = 0;
  atkp_t p;
  int16_t _temp;
  int32_t _temp2 = alt;

  p.msgID = UP_STATUS;

  _temp = (int)(roll * 100);
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = (int)(pitch * 100);
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = (int)(yaw * 100);
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);

  p.data[_cnt++] = BYTE3(_temp2);
  p.data[_cnt++] = BYTE2(_temp2);
  p.data[_cnt++] = BYTE1(_temp2);
  p.data[_cnt++] = BYTE0(_temp2);

  p.data[_cnt++] = fly_model;
  p.data[_cnt++] = armed;

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

/* Pack and enqueue IMU/mag data */
void packSensor(int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z, int16_t m_x, int16_t m_y,
                int16_t m_z) {
  uint8_t _cnt = 0;
  atkp_t p;
  int16_t _temp;

  p.msgID = UP_SENSER;

  _temp = a_x;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = a_y;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = a_z;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);

  _temp = g_x;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = g_y;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = g_z;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);

  _temp = m_x;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = m_y;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = m_z;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = 0;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

/* Pack and enqueue RC channels */
void packRcData(uint16_t thrust, uint16_t yaw, uint16_t roll, uint16_t pitch, uint16_t aux1, uint16_t aux2,
                uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_RCDATA;
  p.data[_cnt++] = BYTE1(thrust);
  p.data[_cnt++] = BYTE0(thrust);
  p.data[_cnt++] = BYTE1(yaw);
  p.data[_cnt++] = BYTE0(yaw);
  p.data[_cnt++] = BYTE1(roll);
  p.data[_cnt++] = BYTE0(roll);
  p.data[_cnt++] = BYTE1(pitch);
  p.data[_cnt++] = BYTE0(pitch);
  p.data[_cnt++] = BYTE1(aux1);
  p.data[_cnt++] = BYTE0(aux1);
  p.data[_cnt++] = BYTE1(aux2);
  p.data[_cnt++] = BYTE0(aux2);
  p.data[_cnt++] = BYTE1(aux3);
  p.data[_cnt++] = BYTE0(aux3);
  p.data[_cnt++] = BYTE1(aux4);
  p.data[_cnt++] = BYTE0(aux4);
  p.data[_cnt++] = BYTE1(aux5);
  p.data[_cnt++] = BYTE0(aux5);
  p.data[_cnt++] = BYTE1(aux6);
  p.data[_cnt++] = BYTE0(aux6);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

/* Pack and enqueue power */
void packPower(uint16_t voltage, uint16_t current) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_POWER;

  p.data[_cnt++] = BYTE1(voltage);
  p.data[_cnt++] = BYTE0(voltage);
  p.data[_cnt++] = BYTE1(current);
  p.data[_cnt++] = BYTE0(current);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

/* Pack and enqueue motor PWM */
void packMotorPWM(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7,
                  uint16_t m_8) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_MOTOR;

  p.data[_cnt++] = BYTE1(m_1);
  p.data[_cnt++] = BYTE0(m_1);
  p.data[_cnt++] = BYTE1(m_2);
  p.data[_cnt++] = BYTE0(m_2);
  p.data[_cnt++] = BYTE1(m_3);
  p.data[_cnt++] = BYTE0(m_3);
  p.data[_cnt++] = BYTE1(m_4);
  p.data[_cnt++] = BYTE0(m_4);
  p.data[_cnt++] = BYTE1(m_5);
  p.data[_cnt++] = BYTE0(m_5);
  p.data[_cnt++] = BYTE1(m_6);
  p.data[_cnt++] = BYTE0(m_6);
  p.data[_cnt++] = BYTE1(m_7);
  p.data[_cnt++] = BYTE0(m_7);
  p.data[_cnt++] = BYTE1(m_8);
  p.data[_cnt++] = BYTE0(m_8);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

/* Pack and enqueue barometer/alt data */
void packSensor2(int32_t bar_alt, uint16_t csb_alt) {
  uint8_t _cnt = 0;
  atkp_t p;

  p.msgID = UP_SENSER2;

  p.data[_cnt++] = BYTE3(bar_alt);
  p.data[_cnt++] = BYTE2(bar_alt);
  p.data[_cnt++] = BYTE1(bar_alt);
  p.data[_cnt++] = BYTE0(bar_alt);

  p.data[_cnt++] = BYTE1(csb_alt);
  p.data[_cnt++] = BYTE0(csb_alt);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

/* Pack and enqueue PID params */
void packPid(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p,
             float p3_i, float p3_d) {
  uint8_t _cnt = 0;
  atkp_t p;
  int16_t _temp;

  p.msgID = 0x10 + group - 1;

  _temp = p1_p * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p1_i * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p1_d * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p2_p * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p2_i * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p2_d * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p3_p * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p3_i * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);
  _temp = p3_d * 10;
  p.data[_cnt++] = BYTE1(_temp);
  p.data[_cnt++] = BYTE0(_temp);

  p.dataLen = _cnt;
  anotcMqStash(&p);
}

void packCheck(uint8_t head, uint8_t check_sum) {
  atkp_t p;

  p.msgID = UP_CHECK;
  p.dataLen = 2;
  p.data[0] = head;
  p.data[1] = check_sum;
  anotcMqStash(&p);
}

uint8_t atkpCheckSum(atkp_t *packet) {
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