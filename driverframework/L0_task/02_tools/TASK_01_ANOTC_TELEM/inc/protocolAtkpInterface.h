#ifndef __PROTOCOL_ATKP_INTERFACE_H__
#define __PROTOCOL_ATKP_INTERFACE_H__

#include <stdint.h>

#define PERIOD_1ms 1
#define PERIOD_10ms 10
#define PERIOD_20ms 20
#define PERIOD_30ms 30
#define PERIOD_40ms 40
#define PERIOD_50ms 50
#define PERIOD_100ms 100

#define D_ACK_READ_PID 0x01
#define D_ACK_READ_VERSION 0xA0
#define D_ACK_RESET_PARAM 0xA1

#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

typedef struct {
  uint8_t msgID;
  uint8_t dataLen;
  uint8_t data[128];
} atkp_t;

void sendUserDatafloat3(uint8_t group, float a, float b, float c);
void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f);
void sendUserDatafloat6_u32(uint8_t group, float a, float b, float c, float d, float e, float f, uint32_t uint32_t);
void sendUserDatafloat12_u32(uint8_t group, float a, float b, float c, float d, float e, float f, float g, float h,
                             float i, float j, float k, float l, uint32_t uint32_t);
void sendUserDatauint16_8(uint8_t group, uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e, uint16_t f, uint16_t g, uint16_t h);

void packStatus(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void packSensor(int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z, int16_t m_x, int16_t m_y,
                int16_t m_z);
void packSensor2(int32_t bar_alt, uint16_t csb_alt);
void packPid(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p,
             float p3_i, float p3_d);
void packPower(uint16_t voltage, uint16_t current);
void packMotorPWM(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7,
                  uint16_t m_8);
void packRcData(uint16_t thrust, uint16_t yaw, uint16_t roll, uint16_t pitch, uint16_t aux1, uint16_t aux2,
                uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6);

void packCheck(uint8_t head, uint8_t check_sum);
uint8_t atkpCheckSum(atkp_t *packet);

#endif /* __PROTOCOL_ATKP_INTERFACE_H__ */