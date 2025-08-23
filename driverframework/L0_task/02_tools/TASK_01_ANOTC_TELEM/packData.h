#ifndef __PACK_DATA_H__
#define __PACK_DATA_H__

#include <stdint.h>

#define PERIOD_1ms 1
#define PERIOD_10ms 10
#define PERIOD_20ms 20
#define PERIOD_30ms 30
#define PERIOD_40ms 40
#define PERIOD_50ms 50
#define PERIOD_100ms 100

/*上行指令ID*/
typedef enum {
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

  // 新增：光流质量数据
  UP_OPTICAL_FLOW_QUALITY = 0xFB,

  // 新增：光流质量对比数据
  UP_OPTICAL_FLOW_COMPARISON = 0xFC,
} upmsgID_e;

/* ATKP 数据结构 */
typedef struct {
  uint8_t msgID;
  uint8_t dataLen;
  uint8_t data[128];  // ATKP_MAX_DATA_SIZE
} atkp_t;

void sendUserDatafloat3(uint8_t group, float a, float b, float c);
void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f);
void sendUserDatafloat6_u32(uint8_t group, float a, float b, float c, float d, float e, float f, uint32_t u32);

#endif /* __PACK_DATA_H__ */
