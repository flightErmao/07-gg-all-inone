#include <stdint.h>

/* 通用字节访问宏 */
#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

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
