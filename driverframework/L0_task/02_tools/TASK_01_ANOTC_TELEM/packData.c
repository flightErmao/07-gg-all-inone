#include <rtthread.h>
#include "deviceManager.h"
#include "taskAnotcTelem.h"

#define PERIOD_1ms 1
#define PERIOD_10ms 10
#define PERIOD_20ms 20
#define PERIOD_30ms 30
#define PERIOD_40ms 40
#define PERIOD_50ms 50
#define PERIOD_100ms 100

/* 通用字节访问宏 */
#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

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

void sendUserDatafloat3(uint8_t group, float a, float b, float c) {
  float values[3] = {a, b, c};
  anotc_telem_send_floats(group, values, 3, MSG_ASYNC);
}

void sendUserDatafloat6(uint8_t group, float a, float b, float c, float d, float e, float f) {
  float values[6] = {a, b, c, d, e, f};
  anotc_telem_send_floats(group, values, 6, MSG_ASYNC);
}

#ifdef PROJECT_MINIFLY_SENSOR

#include "sensorsTypes.h"
#include "taskSensorMinifly.h"

extern void sensorsAcquire(sensorData_t *sensors);

void sendSensorImuData(uint16_t count_ms) {
  if (!(count_ms % PERIOD_10ms)) {
    sensorsAcquire(&sensors);
    sendUserDatafloat6(1, sensors.acc_filter.x, sensors.acc_filter.y, sensors.acc_filter.z, sensors.gyro_filter.x,
                       sensors.gyro_filter.y, sensors.gyro_filter.z);
  }
}

int addPeriodFunListProjectMiniFlySensor(void) {
  anotc_telem_add_sensor_func(sendSensorImuData);
  return 0;
}

INIT_APP_EXPORT(addPeriodFunListProjectMiniFlySensor);

#else

/* 未启用PROJECT_MINIFLY_SENSOR时，注册假的IMU数据发送函数 */
static void sendSensorImuData(uint16_t count_ms) {
  static float fx = 0.0f, fy = 1.0f, fz = 2.0f;
  static float gx = 3.0f, gy = 4.0f, gz = 5.0f;

  if (!(count_ms % PERIOD_10ms)) {
    /* 累加生成变化数据 */
    fx += 0.1f; fy += 0.2f; fz += 0.3f;
    gx += 0.4f; gy += 0.5f; gz += 0.6f;

    sendUserDatafloat6(1, fx, fy, fz, gx, gy, gz);
  }
}

static int addPeriodFunListFakeSensor(void) {
  anotc_telem_add_sensor_func(sendSensorImuData);
  return 0;
}

INIT_APP_EXPORT(addPeriodFunListFakeSensor);

#endif
