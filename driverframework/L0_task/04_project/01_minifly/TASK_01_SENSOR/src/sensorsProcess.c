
#include "rtthread.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"
#include "rotation.h"
#include <math.h>
#include <string.h>

#define SENSORS_ACC_SCALE_SAMPLES 200

static sensorData_t sensors;

static Axis3f gyroBias;
static float accScale = 1;
static bool gyroBiasFound = false;
static float g_gyro_deg_per_lsb = (float)((2 * 2000.0) / 65536.0);
static float g_acc_g_per_lsb = (float)((2 * 16) / 65536.0);
static enum Rotation imu_mount_rotation = ROTATION_NEGATE_X;

static inline void gyroRemoveBiasRaw(Axis3f* out, Axis3i16* raw, const Axis3f* bias) {
  out->x = (float)raw->x - bias->x;
  out->y = (float)raw->y - bias->y;
  out->z = (float)raw->z - bias->z;
}

static inline void gyroApplyScale(Axis3f* raw) {
  raw->x = raw->x * g_gyro_deg_per_lsb;
  raw->y = raw->y * g_gyro_deg_per_lsb;
  raw->z = raw->z * g_gyro_deg_per_lsb;
}

static inline void gyroApplyRotation(enum Rotation rot, Axis3f* v) { rotation(rot, &v->x, &v->y, &v->z); }

static inline void accApplyScale(Axis3f* out, Axis3i16* raw) {
  out->x = (float)raw->x * g_acc_g_per_lsb / accScale;
  out->y = (float)raw->y * g_acc_g_per_lsb / accScale;
  out->z = (float)raw->z * g_acc_g_per_lsb / accScale;
}

static inline void accApplyRotation(enum Rotation rot, Axis3f* v) { rotation(rot, &v->x, &v->y, &v->z); }

static enum Rotation parseRotationFromString(const char* rot_str) {
  if (rot_str == RT_NULL) return ROTATION_NONE;
  if (strcmp(rot_str, "ROTATION_NONE") == 0) return ROTATION_NONE;
  if (strcmp(rot_str, "ROTATION_NEGATE_X") == 0) return ROTATION_NEGATE_X;
  if (strcmp(rot_str, "ROTATION_YAW_90") == 0) return ROTATION_YAW_90;
  if (strcmp(rot_str, "ROTATION_YAW_180") == 0) return ROTATION_YAW_180;
  if (strcmp(rot_str, "ROTATION_YAW_270") == 0) return ROTATION_YAW_270;
  if (strcmp(rot_str, "ROTATION_ROLL_180") == 0) return ROTATION_ROLL_180;
  if (strcmp(rot_str, "ROTATION_PITCH_180") == 0) return ROTATION_PITCH_180;
  if (strcmp(rot_str, "ROTATION_ROLL_180_YAW_180") == 0) return ROTATION_ROLL_180_YAW_180;
  if (strcmp(rot_str, "ROTATION_PITCH_180_ROLL_180") == 0) return ROTATION_PITCH_180_ROLL_180;
  if (strcmp(rot_str, "ROTATION_ROLL_180_NEGATE_X") == 0) return ROTATION_ROLL_180_NEGATE_X;
  if (strcmp(rot_str, "ROTATION_PITCH_180_NEGATE_X") == 0) return ROTATION_PITCH_180_NEGATE_X;
  return ROTATION_NONE;
}

static bool getAccScale(Axis3i16 accRaw) {
  if (!gyroBiasFound) {
    return false;
  }
  static float accScaleSum = 0;
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;

  if (!accBiasFound) {
    accScaleSum += sqrtf(powf(accRaw.x * g_acc_g_per_lsb, 2) + powf(accRaw.y * g_acc_g_per_lsb, 2) +
                         powf(accRaw.z * g_acc_g_per_lsb, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }
  return accBiasFound;
}

static inline void sensorsLoadRawFromBuffer(const uint8_t* buffer) {
#ifdef BSP_USING_BMI270
  sensors.acc_raw.x = (int16_t)((((int16_t)buffer[1]) << 8) | buffer[0]);
  sensors.acc_raw.y = (int16_t)((((int16_t)buffer[3]) << 8) | buffer[2]);
  sensors.acc_raw.z = (int16_t)((((int16_t)buffer[5]) << 8) | buffer[4]);
  sensors.gyro_raw.x = (int16_t)((((int16_t)buffer[7]) << 8) | buffer[6]);
  sensors.gyro_raw.y = (int16_t)((((int16_t)buffer[9]) << 8) | buffer[8]);
  sensors.gyro_raw.z = (int16_t)((((int16_t)buffer[11]) << 8) | buffer[10]);
#elif
  sensors.acc_raw.x = (int16_t)((((int16_t)buffer[2]) << 8) | buffer[3]);
  sensors.acc_raw.y = (int16_t)((((int16_t)buffer[0]) << 8) | buffer[1]);
  sensors.acc_raw.z = (int16_t)((((int16_t)buffer[4]) << 8) | buffer[5]);
  sensors.gyro_raw.x = (int16_t)((((int16_t)buffer[10]) << 8) | buffer[11]);
  sensors.gyro_raw.y = (int16_t)((((int16_t)buffer[8]) << 8) | buffer[9]);
  sensors.gyro_raw.z = (int16_t)((((int16_t)buffer[12]) << 8) | buffer[13]);
#endif
}

void initImuRotationDir(void) {
  imu_mount_rotation =
#ifdef PROJECT_MINIFLY_TASK_SENSOR_ROTATION
      parseRotationFromString(PROJECT_MINIFLY_TASK_SENSOR_ROTATION);
#else
      ROTATION_NEGATE_X;
#endif
}

sensorData_t processAccGyroMeasurements(const uint8_t* buffer) {
  sensorsLoadRawFromBuffer(buffer);

  gyroBiasFound = getGyroBias(sensors.gyro_raw, &gyroBias);
  gyroRemoveBiasRaw(&sensors.gyro_filter, &sensors.gyro_raw, &gyroBias);
  gyroApplyScale(&sensors.gyro_filter);
  gyroApplyRotation(imu_mount_rotation, &sensors.gyro_filter);
  applyAxis3fLpfGyro(&sensors.gyro_filter);

  getAccScale(sensors.acc_raw);
  accApplyScale(&sensors.acc_filter, &sensors.acc_raw);
  accApplyRotation(imu_mount_rotation, &sensors.acc_filter);
  applyAxis3fLpfAcc(&sensors.acc_filter);

  return sensors;
}

void sensorsProcess_set_lsb(float acc_g_per_lsb, float gyro_deg_per_lsb) {
  if (acc_g_per_lsb > 0.0f) {
    g_acc_g_per_lsb = acc_g_per_lsb;
  }
  if (gyro_deg_per_lsb > 0.0f) {
    g_gyro_deg_per_lsb = gyro_deg_per_lsb;
  }
}
