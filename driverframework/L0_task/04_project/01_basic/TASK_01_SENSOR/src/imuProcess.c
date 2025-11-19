
#include "rtthread.h"
#include "filterLpf2p.h"
#include "filterNotch2p.h"
#include "imuProcess.h"
#include "rotation.h"
#include <math.h>
#include <string.h>
#include "aMlogSensorImu.h"
#include "timestamp.h"
#include "../../TASK_05_PARAM/inc/param.h"
#include "../../TASK_05_PARAM/inc/imuCaliParam.h"
#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
#include "rpmFilter.h"
#include "aMcnDshot.h"
#endif
#include "debugPin.h"
#ifndef DSHOT_DEVICE_NAME
#define DSHOT_DEVICE_NAME "dshot"
#endif
#define SENSORS_ACC_SCALE_SAMPLES 200

// #define IMUPROCESS_DEBUG

#define LOG_TAG "imu_process"
#ifdef IMUPROCESS_DEBUG
#define LOG_LVL LOG_LVL_DBG
#else
#define LOG_LVL LOG_LVL_WARNING
#endif
#include <ulog.h>

static sensorData_t sensors;

static Axis3f gyroBias = {0};
static Axis3f accBias = {0};
static float accScale = 1;
static bool gyroBiasFound = false;
static bool accBiasFound = false;
static float g_gyro_deg_per_lsb = (float)((2 * 2000.0) / 65536.0);
static float g_acc_g_per_lsb = (float)((2 * 16) / 65536.0);
static enum Rotation imu_mount_rotation = ROTATION_NEGATE_X;

// sensor_align_e from cmdImuCalGeneral.c
typedef enum {
  ALIGN_DEFAULT = 0,
  CW0_DEG = 1,
  CW90_DEG = 2,
  CW180_DEG = 3,
  CW270_DEG = 4,
  CW0_DEG_FLIP = 5,
  CW90_DEG_FLIP = 6,
  CW180_DEG_FLIP = 7,
  CW270_DEG_FLIP = 8,
} sensor_align_e;

// static enum Rotation sensor_align_to_rotation(uint8_t align) {
//   switch (align) {
//     case ALIGN_DEFAULT:
//     case CW0_DEG:
//       return ROTATION_NONE;
//     case CW90_DEG:
//       return ROTATION_YAW_90;
//     case CW180_DEG:
//       return ROTATION_YAW_180;
//     case CW270_DEG:
//       return ROTATION_YAW_270;
//     case CW0_DEG_FLIP:
//       return ROTATION_NEGATE_X;
//     case CW90_DEG_FLIP:
//       return ROTATION_ROLL_180_YAW_90;
//     case CW180_DEG_FLIP:
//       return ROTATION_ROLL_180;
//     case CW270_DEG_FLIP:
//       return ROTATION_ROLL_180_YAW_270;
//     default:
//       return ROTATION_NONE;
//   }
// }

static inline void gyroRemoveBiasRaw(Axis3f* out, Axis3i16* raw, const Axis3f* bias) {
  out->x = (float)raw->x - bias->x;
  out->y = (float)raw->y - bias->y;
  out->z = (float)raw->z - bias->z;
}

static inline void gyroApplyScale(Axis3f* raw) {
  raw->x = raw->x * g_gyro_deg_per_lsb;
  raw->y = -raw->y * g_gyro_deg_per_lsb;
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

// Removed getAccScale - acc scale is now handled via calibration parameters

static inline void sensorsLoadRawFromBuffer(const uint8_t* buffer) {
  sensors.acc_raw.x = (int16_t)((((int16_t)buffer[1]) << 8) | buffer[0]);
  sensors.acc_raw.y = (int16_t)((((int16_t)buffer[3]) << 8) | buffer[2]);
  sensors.acc_raw.z = (int16_t)((((int16_t)buffer[5]) << 8) | buffer[4]);
  sensors.gyro_raw.x = (int16_t)((((int16_t)buffer[7]) << 8) | buffer[6]);
  sensors.gyro_raw.y = (int16_t)((((int16_t)buffer[9]) << 8) | buffer[8]);
  sensors.gyro_raw.z = (int16_t)((((int16_t)buffer[11]) << 8) | buffer[10]);
}

void initImuRotationDir(void) {
  // Load calibration parameters from parameter table
  float gyro_bias[3] = {0.0f};
  float acc_bias[3] = {0.0f};
  // uint8_t orientation = ALIGN_DEFAULT;

  rt_err_t ret;

  // Load gyro bias
  ret = getParam(IMU_CALI_PARAM_GYRO_BIAS, gyro_bias, sizeof(gyro_bias));
  if (ret == RT_EOK) {
    gyroBias.x = gyro_bias[0];
    gyroBias.y = gyro_bias[1];
    gyroBias.z = gyro_bias[2];
    gyroBiasFound = !(gyro_bias[0] == 0.0f && gyro_bias[1] == 0.0f && gyro_bias[2] == 0.0f);
    if (gyroBiasFound) {
      LOG_I("Gyro bias loaded: [%.6f, %.6f, %.6f]", gyroBias.x, gyroBias.y, gyroBias.z);
    }
  } else {
    LOG_W("Failed to load gyro bias: %d", ret);
  }

  // Load acc bias
  ret = getParam(IMU_CALI_PARAM_ACC_BIAS, acc_bias, sizeof(acc_bias));
  if (ret == RT_EOK) {
    accBias.x = acc_bias[0];
    accBias.y = acc_bias[1];
    accBias.z = acc_bias[2];
    accBiasFound = !(acc_bias[0] == 0.0f && acc_bias[1] == 0.0f && acc_bias[2] == 0.0f);
    if (accBiasFound) {
      LOG_I("Acc bias loaded: [%.6f, %.6f, %.6f]", accBias.x, accBias.y, accBias.z);
    }
  } else {
    LOG_W("Failed to load acc bias: %d", ret);
  }

  //   // Load orientation
  //   ret = getParam(IMU_CALI_PARAM_ORIENTATION, &orientation, sizeof(orientation));
  //   if (ret == RT_EOK) {
  //     imu_mount_rotation = sensor_align_to_rotation(orientation);
  //     LOG_I("IMU orientation loaded: %d -> rotation %d", orientation, imu_mount_rotation);
  //   } else {
  //     // Fallback to compile-time or default rotation
  //     imu_mount_rotation =
  // #ifdef PROJECT_MINIFLY_TASK_SENSOR_ROTATION
  //         parseRotationFromString(PROJECT_MINIFLY_TASK_SENSOR_ROTATION);
  // #else
  //         ROTATION_NEGATE_X;
  // #endif
  //     LOG_W("Failed to load orientation: %d, using default", ret);
  //   }

  imu_mount_rotation = parseRotationFromString(PROJECT_MINIFLY_TASK_SENSOR_ROTATION);
}

static void generateAngularAccel(void) {
  static uint32_t last_timestamp = 0;
  static Axis3f last_gyro_filter = {0};
  float dt = 0.0f;
  uint32_t timestamp = timestamp_micros();
#ifdef PROJECT_MINIFLY_TASK_SENSOR_DEBUGPIN_MLOG_EN
  // DEBUG_PIN_DEBUG0_TOGGLE();
#endif
  if (last_timestamp != 0) {
    dt = (timestamp - last_timestamp) / 1000000.0f;
    dt = fmaxf(fminf(dt, 0.1f), 0.0005f);

    Axis3f angular_accel = {0};
    angular_accel.x = (sensors.gyro_filter.x - last_gyro_filter.x) / dt;
    angular_accel.y = (sensors.gyro_filter.y - last_gyro_filter.y) / dt;
    angular_accel.z = (sensors.gyro_filter.z - last_gyro_filter.z) / dt;

    applyAxis3fLpfAngularAccel(&angular_accel);

    sensors.angular_accel = angular_accel;
  }

  static uint16_t count = 0;
  count++;
  if (count == 1000) {
    LOG_D("%d", timestamp - last_timestamp);
    LOG_D("dt: %.6f", dt);
    LOG_D("Gyro: %.2f %.2f %.2f | Acc: %.2f %.2f %.2f", sensors.gyro_filter.x, sensors.gyro_filter.y,
          sensors.gyro_filter.z, sensors.acc_filter.x, sensors.acc_filter.y, sensors.acc_filter.z);
    count = 0;
  }

  last_gyro_filter = sensors.gyro_filter;
  last_timestamp = timestamp;
}

static void dealWithGyroData(void) {
  if (gyroBiasFound) {
    gyroRemoveBiasRaw(&sensors.gyro_filter, &sensors.gyro_raw, &gyroBias);
  } else {
    sensors.gyro_filter.x = (float)sensors.gyro_raw.x;
    sensors.gyro_filter.y = (float)sensors.gyro_raw.y;
    sensors.gyro_filter.z = (float)sensors.gyro_raw.z;
  }
  gyroApplyScale(&sensors.gyro_filter);
  gyroApplyRotation(imu_mount_rotation, &sensors.gyro_filter);
  mlogImuCopyGyroData(&sensors.gyro_filter, RT_NULL);
#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
  rpm_data_bus_t rpm_msg __attribute__((aligned(4))) = {0};
  mcnRpmDataAcquire(&rpm_msg);
  rpmFilter((float*)&sensors.gyro_filter, rpm_msg.rpm, (float*)&sensors.gyro_filter);
#endif
  applyAxis3fNotchGyro(&sensors.gyro_filter);
  applyAxis3fLpfGyro(&sensors.gyro_filter);
  mlogImuCopyGyroData(RT_NULL, &sensors.gyro_filter);
  generateAngularAccel();
}

static void dealWithAccData(void) {
  // Apply scale first
  accApplyScale(&sensors.acc_filter, &sensors.acc_raw);
  // Apply calibration bias from parameter table if available
  if (accBiasFound) {
    sensors.acc_filter.x -= accBias.x;
    sensors.acc_filter.y -= accBias.y;
    sensors.acc_filter.z -= accBias.z;
  }
  accApplyRotation(imu_mount_rotation, &sensors.acc_filter);
  mlogImuCopyAccData(&sensors.acc_filter, RT_NULL);
  applyAxis3fNotchAcc(&sensors.acc_filter);
  applyAxis3fLpfAcc(&sensors.acc_filter);
  mlogImuCopyAccData(RT_NULL, &sensors.acc_filter);
}

sensorData_t processAccGyroMeasurements(const uint8_t* buffer) {
  sensorsLoadRawFromBuffer(buffer);
  dealWithGyroData();
  dealWithAccData();
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
