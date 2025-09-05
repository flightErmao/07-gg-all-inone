#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"
#include <math.h>

static float g_gyro_deg_per_lsb = (float)((2 * 2000.0) / 65536.0);
static float g_acc_g_per_lsb = (float)((2 * 16) / 65536.0);

#define SENSORS_ACC_SCALE_SAMPLES 200

static sensorData_t sensors;
static float accScale = 1;

static bool processAccScale(int16_t ax, int16_t ay, int16_t az) {
  static float accScaleSum = 0;
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;

  if (!accBiasFound) {
    accScaleSum += sqrtf(powf(ax * g_acc_g_per_lsb, 2) + powf(ay * g_acc_g_per_lsb, 2) + powf(az * g_acc_g_per_lsb, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }
  return accBiasFound;
}

sensorData_t processAccGyroMeasurements(const uint8_t* buffer) {
  static Axis3f gyroBias;
  static bool gyroBiasFound = false;

  int16_t ay = (((int16_t)buffer[0]) << 8) | buffer[1];
  int16_t ax = (((int16_t)buffer[2]) << 8) | buffer[3];
  int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];
  int16_t gy = (((int16_t)buffer[8]) << 8) | buffer[9];
  int16_t gx = (((int16_t)buffer[10]) << 8) | buffer[11];
  int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];

  sensors.acc_raw.x = ax;
  sensors.acc_raw.y = ay;
  sensors.acc_raw.z = az;
  sensors.gyro_raw.x = gx;
  sensors.gyro_raw.y = gy;
  sensors.gyro_raw.z = gz;

  gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);

  if (gyroBiasFound) {
    processAccScale(ax, ay, az);
  }

  sensors.gyro_filter.x = -(gx - gyroBias.x) * g_gyro_deg_per_lsb;
  sensors.gyro_filter.y = (gy - gyroBias.y) * g_gyro_deg_per_lsb;
  sensors.gyro_filter.z = (gz - gyroBias.z) * g_gyro_deg_per_lsb;
  applyAxis3fLpfGyro(&sensors.gyro_filter);

  sensors.acc_filter.x = -(ax)*g_acc_g_per_lsb / accScale;
  sensors.acc_filter.y = (ay)*g_acc_g_per_lsb / accScale;
  sensors.acc_filter.z = (az)*g_acc_g_per_lsb / accScale;
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
