#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"

#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)
#define SENSORS_DEG_PER_LSB_CFG MPU6500_DEG_PER_LSB_2000

#define MPU6500_G_PER_LSB_16 (float)((2 * 16) / 65536.0)
#define SENSORS_G_PER_LSB_CFG MPU6500_G_PER_LSB_16

#define SENSORS_ACC_SCALE_SAMPLES 200

static sensorData_t sensors;
static float accScale = 1;

static bool processAccScale(int16_t ax, int16_t ay, int16_t az) {
  static float accScaleSum = 0;
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;

  if (!accBiasFound) {
    accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) +
                         powf(az * SENSORS_G_PER_LSB_CFG, 2));
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
  int16_t ax = ((((int16_t)buffer[2]) << 8) | buffer[3]);
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

  sensors.gyro_filter.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro_filter.y = (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro_filter.z = (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
  applyAxis3fLpfGyro(&sensors.gyro_filter);

  sensors.acc_filter.x = -(ax)*SENSORS_G_PER_LSB_CFG / accScale;
  sensors.acc_filter.y = (ay)*SENSORS_G_PER_LSB_CFG / accScale;
  sensors.acc_filter.z = (az)*SENSORS_G_PER_LSB_CFG / accScale;
  applyAxis3fLpfAcc(&sensors.acc_filter);

  return sensors;
}
