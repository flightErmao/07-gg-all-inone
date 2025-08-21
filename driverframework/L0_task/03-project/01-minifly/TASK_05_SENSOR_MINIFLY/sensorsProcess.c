#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "config.h"
#include "config_param.h"
#include "ledseq.h"
#include "mpu6500.h"
#include "sensors.h"
#include "ak8963.h"
#include "bmp280.h"
#include "spl06.h"
#include "filters.h"
#include "FreeRTOS.h"
#include "task.h"

#define SENSORS_GYRO_FS_CFG MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6500_G_PER_LSB_16


#define GYRO_VARIANCE_BASE 4000
#define SENSORS_ACC_SCALE_SAMPLES 200

#define SENSORS_MPU6500_BUFF_LEN 14

#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30

static Axis3f gyroBias;
static bool gyroBiasFound = false;

static float accScaleSum = 0;
static float accScale = 1;

static bool isInit = false;
static sensorData_t sensors;

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static bool isMPUPresent = false;

static bool processAccScale(int16_t ax, int16_t ay, int16_t az) {
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

void filterInitLpf2AccGyro(void)
{
	for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
}

void processAccGyroMeasurements(const uint8_t* buffer) {
  int16_t ay = (((int16_t)buffer[0]) << 8) | buffer[1];
  int16_t ax = ((((int16_t)buffer[2]) << 8) | buffer[3]);
  int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];
  int16_t gy = (((int16_t)buffer[8]) << 8) | buffer[9];
  int16_t gx = (((int16_t)buffer[10]) << 8) | buffer[11];
  int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];

  accRaw.x = ax;
  accRaw.y = ay;
  accRaw.z = az;
  gyroRaw.x = gx - gyroBias.x;
  gyroRaw.y = gy - gyroBias.y;
  gyroRaw.z = gz - gyroBias.z;

  gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);

  if (gyroBiasFound) {
    processAccScale(ax, ay, az);
  }

  sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro.y = (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro.z = (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
  applyAxis3fLpf(gyroLpf, &sensors.gyro);

  sensors.acc.x = -(ax)*SENSORS_G_PER_LSB_CFG / accScale;
  sensors.acc.y = (ay)*SENSORS_G_PER_LSB_CFG / accScale;
  sensors.acc.z = (az)*SENSORS_G_PER_LSB_CFG / accScale;

  applyAxis3fLpf(accLpf, &sensors.acc);
}

bool sensorsAreCalibrated() { return gyroBiasFound; }

bool getIsMPU9250Present(void) {
  bool value = isMPUPresent;
  return value;
}

