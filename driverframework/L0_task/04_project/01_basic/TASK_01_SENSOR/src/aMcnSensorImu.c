#include "aMcnSensorImu.h"
#include "floatConvert.h"

/* MCN topic declaration */
MCN_DECLARE(imu);
/* MCN topic definition */
MCN_DEFINE(imu, sizeof(sensorData_t));

/* MCN subscriber node */
static McnNode_t sensor_imu_sub_node = RT_NULL;

/* Echo function for sensor IMU data */
static int sensor_imu_echo(void* parameter) {
  sensorData_t sensor_data;
  
  if (mcn_copy_from_hub((McnHub*)parameter, &sensor_data) != RT_EOK) {
    return -1;
  }
  
  char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
  float_to_string(sensor_data.acc_filter.x, ax, sizeof(ax));
  float_to_string(sensor_data.acc_filter.y, ay, sizeof(ay));
  float_to_string(sensor_data.acc_filter.z, az, sizeof(az));
  float_to_string(sensor_data.gyro_filter.x, gx, sizeof(gx));
  float_to_string(sensor_data.gyro_filter.y, gy, sizeof(gy));
  float_to_string(sensor_data.gyro_filter.z, gz, sizeof(gz));
  
  rt_kprintf("[aMcnSensorImu] acc: %s, %s, %s, gyro: %s, %s, %s, ts: %lu\n", 
             ax, ay, az, gx, gy, gz, sensor_data.timestamp);
  
  return 0;
}

/* Initialize MCN sensor IMU */
static int mcnSensorImuInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(imu), sensor_imu_echo);
  if (result != RT_EOK) {
    rt_kprintf("[aMcnSensorImu] Failed to advertise imu topic: %d\n", result);
    return -1;
  }
  
  sensor_imu_sub_node = mcn_subscribe(MCN_HUB(imu), RT_NULL, RT_NULL);
  if (sensor_imu_sub_node == RT_NULL) {
    rt_kprintf("[aMcnSensorImu] Failed to subscribe to imu topic\n");
    return -1;
  }
  
  rt_kprintf("[aMcnSensorImu] Sensor IMU MCN initialized\n");
  return 0;
}

/* Publish sensor IMU data to MCN */
int mcnSensorImuPublish(const sensorData_t* sensor_data) {
  if (!sensor_data) {
    return -1;
  }
  
  return mcn_publish(MCN_HUB(imu), sensor_data);
}

/* Acquire sensor IMU data from MCN */
int mcnSensorImuAcquire(sensorData_t* sensor_data) {
  if (!sensor_data) {
    return -1;
  }
  
  if (sensor_imu_sub_node == RT_NULL) {
    return -1;
  }
  
  // if (mcn_poll(sensor_imu_sub_node)) {
  return mcn_copy(MCN_HUB(imu), sensor_imu_sub_node, sensor_data);
  // }
  
  // return -1;
}

INIT_COMPONENT_EXPORT(mcnSensorImuInit);

