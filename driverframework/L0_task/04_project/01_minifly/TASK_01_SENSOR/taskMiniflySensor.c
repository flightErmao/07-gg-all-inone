#include "rtthread.h"
#include <rtdevice.h>
#include "imu.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"
#include "floatConvert.h"
#include "uMCN.h"

/*task definition*/
#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

#define SENSORS_MPU6500_BUFF_LEN 14

static struct rt_thread task_tid_sensor_minifly;
static rt_uint8_t task_stack_sensor_minifly[THREAD_STACK_SIZE];
static rt_device_t dev_sensor_imu = RT_NULL;

// MCN主题声明和定义
MCN_DECLARE(minifly_sensor_imu);
MCN_DEFINE(minifly_sensor_imu, sizeof(sensorData_t));

// MCN订阅节点
static McnNode_t sensor_sub_node = RT_NULL;

// 简单的echo函数，打印所有传感器值
static int sensor_imu_echo(void *parameter) {
  sensorData_t sensor_data;

  if (mcn_copy_from_hub((McnHub *)parameter, &sensor_data) != RT_EOK) {
    return -1;
  }

  char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
  float_to_string(sensor_data.acc_filter.x, ax, sizeof(ax));
  float_to_string(sensor_data.acc_filter.y, ay, sizeof(ay));
  float_to_string(sensor_data.acc_filter.z, az, sizeof(az));
  float_to_string(sensor_data.gyro_filter.x, gx, sizeof(gx));
  float_to_string(sensor_data.gyro_filter.y, gy, sizeof(gy));
  float_to_string(sensor_data.gyro_filter.z, gz, sizeof(gz));

  rt_kprintf("Sensor IMU Echo - acc: %s, %s, %s, gyro: %s, %s, %s, timestamp: %lu\n", ax, ay, az, gx, gy, gz,
             sensor_data.timestamp);

  return 0;
}

static void deviceInit(void) {
  rt_device_t dev_temp = RT_NULL;

#ifdef PROJECT_MINIFLY_TASK_SENSOR_IMU_NAME
  dev_temp = rt_device_find(PROJECT_MINIFLY_TASK_SENSOR_IMU_NAME);
#else
  dev_temp = rt_device_find(SENSOR_NAME_MPU6500_MINIFLY);
#endif
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_imu = dev_temp;
  }
}

static void rtosToolsInit(void) {
  /* 初始化MCN主题 */
  rt_err_t result = mcn_advertise(MCN_HUB(minifly_sensor_imu), sensor_imu_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise minifly_sensor_imu topic: %d\n", result);
  }

  /* 订阅MCN主题 */
  sensor_sub_node = mcn_subscribe(MCN_HUB(minifly_sensor_imu), RT_NULL, RT_NULL);
  if (sensor_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to minifly_sensor_imu topic\n");
  }
}

static void sensor_minifly_thread_entry(void *parameter) {
  deviceInit();
  rtosToolsInit();
  filterInitLpf2AccGyro();
  sensorsBiasObjInit();

  uint8_t sensor_buffer[SENSORS_MPU6500_BUFF_LEN] = {0};
  sensorData_t sensors_data = {0};

  while (1) {
    if (dev_sensor_imu) {
      int rb = rt_device_read(dev_sensor_imu, NULL, sensor_buffer, SENSORS_MPU6500_BUFF_LEN);
      if (rb == SENSORS_MPU6500_BUFF_LEN) {
        uint32_t timestamp = rt_tick_get();
        sensors_data = processAccGyroMeasurements(sensor_buffer);
        sensors_data.timestamp = timestamp;
        mcn_publish(MCN_HUB(minifly_sensor_imu), &sensors_data);
      } else {
        static int err_cnt = 0;
        if (++err_cnt % 100 == 0) {
          rt_kprintf("imu read fail %d\n", err_cnt);
        }
      }
    } else {
      static int not_found = 0;
      if (++not_found % 30 == 0) {
        rt_kprintf("imu read fail %d\n", not_found);
      }
      rt_thread_mdelay(100);
    }
  }
}

// 包装的sensorsAcquire函数，使用MCN操作
void sensorsAcquire(sensorData_t *sensors) {
  if (!sensors) return;

  // 检查是否有新的传感器数据
  if (mcn_poll(sensor_sub_node)) {
    // 复制最新的传感器数据
    mcn_copy(MCN_HUB(minifly_sensor_imu), sensor_sub_node, sensors);
  }
}

static int taskSensorThreadAutoStart(void) {
  rt_thread_init(&task_tid_sensor_minifly, "t_sensor6", sensor_minifly_thread_entry, RT_NULL, task_stack_sensor_minifly,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_minifly);
}

#ifdef PROJECT_MINIFLY_TASK_SENSOR_EN
INIT_APP_EXPORT(taskSensorThreadAutoStart);
#endif