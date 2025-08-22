#include "rtthread.h"
#include <rtdevice.h>
#include "imu.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"

/*task definition*/
#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

#define SENSORS_MPU6500_BUFF_LEN 14

static struct rt_thread task_tid_sensor_minifly;
static rt_uint8_t task_stack_sensor_minifly[THREAD_STACK_SIZE];
static rt_device_t dev_sensor_imu = RT_NULL;
/* queues for acc/gyro */
#define MQ_LEN 1
#define MQ_ITEM_SIZE_ACC (sizeof(Axis3f))
#define MQ_ITEM_SIZE_GYRO (sizeof(Axis3f))
static rt_uint8_t acc_pool[MQ_ITEM_SIZE_ACC * MQ_LEN] __attribute__((aligned(RT_ALIGN_SIZE)));
static rt_uint8_t gyro_pool[MQ_ITEM_SIZE_GYRO * MQ_LEN] __attribute__((aligned(RT_ALIGN_SIZE)));
static struct rt_messagequeue mq_acc;
static struct rt_messagequeue mq_gyro;

static void mq_overwrite(struct rt_messagequeue *mq, const void *msg, rt_size_t size) {
  if (rt_mq_send(mq, msg, size) != RT_EOK) {
    rt_mq_control(mq, RT_IPC_CMD_RESET, RT_NULL);
    rt_mq_send(mq, msg, size);
  }
}

static rt_err_t sensorMiniflyReadAcc(Axis3f *out) {
  if (!out) return -RT_EINVAL;
  return rt_mq_recv(&mq_acc, out, sizeof(Axis3f), 0) > 0 ? RT_EOK : -RT_ETIMEOUT;
}

static rt_err_t sensorMiniflyReadGyro(Axis3f *out) {
  if (!out) return -RT_EINVAL;
  return rt_mq_recv(&mq_gyro, out, sizeof(Axis3f), 0) > 0 ? RT_EOK : -RT_ETIMEOUT;
}

static void deviceInit(void) {
  rt_device_t dev_temp = RT_NULL;

#ifdef TASK_SENSOR_MINIFLY_IMU_NAME
  dev_temp = rt_device_find(TASK_SENSOR_MINIFLY_IMU_NAME);
#else
  dev_temp = rt_device_find(SENSOR_NAME_MPU6500_MINIFLY);
#endif
  if (dev_temp) {
    rt_device_open(dev_temp, RT_DEVICE_OFLAG_RDWR);
    dev_sensor_imu = dev_temp;
  }
}

static void rtosToolsInit(void) {
  /* init queues */
  rt_mq_init(&mq_acc, "mq_acc", acc_pool, MQ_ITEM_SIZE_ACC, sizeof(acc_pool), RT_IPC_FLAG_PRIO);
  rt_mq_init(&mq_gyro, "mq_gyr", gyro_pool, MQ_ITEM_SIZE_GYRO, sizeof(gyro_pool), RT_IPC_FLAG_PRIO);
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
        sensors_data = processAccGyroMeasurements(sensor_buffer);
        mq_overwrite(&mq_acc, &sensors_data.acc_filter, sizeof(sensors_data.acc_filter));
        mq_overwrite(&mq_gyro, &sensors_data.gyro_filter, sizeof(sensors_data.gyro_filter));

      } else {
        static int err_cnt = 0;
        if (++err_cnt % 100 == 0) {
          rt_kprintf("imu read fail %d\n", err_cnt);
        }
      }
    } else {
      static int not_found = 0;
      if (++not_found % 30 == 0) {
        rt_kprintf("imu device not found, retrying...\n");
      }
      rt_thread_mdelay(100);
    }
  }
}

void sensorsAcquire(sensorData_t *sensors) {
  sensorMiniflyReadGyro(&sensors->gyro_filter);
  sensorMiniflyReadAcc(&sensors->acc_filter);
}

static void task_thread_init(void) {
  rt_thread_init(&task_tid_sensor_minifly, "t_sensor6", sensor_minifly_thread_entry, RT_NULL, task_stack_sensor_minifly,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_minifly);
}

#ifdef PROJECT_MINIFLY_SENSOR
INIT_APP_EXPORT(task_thread_init);
#endif