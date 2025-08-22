#include "rtthread.h"
#include <rtdevice.h>
#include "imu.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"
#include "floatConvert.h"

/*task definition*/
#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

#define SENSORS_MPU6500_BUFF_LEN 14

static struct rt_thread task_tid_sensor_minifly;
static rt_uint8_t task_stack_sensor_minifly[THREAD_STACK_SIZE];
static rt_device_t dev_sensor_imu = RT_NULL;
/* queues for acc/gyro */
#define MQ_LEN 2  // 增加队列长度，避免数据覆盖
#define MQ_ITEM_SIZE_ACC (sizeof(Axis3f))
#define MQ_ITEM_SIZE_GYRO (sizeof(Axis3f))

// 确保内存池大小足够且对齐
static rt_uint8_t acc_pool[MQ_ITEM_SIZE_ACC * MQ_LEN] __attribute__((aligned(RT_ALIGN_SIZE)));
static rt_uint8_t gyro_pool[MQ_ITEM_SIZE_GYRO * MQ_LEN] __attribute__((aligned(RT_ALIGN_SIZE)));

static struct rt_messagequeue mq_acc;
static struct rt_messagequeue mq_gyro;

// 优化的队列覆盖函数
static void mq_overwrite(struct rt_messagequeue *mq, const void *msg, rt_size_t size) {
  rt_err_t result = rt_mq_send(mq, msg, size);
  if (result != RT_EOK) {
    // 如果队列满，先清空队列再发送
    rt_mq_control(mq, RT_IPC_CMD_RESET, RT_NULL);
    result = rt_mq_send(mq, msg, size);
    if (result != RT_EOK) {
      rt_kprintf("mq_overwrite failed: %d\n", result);
    } else {
      // 记录队列重置事件
      static int reset_cnt = 0;
      if (++reset_cnt % 50 == 0) {
        rt_kprintf("Queue reset count: %d\n", reset_cnt);
      }
    }
  }
}

static rt_err_t sensorMiniflyReadAcc(Axis3f *out) {
  if (!out) return -RT_EINVAL;

  rt_size_t recv_size = rt_mq_recv(&mq_acc, out, sizeof(Axis3f), 10);  // 10ms超时

  // 检查返回值：如果大于0表示成功接收，如果等于0表示超时，如果小于0表示错误
  if (recv_size > 0) {
    if (recv_size == sizeof(Axis3f)) {
      return RT_EOK;
    } else {
      rt_kprintf("Read acc data size mismatch: expected %d, got %d\n", sizeof(Axis3f), recv_size);
      return -RT_ERROR;
    }
  } else if (recv_size == 0) {
    return -RT_ETIMEOUT;
  } else {
    // 负值表示错误码
    rt_kprintf("Read acc data failed with error: %d\n", (int)recv_size);
    return (rt_err_t)recv_size;
  }
}

static rt_err_t sensorMiniflyReadGyro(Axis3f *out) {
  if (!out) return -RT_EINVAL;

  rt_size_t recv_size = rt_mq_recv(&mq_gyro, out, sizeof(Axis3f), 10);  // 10ms超时

  // 检查返回值：如果大于0表示成功接收，如果等于0表示超时，如果小于0表示错误
  if (recv_size > 0) {
    if (recv_size == sizeof(Axis3f)) {
      return RT_EOK;
    } else {
      rt_kprintf("Read gyro data size mismatch: expected %d, got %d\n", sizeof(Axis3f), recv_size);
      return -RT_ERROR;
    }
  } else if (recv_size == 0) {
    return -RT_ETIMEOUT;
  } else {
    // 负值表示错误码
    rt_kprintf("Read gyro data failed with error: %d\n", (int)recv_size);
    return (rt_err_t)recv_size;
  }
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
  rt_err_t result;

  // 参考 demo 的初始化方式，确保参数正确
  result = rt_mq_init(&mq_acc,
                      "mq_acc",           // 队列名称
                      &acc_pool[0],       // 内存池指向acc_pool
                      MQ_ITEM_SIZE_ACC,   // 每个消息的大小
                      sizeof(acc_pool),   // 内存池的大小
                      RT_IPC_FLAG_PRIO);  // 优先级标志
  if (result != RT_EOK) {
    rt_kprintf("Failed to init mq_acc: %d\n", result);
  } else {
    rt_kprintf("mq_acc initialized: item_size=%d, pool_size=%d\n", MQ_ITEM_SIZE_ACC, sizeof(acc_pool));
  }

  result = rt_mq_init(&mq_gyro,
                      "mq_gyr",           // 队列名称
                      &gyro_pool[0],      // 内存池指向gyro_pool
                      MQ_ITEM_SIZE_GYRO,  // 每个消息的大小
                      sizeof(gyro_pool),  // 内存池的大小
                      RT_IPC_FLAG_PRIO);  // 优先级标志
  if (result != RT_EOK) {
    rt_kprintf("Failed to init mq_gyro: %d\n", result);
  } else {
    rt_kprintf("mq_gyro initialized: item_size=%d, pool_size=%d\n", MQ_ITEM_SIZE_GYRO, sizeof(gyro_pool));
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
        sensors_data = processAccGyroMeasurements(sensor_buffer);

        // 调试：打印原始数据
        static int debug_cnt = 0;
        if (++debug_cnt % 100 == 0) {
          char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
          float_to_string(sensors_data.acc_filter.x, ax, sizeof(ax));
          float_to_string(sensors_data.acc_filter.y, ay, sizeof(ay));
          float_to_string(sensors_data.acc_filter.z, az, sizeof(az));
          float_to_string(sensors_data.gyro_filter.x, gx, sizeof(gx));
          float_to_string(sensors_data.gyro_filter.y, gy, sizeof(gy));
          float_to_string(sensors_data.gyro_filter.z, gz, sizeof(gz));
          rt_kprintf("Raw sensor data - acc: %s, %s, %s, gyro: %s, %s, %s\n", ax, ay, az, gx, gy, gz);
        }

        // 发送数据到队列
        mq_overwrite(&mq_acc, &sensors_data.acc_filter, sizeof(sensors_data.acc_filter));
        mq_overwrite(&mq_gyro, &sensors_data.gyro_filter, sizeof(sensors_data.gyro_filter));

        // 调试：打印队列状态和性能信息
        static int queue_debug_cnt = 0;
        if (++queue_debug_cnt % 200 == 0) {
          rt_kprintf("Queue status - acc: %d bytes sent, gyro: %d bytes sent, total: %d\n",
                     sizeof(sensors_data.acc_filter), sizeof(sensors_data.gyro_filter), queue_debug_cnt);
        }

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
  if (!sensors) return;

  rt_err_t gyro_result = sensorMiniflyReadGyro(&sensors->gyro_filter);
  rt_err_t acc_result = sensorMiniflyReadAcc(&sensors->acc_filter);

  // 调试：打印读取结果
  static int read_debug_cnt = 0;
  if (++read_debug_cnt % 100 == 0) {
    if (gyro_result == RT_EOK && acc_result == RT_EOK) {
      char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
      float_to_string(sensors->acc_filter.x, ax, sizeof(ax));
      float_to_string(sensors->acc_filter.y, ay, sizeof(ay));
      float_to_string(sensors->acc_filter.z, az, sizeof(az));
      float_to_string(sensors->gyro_filter.x, gx, sizeof(gx));
      float_to_string(sensors->gyro_filter.y, gy, sizeof(gy));
      float_to_string(sensors->gyro_filter.z, gz, sizeof(gz));
      rt_kprintf("Queue read success - acc: %s, %s, %s, gyro: %s, %s, %s\n", ax, ay, az, gx, gy, gz);
    } else {
      rt_kprintf("Queue read failed - gyro: %d, acc: %d\n", gyro_result, acc_result);
    }
  }
}

static void task_thread_init(void) {
  rt_thread_init(&task_tid_sensor_minifly, "t_sensor6", sensor_minifly_thread_entry, RT_NULL, task_stack_sensor_minifly,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_minifly);
}

#ifdef PROJECT_MINIFLY_SENSOR
INIT_APP_EXPORT(task_thread_init);
#endif