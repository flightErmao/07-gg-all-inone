#include "rtthread.h"
#include <rtdevice.h>
#include "imu.h"
#include "biasGyro.h"
#include "filterLpf2p.h"
#include "sensorsProcess.h"
#include "floatConvert.h"
#include "uMCN.h"
#include "rtconfig.h"

#ifdef TASK_TOOL_02_SD_MLOG
#include "mlog.h"
#include "taskMlog.h"
#endif

/*task definition*/
#define THREAD_PRIORITY 4
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

#define SENSORS_MPU6500_BUFF_LEN 14

static struct rt_thread task_tid_sensor_minifly;
static rt_uint8_t task_stack_sensor_minifly[THREAD_STACK_SIZE];
static rt_device_t dev_sensor_imu = RT_NULL;
#ifdef TASK_TOOL_02_SD_MLOG
static uint8_t mlog_push_en = 0;
#endif

MCN_DECLARE(imu);
MCN_DEFINE(imu, sizeof(sensorData_t));

static McnNode_t sensor_sub_node = RT_NULL;

#ifdef PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
#define SENSOR_EVENT_FLAG_TRIGGER (1u << 0)
static struct rt_event sensor_event;
static rt_timer_t sensor_timer = RT_NULL;
static void sensor_timer_cb(void *parameter) {
  RT_UNUSED(parameter);
  rt_event_send(&sensor_event, SENSOR_EVENT_FLAG_TRIGGER);
}
#endif

#ifdef TASK_TOOL_02_SD_MLOG
/* Mlog bus definition for sensor data */
static mlog_elem_t Minifly_Sensor_IMU_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(acc_raw, MLOG_INT16, 3),
    MLOG_ELEMENT_VEC(gyro_raw, MLOG_INT16, 3),
    MLOG_ELEMENT_VEC(acc_filter, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filter, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(Minifly_Sensor_IMU, Minifly_Sensor_IMU_Elems);

static int Minifly_Sensor_IMU_ID = -1;
#endif

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
    static imu_dev_t imu_ptr = RT_NULL;
    imu_ptr = (imu_dev_t)dev_temp;
    if (imu_ptr && imu_ptr->ops) {
      float acc_g_per_lsb = imu_ptr->config.acc_scale_factor;      // g/lsb
      float gyro_deg_per_lsb = imu_ptr->config.gyro_scale_factor;  // dps/lsb
      sensorsProcess_set_lsb(acc_g_per_lsb, gyro_deg_per_lsb);
    }
  }
}

static void mlogInit(void) {
#ifdef TASK_TOOL_02_SD_MLOG
  /* Initialize mlog bus ID for sensor data */
  Minifly_Sensor_IMU_ID = mlog_get_bus_id("Minifly_Sensor_IMU");
  if (Minifly_Sensor_IMU_ID < 0) {
    rt_kprintf("Failed to get mlog bus ID for Minifly_Sensor_IMU\n");
  } else {
    rt_kprintf("Minifly_Sensor_IMU mlog bus ID: %d\n", Minifly_Sensor_IMU_ID);
  }
#endif
}

static void rtosToolsInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(imu), sensor_imu_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise imu topic: %d\n", result);
  }

  sensor_sub_node = mcn_subscribe(MCN_HUB(imu), RT_NULL, RT_NULL);
  if (sensor_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to imu topic\n");
  }

#ifdef PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
  rt_event_init(&sensor_event, "sns_evt", RT_IPC_FLAG_PRIO);
  if (sensor_timer == RT_NULL) {
    /* Default 10ms period; adjust if needed */
    rt_tick_t period_ticks = rt_tick_from_millisecond(1);
    sensor_timer = rt_timer_create("sns_tmr", sensor_timer_cb, RT_NULL, period_ticks,
                                   RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (sensor_timer) {
      rt_timer_start(sensor_timer);
    } else {
      rt_kprintf("Failed to create sensor timer\n");
    }
  }
#endif
}

#ifdef TASK_TOOL_02_SD_MLOG
static void mlogStartCb(void) { mlog_push_en = 1; }
#endif

static void sensor_minifly_thread_entry(void *parameter) {
  deviceInit();
  rtosToolsInit();
  filterInitLpf2AccGyro();
  sensorsBiasObjInit();
  initImuRotationDir();
  mlogInit();
#ifdef TASK_TOOL_02_SD_MLOG
  mlog_register_callback(MLOG_CB_START, mlogStartCb);
#endif

  uint8_t sensor_buffer[SENSORS_MPU6500_BUFF_LEN] = {0};
  sensorData_t sensors_data = {0};

  while (1) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
    {
      rt_uint32_t received = 0;
      rt_event_recv(&sensor_event, SENSOR_EVENT_FLAG_TRIGGER, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                    RT_WAITING_FOREVER, &received);
    }
#endif
    if (dev_sensor_imu) {
      int rb = rt_device_read(dev_sensor_imu, 0, sensor_buffer, SENSORS_MPU6500_BUFF_LEN);
      if (rb == SENSORS_MPU6500_BUFF_LEN) {
        uint32_t timestamp = rt_tick_get();
        sensors_data = processAccGyroMeasurements(sensor_buffer);
        sensors_data.timestamp = timestamp;
        mcn_publish(MCN_HUB(imu), &sensors_data);

#ifdef TASK_TOOL_02_SD_MLOG
        if (Minifly_Sensor_IMU_ID >= 0 && mlog_push_en) {
          mlog_push_msg((uint8_t*)&sensors_data, Minifly_Sensor_IMU_ID, sizeof(sensorData_t));
        }
#endif
      } else {
        static int err_cnt = 0;
        if (++err_cnt % 100 == 0) {
          rt_kprintf("imu read fail %d\n", err_cnt);
        }
      }
    } else {
      static int not_found = 0;
      if (++not_found % 30 == 0) {
        rt_kprintf("imu not found\n");
      }
      rt_thread_mdelay(100);
    }
  }
}

void sensorsAcquire(sensorData_t *sensors) {
  if (!sensors) return;
  if (mcn_poll(sensor_sub_node)) {
    mcn_copy(MCN_HUB(imu), sensor_sub_node, sensors);
  }
}

static int taskSensorThreadAutoStart(void) {
  rt_thread_init(&task_tid_sensor_minifly, "imu", sensor_minifly_thread_entry, RT_NULL, task_stack_sensor_minifly,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_sensor_minifly);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_SENSOR_EN
INIT_APP_EXPORT(taskSensorThreadAutoStart);
#endif