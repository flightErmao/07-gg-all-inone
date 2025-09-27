#include "rtthread.h"
#include <rtdevice.h>

#ifdef TASK_TOOL_02_SD_MLOG_TEST_THREAD

#ifdef PKG_USING_UMCN
#include "uMCN.h"
#endif

#include "rtconfig.h"
#include "mlog.h"

/* Task definition */
#define MLOG_TEST_THREAD_PRIORITY 5
#define MLOG_TEST_THREAD_STACK_SIZE 2048
#define MLOG_TEST_THREAD_TIMESLICE 5

/* Test data generation frequency control */
#ifndef MLOG_TEST_02_SD_MLOG_FREQ_HZ
#define MLOG_TEST_02_SD_MLOG_FREQ_HZ 100  // Default 100Hz
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

typedef union {
  struct {
    int16_t x;
    int16_t y;
    int16_t z;
  };
  int16_t axis[3];
} Axis3i16;

typedef union {
  struct {
    int32_t x;
    int32_t y;
    int32_t z;
  };
  int32_t axis[3];
} Axis3i32;

typedef union {
  struct {
    int64_t x;
    int64_t y;
    int64_t z;
  };
  int64_t axis[3];
} Axis3i64;

typedef union {
  struct {
    float x;
    float y;
    float z;
  };
  float axis[3];
} Axis3f;

typedef struct {
  uint32_t timestamp;
  Axis3i16 acc_raw;
  Axis3i16 gyro_raw;
  Axis3f acc_filter;
  Axis3f gyro_filter;
} sensorData_t;

// 恢复GCC诊断设置
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

static struct rt_thread task_tid_mlog_test;
static rt_uint8_t task_stack_mlog_test[MLOG_TEST_THREAD_STACK_SIZE];
static rt_bool_t mlog_test_running = RT_FALSE;
static rt_bool_t mlog_test_thread_created = RT_FALSE;
static uint8_t mlog_push_en = 0;

/* MCN hub for test data */
#ifdef PKG_USING_UMCN
MCN_DECLARE(mlog_test_data);
MCN_DEFINE(mlog_test_data, sizeof(sensorData_t));

static McnNode_t test_sub_node = RT_NULL;
#endif

/* Timer trigger support */
#define MLOG_TEST_EVENT_FLAG_TRIGGER (1u << 0)
static struct rt_event mlog_test_event;
static rt_timer_t mlog_test_timer = RT_NULL;

static void mlog_test_timer_cb(void *parameter) {
  RT_UNUSED(parameter);
  rt_event_send(&mlog_test_event, MLOG_TEST_EVENT_FLAG_TRIGGER);
}

/* Mlog bus definition for test data - same as sensor data */
static mlog_elem_t Mlog_Test_Data_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),         MLOG_ELEMENT_VEC(acc_raw, MLOG_INT16, 3),
    MLOG_ELEMENT_VEC(gyro_raw, MLOG_INT16, 3),    MLOG_ELEMENT_VEC(acc_filter, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filter, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(Mlog_Test_Data, Mlog_Test_Data_Elems);

static int Mlog_Test_Data_ID = -1;

/* Test data generation variables */
static int16_t test_counter = 0;
static rt_bool_t test_increment = RT_TRUE;
static const int16_t test_max_value = 1000;
static const int16_t test_min_value = -1000;

#ifdef PKG_USING_UMCN
static int mlog_test_echo(void *parameter) {
  sensorData_t test_data;

  if (mcn_copy_from_hub((McnHub *)parameter, &test_data) != RT_EOK) {
    return -1;
  }

  char ax[16], ay[16], az[16], gx[16], gy[16], gz[16];
  rt_snprintf(ax, sizeof(ax), "%.2f", (float)test_data.acc_raw.x);
  rt_snprintf(ay, sizeof(ay), "%.2f", (float)test_data.acc_raw.y);
  rt_snprintf(az, sizeof(az), "%.2f", (float)test_data.acc_raw.z);
  rt_snprintf(gx, sizeof(gx), "%.2f", (float)test_data.gyro_raw.x);
  rt_snprintf(gy, sizeof(gy), "%.2f", (float)test_data.gyro_raw.y);
  rt_snprintf(gz, sizeof(gz), "%.2f", (float)test_data.gyro_raw.z);

  rt_kprintf("Mlog Test Echo - acc_raw: %s, %s, %s, gyro_raw: %s, %s, %s, timestamp: %lu\n", ax, ay, az, gx, gy, gz,
             test_data.timestamp);

  return 0;
}
#endif

static void mlog_test_init(void) {
  /* Initialize mlog bus ID for test data */
  Mlog_Test_Data_ID = mlog_get_bus_id("Mlog_Test_Data");
  if (Mlog_Test_Data_ID < 0) {
    rt_kprintf("Failed to get mlog bus ID for Mlog_Test_Data\n");
  } else {
    rt_kprintf("Mlog_Test_Data mlog bus ID: %d\n", Mlog_Test_Data_ID);
  }
}

static void mlog_test_rtos_init(void) {
#ifdef PKG_USING_UMCN
  rt_err_t result = mcn_advertise(MCN_HUB(mlog_test_data), mlog_test_echo);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise mlog_test_data topic: %d\n", result);
  }

  test_sub_node = mcn_subscribe(MCN_HUB(mlog_test_data), RT_NULL, RT_NULL);
  if (test_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to mlog_test_data topic\n");
  }
#endif

  /* Initialize event and timer */
  rt_event_init(&mlog_test_event, "mlog_test_evt", RT_IPC_FLAG_PRIO);

  /* Calculate timer period based on frequency */
  rt_tick_t period_ticks = rt_tick_from_millisecond(1000 / MLOG_TEST_02_SD_MLOG_FREQ_HZ);
  mlog_test_timer = rt_timer_create("mlog_test_tmr", mlog_test_timer_cb, RT_NULL, period_ticks,
                                    RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
  if (mlog_test_timer == RT_NULL) {
    rt_kprintf("Failed to create mlog test timer\n");
  }
}

static void mlog_test_start_cb(void) {
  mlog_push_en = 1;
  rt_kprintf("Mlog test data logging started\n");
}

static void mlog_test_stop_cb(void) {
  mlog_push_en = 0;
  rt_kprintf("Mlog test data logging stopped\n");
}

static void generate_test_data(sensorData_t *data) {
  /* Generate simple increment/decrement pattern */
  if (test_increment) {
    test_counter += 10;
    if (test_counter >= test_max_value) {
      test_increment = RT_FALSE;
    }
  } else {
    test_counter -= 10;
    if (test_counter <= test_min_value) {
      test_increment = RT_TRUE;
    }
  }

  /* Fill test data */
  data->timestamp = rt_tick_get();

  /* Raw data - simple pattern */
  data->acc_raw.x = test_counter;
  data->acc_raw.y = test_counter / 2;
  data->acc_raw.z = test_counter / 3;

  data->gyro_raw.x = -test_counter;
  data->gyro_raw.y = -test_counter / 2;
  data->gyro_raw.z = -test_counter / 3;

  /* Filtered data - convert to float */
  data->acc_filter.x = (float)data->acc_raw.x;
  data->acc_filter.y = (float)data->acc_raw.y;
  data->acc_filter.z = (float)data->acc_raw.z;

  data->gyro_filter.x = (float)data->gyro_raw.x;
  data->gyro_filter.y = (float)data->gyro_raw.y;
  data->gyro_filter.z = (float)data->gyro_raw.z;
}

static void mlog_test_thread_entry(void *parameter) {
  RT_UNUSED(parameter);

  mlog_test_rtos_init();
  mlog_test_init();
  mlog_register_callback(MLOG_CB_START, mlog_test_start_cb);
  mlog_register_callback(MLOG_CB_STOP, mlog_test_stop_cb);

  sensorData_t test_data = {0};
  rt_uint32_t received = 0;

  rt_kprintf("Mlog test thread started, frequency: %d Hz\n", MLOG_TEST_02_SD_MLOG_FREQ_HZ);

  while (1) {
    /* Wait for timer event */
    rt_event_recv(&mlog_test_event, MLOG_TEST_EVENT_FLAG_TRIGGER, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                  RT_WAITING_FOREVER, &received);

    if (mlog_test_running) {
      /* Generate test data */
      generate_test_data(&test_data);

/* Publish to MCN hub (disabled) */
#ifdef PKG_USING_UMCN
      mcn_publish(MCN_HUB(mlog_test_data), &test_data);
#endif

      /* Push to mlog if enabled */
      if (Mlog_Test_Data_ID >= 0 && mlog_push_en) {
        mlog_push_msg((uint8_t *)&test_data, Mlog_Test_Data_ID, sizeof(sensorData_t));
      }
    }
  }
}

/* Public interface functions */
int mlog_test_start(void) {
  if (mlog_test_running) {
    rt_kprintf("Mlog test is already running\n");
    return -1;
  }

  if (!mlog_test_thread_created) {
    /* Create and start thread */
    rt_thread_init(&task_tid_mlog_test, "mlogTest", mlog_test_thread_entry, RT_NULL, task_stack_mlog_test,
                   MLOG_TEST_THREAD_STACK_SIZE, MLOG_TEST_THREAD_PRIORITY, MLOG_TEST_THREAD_TIMESLICE);
    rt_thread_startup(&task_tid_mlog_test);
    mlog_test_thread_created = RT_TRUE;
  }

  /* Start timer and enable data generation */
  if (mlog_test_timer) {
    rt_timer_start(mlog_test_timer);
  }
  mlog_test_running = RT_TRUE;

  rt_kprintf("Mlog test started\n");
  return 0;
}

void mlog_test_stop(void) {
  if (!mlog_test_running) {
    rt_kprintf("Mlog test is not running\n");
    return;
  }

  /* Stop timer and disable data generation */
  if (mlog_test_timer) {
    rt_timer_stop(mlog_test_timer);
  }
  mlog_test_running = RT_FALSE;

  rt_kprintf("Mlog test stopped\n");
}

rt_bool_t mlog_test_is_running(void) { return mlog_test_running; }

void mlog_test_get_data(sensorData_t *data) {
  if (!data) return;
#ifdef PKG_USING_UMCN
  if (mcn_poll(test_sub_node)) {
    mcn_copy(MCN_HUB(mlog_test_data), test_sub_node, data);
  }
#else
/* MCN 未启用（PKG_USING_UMCN 未定义），此处不返回任何数据 */
#endif
}

INIT_APP_EXPORT(mlog_test_start);

#endif