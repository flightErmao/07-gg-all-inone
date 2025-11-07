/****************************************************************************
 *
 * Rate Control Task
 * This task periodically calls MulticopterRateControl::Run()
 *
 ****************************************************************************/

#include "rtthread.h"
#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>
#include "taskRateControl.h"
#include "multicopter_rate_control_wrapper.h"
#include "vehicle_angular_velocity_filter_wrapper.h"
#include "param.h"
#include <string.h>
#include "rtconfig.h"

#ifdef PROJECT_PX4_TASK_RATE_CONTROL_DEBUGPIN_EN
#include "debugPin.h"
#endif

typedef struct {
    float gyro[3];
    float accel[3];
    float dt;
    rt_bool_t valid;
} imu_sample_t;

static imu_sample_t g_latest_imu_sample = {0};
static struct rt_mutex g_imu_mutex;
static rt_bool_t g_imu_mutex_ready = RT_FALSE;

static void ensureImuMutex(void) {
    if (g_imu_mutex_ready) {
        return;
    }
    if (rt_mutex_init(&g_imu_mutex, "imu_mtx", RT_IPC_FLAG_FIFO) == RT_EOK) {
        g_imu_mutex_ready = RT_TRUE;
    }
}

static void loadImuFilterParams(imu_filter_params_t *params) {
    if (params == RT_NULL) {
        return;
    }

    memset(params, 0, sizeof(*params));

    params->sample_rate_hz = 1000.0f;
    params->gyro_cutoff_hz = 80.0f;
    params->notch0_freq_hz = 0.0f;
    params->notch0_bw_hz = 0.0f;
    params->notch1_freq_hz = 0.0f;
    params->notch1_bw_hz = 0.0f;
    params->accel_cutoff_hz = 30.0f;

    float value = 0.0f;

    if (getParam("imu_filter_sample_rate_hz", &value, sizeof(value)) == RT_EOK) {
        params->sample_rate_hz = value;
    }

    if (getParam("imu_filter_gyro_cutoff_hz", &value, sizeof(value)) == RT_EOK) {
        params->gyro_cutoff_hz = value;
    }

    if (getParam("imu_filter_notch0_freq_hz", &value, sizeof(value)) == RT_EOK) {
        params->notch0_freq_hz = value;
    }

    if (getParam("imu_filter_notch0_bw_hz", &value, sizeof(value)) == RT_EOK) {
        params->notch0_bw_hz = value;
    }

    if (getParam("imu_filter_notch1_freq_hz", &value, sizeof(value)) == RT_EOK) {
        params->notch1_freq_hz = value;
    }

    if (getParam("imu_filter_notch1_bw_hz", &value, sizeof(value)) == RT_EOK) {
        params->notch1_bw_hz = value;
    }

    if (getParam("imu_filter_accel_cutoff_hz", &value, sizeof(value)) == RT_EOK) {
        params->accel_cutoff_hz = value;
    }
}

static rt_bool_t fetchLatestImuSample(imu_sample_t *sample) {
    if ((sample == RT_NULL) || !g_imu_mutex_ready) {
        return RT_FALSE;
    }

    if (rt_mutex_take(&g_imu_mutex, RT_WAITING_NO) != RT_EOK) {
        return RT_FALSE;
    }

    if (!g_latest_imu_sample.valid) {
        rt_mutex_release(&g_imu_mutex);
        return RT_FALSE;
    }

    *sample = g_latest_imu_sample;
    g_latest_imu_sample.valid = RT_FALSE;

    rt_mutex_release(&g_imu_mutex);
    return RT_TRUE;
}

void taskRateControlSubmitImuSample(const float gyro[3], const float accel[3], float dt) {
    if (!g_imu_mutex_ready || (gyro == RT_NULL) || (dt <= 0.0f)) {
        return;
    }

    if (rt_mutex_take(&g_imu_mutex, RT_WAITING_FOREVER) != RT_EOK) {
        return;
    }

    memcpy(g_latest_imu_sample.gyro, gyro, sizeof(g_latest_imu_sample.gyro));

    if (accel != RT_NULL) {
        memcpy(g_latest_imu_sample.accel, accel, sizeof(g_latest_imu_sample.accel));
    } else {
        memset(g_latest_imu_sample.accel, 0, sizeof(g_latest_imu_sample.accel));
    }

    g_latest_imu_sample.dt = dt;
    g_latest_imu_sample.valid = RT_TRUE;

    rt_mutex_release(&g_imu_mutex);
}

static void taskRateControlInit(void) {
    // Initialize MulticopterRateControl instance
    // Set vtol = 0 for regular multicopter mode
    int ret = multicopter_rate_control_init(0);
    if (ret != 0) {
        rt_kprintf("Failed to initialize MulticopterRateControl\n");
        return;
    }
    rt_kprintf("MulticopterRateControl initialized successfully\n");

    ensureImuMutex();

    imu_filter_params_t filter_params;
    loadImuFilterParams(&filter_params);

    if (vehicle_angular_velocity_filter_init(&filter_params) != 0) {
        rt_kprintf("Failed to initialize IMU filter\n");
    } else {
        rt_kprintf("IMU filter initialized sr=%.1fHz cutoff=%.1fHz\n",
                   (double)filter_params.sample_rate_hz,
                   (double)filter_params.gyro_cutoff_hz);
    }
}

static void rateControlThreadEntry(void* parameter) {
    uint32_t tick = 0;
    
    taskRateControlInit();
    
    // Wait a bit for initialization to complete
    rt_thread_mdelay(100);
    while (1) {
      imu_sample_t sample = {0};
      float sample_dt = PROJECT_PX4_TASK_RATE_CONTROL_PERIOD_MS / 1000.0f;

      if (fetchLatestImuSample(&sample)) {
        sample_dt = sample.dt;
        vehicle_angular_velocity_filter_push_measurement(sample.dt, sample.gyro, sample.accel);
      }

      float filtered_gyro[3] = {0.0f, 0.0f, 0.0f};
      float filtered_accel[3] = {0.0f, 0.0f, 0.0f};

      vehicle_angular_velocity_filter_get_latest(filtered_gyro, filtered_accel);

      multicopter_rate_control_step(filtered_gyro, filtered_accel, sample_dt);

      rt_thread_mdelay(PROJECT_PX4_TASK_RATE_CONTROL_PERIOD_MS);

      tick++;
    }
}

static int taskRateControlThreadAutoStart(void) {
#define THREAD_PRIORITY PROJECT_PX4_TASK_RATE_CONTROL_PRIORITY
#define THREAD_STACK_SIZE PROJECT_PX4_TASK_RATE_CONTROL_STACK_SIZE
#define THREAD_TIMESLICE 5

    static struct rt_thread task_tid_rate_control;
    static rt_uint8_t task_stack_rate_control[THREAD_STACK_SIZE];

    rt_thread_init(&task_tid_rate_control, "rate_control", rateControlThreadEntry, RT_NULL, 
                   task_stack_rate_control, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&task_tid_rate_control);
    
    rt_kprintf("Rate Control Task started\n");
    return RT_EOK;
}

#ifdef PROJECT_PX4_TASK_RATE_CONTROL_EN
INIT_APP_EXPORT(taskRateControlThreadAutoStart);
#endif

