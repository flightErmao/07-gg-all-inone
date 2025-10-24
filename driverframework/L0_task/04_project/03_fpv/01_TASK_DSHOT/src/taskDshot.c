#include "rtthread.h"
#include <rtdevice.h>
#include "taskDshot.h"
#include "aMcnStabilize.h"
#include "maths.h"
#include "string.h"
#include "timestamp.h"

#ifndef DSHOT_DEVICE_NAME
#define DSHOT_DEVICE_NAME "dshot"
#endif

#define INVALID_RPM_MCN 2000.0f  // Invalid RPM value to indicate motor stopped

static rt_device_t dshot_dev_ = RT_NULL;
static uint16_t mixer_remap_dshot_speed_[4] = {0, 0, 0, 0};

#ifdef PROJECT_MINIFLY_TASK_DSHOT_MOTOR_FILTER_EN
static motor_filter_t motor_filter_;
#endif

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
static uint8_t motor_pole_pairs_ = 7;
static uint16_t rpm_read_raw_[DSHOT_MOTOR_NUMS] = {0};  // Raw RPM values from device
static float rpm_freq_hz_[DSHOT_MOTOR_NUMS] = {0};       // Motor frequency in Hz for RPM filter
#endif

/* map motor value to DShot throttle range with armed state check */
static inline uint16_t map_motor_to_dshot(uint16_t motor_val) {
  float fv = (float)motor_val;
  float out = scaleRangef(fv, 0.0f, 65535.0f, 48.0f, 2047.0f);
  return (uint16_t)(out);
}

/* API to get mapped motor values for external access */
void task_dshot_get_mapped(uint16_t* m1, uint16_t* m2, uint16_t* m3, uint16_t* m4) {
  if (m1) *m1 = mixer_remap_dshot_speed_[0];
  if (m2) *m2 = mixer_remap_dshot_speed_[1];
  if (m3) *m3 = mixer_remap_dshot_speed_[2];
  if (m4) *m4 = mixer_remap_dshot_speed_[3];
}

static void device_init(void) {
  dshot_dev_ = rt_device_find(DSHOT_DEVICE_NAME);
  if (dshot_dev_) {
    rt_device_open(dshot_dev_, RT_DEVICE_OFLAG_RDWR);
  } else {
    rt_kprintf("[taskDshot] device %s not found\n", DSHOT_DEVICE_NAME);
  }

  /* Initialize mlog DShot functionality */
  mlogDshotInit();

#ifdef PROJECT_MINIFLY_TASK_DSHOT_MOTOR_FILTER_EN
  /* Initialize motor filter */
  motor_filter_init(&motor_filter_);
  rt_kprintf("[taskDshot] motor filter initialized\n");
#endif
}

static void rtos_init(void) { /* DShot task now uses MCN message queue for synchronization */ }

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
static void dshotReadRpm(void) {
  uint16_t rpm_read_temp[DSHOT_MOTOR_NUMS] = {0};
  uint32_t dshot_erpm100[DSHOT_MOTOR_NUMS] = {0};
  // Read RPM data from all physical motors
  if (dshot_dev_) {
    rt_device_read(dshot_dev_, 0, rpm_read_temp, 0);
  } else {
    return;
  }

  // Save raw RPM values for mlog
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    // store raw eRPM/100 as read from device
    rpm_read_raw_[i] = rpm_read_temp[i];
  }

  // Convert: eRPM/100 -> eRPM -> mechanical RPM -> frequency Hz
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    dshot_erpm100[i] = rpm_read_temp[i];
    if (dshot_erpm100[i] == 0) {
      // TODO:we need safe guard to lock to disarm when in the air and bidir-dshot link invalid
      rpm_freq_hz_[i] = 0.0f;
    } else {
      // real eRPM = (eRPM/100) * 100
      const float erpm_real = (float)(dshot_erpm100[i] * 100u);
      // mechanical RPM = eRPM / pole_pairs
      const float mech_rpm = erpm_real / (float)motor_pole_pairs_;
      rpm_freq_hz_[i] = mech_rpm / 60.0f;
    }
  }
}
#endif

static void mixterRemapToDshotSpeed(void) {
  mixer_data_t mixer_data_raw = {0};
  state_t state = {0};
  bool armed = false;

  mcnMixerAcquire(&mixer_data_raw);

#ifdef PROJECT_MINIFLY_TASK_DSHOT_MOTOR_FILTER_EN
  mixer_data_t mixer_data_filtered = {0};
  motor_filter_apply(&motor_filter_, mixer_data_raw.motor_val, mixer_data_filtered.motor_val);
  mixer_data_filtered.timestamp = mixer_data_raw.timestamp;
#else
  /* Use raw data directly without filtering */
  mixer_data_t mixer_data_filtered = mixer_data_raw;
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
  mcnStateAcquire(&state);
  armed = state.armed;
#else
  armed = false;
#endif

  if (armed) {
    for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
      mixer_remap_dshot_speed_[i] = map_motor_to_dshot(mixer_data_filtered.motor_val[i]);
    }
  } else {
    memset(mixer_remap_dshot_speed_, 0, sizeof(mixer_remap_dshot_speed_));
  }
}

static void getAndPushMlogData(void) {
  /* Prepare mlog data structure with proper alignment */
  mlogDshotData_t mlog_data __attribute__((aligned(4))) = {0};
  mlog_data.timestamp = timestamp_micros();
  memcpy(mlog_data.dshot_mapped, mixer_remap_dshot_speed_, sizeof(mixer_remap_dshot_speed_));

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
  memcpy(mlog_data.erpm, rpm_read_raw_, sizeof(rpm_read_raw_));
#endif
  mlogDshotPush(&mlog_data);
}

static void readAndPubRpmData(void) {
#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
  dshotReadRpm();
  rpm_data_bus_t rpm_msg;
  memcpy(rpm_msg.rpm, rpm_freq_hz_, sizeof(rpm_freq_hz_));
  rpm_msg.timestamp = timestamp_micros();
  mcnRpmDataPublish(&rpm_msg);
#endif
}

static void writeDshotCmdToDevice(void) {
  if (dshot_dev_) {
    rt_device_write(dshot_dev_, 0x0F, mixer_remap_dshot_speed_, 4);
  }
}

static void task_dshot_entry(void* parameter) {
  device_init();
  rtos_init();
  while (1) {
    mcnWaitMixerPub();
    DEBUG_PIN_DEBUG2_HIGH();
    mixterRemapToDshotSpeed();
    writeDshotCmdToDevice();
    readAndPubRpmData();
    getAndPushMlogData();
    DEBUG_PIN_DEBUG2_LOW();
  }
}

static int task_dshot_autostart(void) {
#define THREAD_PRIORITY 6
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

  static struct rt_thread task_tid_dshot;
  static rt_uint8_t task_stack_dshot[THREAD_STACK_SIZE];

  rt_thread_init(&task_tid_dshot, "dshot", task_dshot_entry, RT_NULL, task_stack_dshot, THREAD_STACK_SIZE,
                 THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_dshot);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_DSHOT_EN
INIT_APP_EXPORT(task_dshot_autostart);
#endif