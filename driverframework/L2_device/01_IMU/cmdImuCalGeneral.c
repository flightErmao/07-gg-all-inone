#include "rtconfig.h"

#ifdef BSP_USING_IMU_CMD_CAL_GENERAL

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <rtdevice.h>
#include <rtthread.h>

#include "imu.h"

#include "../../L0_task/04_project/01_basic/TASK_05_PARAM/inc/param.h"
#include "../../L0_task/04_project/01_basic/TASK_05_PARAM/inc/imuCaliParam.h"

#define LOG_TAG "cmd.imu_cal"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define CALIBRATION_SAMPLES 500
#define DELAY_SAMPLES 125
#define GRAVITY_VALUE 1.0f
#define LPF_FACTOR 0.92f

typedef enum {
  ALIGN_DEFAULT = 0,
  CW0_DEG = 1,
  CW90_DEG = 2,
  CW180_DEG = 3,
  CW270_DEG = 4,
  CW0_DEG_FLIP = 5,
  CW90_DEG_FLIP = 6,
  CW180_DEG_FLIP = 7,
  CW270_DEG_FLIP = 8,
  CW45_DEG = 9,
  CW135_DEG = 10,
  CW225_DEG = 11,
  CW315_DEG = 12,
  CW45_DEG_FLIP = 13,
  CW135_DEG_FLIP = 14,
  CW225_DEG_FLIP = 15,
  CW315_DEG_FLIP = 16,
  ALIGN_CUSTOM = 17,
} sensor_align_e;

typedef enum { X, Y, Z } ORDER_e;

typedef struct {
    const char *name;
    const char *param_name;
    float rest_reference[3];
    float max_offset;
    float quality_good;
    float quality_ok;
    float offset[3];
    bool calibrated;
} imu_calib_t;

static rt_device_t imu_dev = RT_NULL;
static bool imu_device_ready = false;
static uint8_t imu_orientation = ALIGN_DEFAULT;

enum {
    SENSOR_ACC = 0,
    SENSOR_GYRO,
    SENSOR_COUNT,
};

static imu_calib_t imu_calibrations[SENSOR_COUNT] = {
    [SENSOR_ACC] =
        {
            .name = "ACC",
            .param_name = IMU_CALI_PARAM_ACC_BIAS,
            .rest_reference = {0.0f, 0.0f, GRAVITY_VALUE},
            .max_offset = 2.0f,
            .quality_good = 0.5f,
            .quality_ok = 1.0f,
            .offset = {0.0f},
            .calibrated = false,
        },
    [SENSOR_GYRO] =
        {
            .name = "GYRO",
            .param_name = IMU_CALI_PARAM_GYRO_BIAS,
            .rest_reference = {0.0f, 0.0f, 0.0f},
            .max_offset = 50.0f,
            .quality_good = 5.0f,
            .quality_ok = 15.0f,
            .offset = {0.0f},
            .calibrated = false,
        },
};

#define acc_cal (imu_calibrations[SENSOR_ACC])
#define gyro_cal (imu_calibrations[SENSOR_GYRO])

typedef struct {
  float avg[3];
  float sum[3];
  int count;
  bool active;
  bool validated;
  float backup_offset[3];
  bool backup_calibrated;
} calib_runtime_t;

static calib_runtime_t calib_state = {0};
static int active_sensor = -1;
static uint32_t timer_guard = 0;
static uint32_t timer_guard_limit = 0;
static float acc_lsb_to_g = ACC_SCALE_16G;
static float gyro_lsb_to_dps = GYRO_SCALE_2000DPS;

#ifndef IMU_CAL_GENERAL_DEVICE_NAME
#define IMU_CAL_GENERAL_DEVICE_NAME "imu0"
#endif

static void loadCalibrationFromFlash(int sensor_id);
static void resetCalibration(int sensor_id);
static void startCalibration(int sensor_id);
static void processCalibration(int sensor_id, float data[3]);

static void lpf_update(float* current, float new_value, float factor) {
  *current = *current * factor + new_value * (1.0f - factor);
}

static uint8_t normalize_orientation(uint8_t rotation) {
  switch (rotation) {
    case CW45_DEG:
      return CW0_DEG;
    case CW135_DEG:
      return CW90_DEG;
    case CW225_DEG:
      return CW180_DEG;
    case CW315_DEG:
      return CW270_DEG;
    case CW45_DEG_FLIP:
      return CW0_DEG_FLIP;
    case CW135_DEG_FLIP:
      return CW90_DEG_FLIP;
    case CW225_DEG_FLIP:
      return CW180_DEG_FLIP;
    case CW315_DEG_FLIP:
      return CW270_DEG_FLIP;
    default:
      return rotation;
  }
}

static void alignSensorViaRotation(float *dest, uint8_t rotation) {
    const float x = dest[X];
    const float y = dest[Y];
    const float z = dest[Z];

    switch (rotation) {
        default:
        case CW0_DEG:
            dest[X] = x;
            dest[Y] = y;
            dest[Z] = z;
            break;
        case CW90_DEG:
            dest[X] = y;
            dest[Y] = -x;
            dest[Z] = z;
            break;
        case CW180_DEG:
            dest[X] = -x;
            dest[Y] = -y;
            dest[Z] = z;
            break;
        case CW270_DEG:
            dest[X] = -y;
            dest[Y] = x;
            dest[Z] = z;
            break;
        case CW0_DEG_FLIP:
            dest[X] = -x;
            dest[Y] = y;
            dest[Z] = -z;
            break;
        case CW90_DEG_FLIP:
            dest[X] = y;
            dest[Y] = x;
            dest[Z] = -z;
            break;
        case CW180_DEG_FLIP:
            dest[X] = x;
            dest[Y] = -y;
            dest[Z] = -z;
            break;
        case CW270_DEG_FLIP:
            dest[X] = -y;
            dest[Y] = -x;
            dest[Z] = -z;
            break;
    }
}

static rt_err_t imu_device_prepare(void) {
  if (imu_device_ready) {
    return RT_EOK;
  }

  imu_dev = rt_device_find(IMU_CAL_GENERAL_DEVICE_NAME);
  if (imu_dev == RT_NULL) {
    LOG_E("%s device not found", IMU_CAL_GENERAL_DEVICE_NAME);
    return -RT_ERROR;
  }

  rt_err_t ret = rt_device_open(imu_dev, RT_DEVICE_OFLAG_RDWR);
  if (ret != RT_EOK && ret != -RT_EBUSY) {
    LOG_E("open imu device failed: %d", ret);
    return ret;
  }

  imu_dev_t imu_ptr = (imu_dev_t)imu_dev;
  if (imu_ptr != RT_NULL) {
    if (imu_ptr->config.acc_scale_factor > 0.0f) {
      acc_lsb_to_g = imu_ptr->config.acc_scale_factor;
    }
    if (imu_ptr->config.gyro_scale_factor > 0.0f) {
      gyro_lsb_to_dps = imu_ptr->config.gyro_scale_factor;
    }
  }

  uint8_t orientation = ALIGN_DEFAULT;
  ret = getParam(IMU_CALI_PARAM_ORIENTATION, &orientation, sizeof(orientation));
  if (ret == RT_EOK) {
    imu_orientation = normalize_orientation(orientation);
  } else {
    imu_orientation = ALIGN_DEFAULT;
    LOG_W("load imu_orientation failed: %d, fallback to default", ret);
  }

  loadCalibrationFromFlash(SENSOR_ACC);
  loadCalibrationFromFlash(SENSOR_GYRO);

  imu_device_ready = true;
  return RT_EOK;
}

static void convert_raw_to_vectors(const uint8_t* raw_buffer, float acc_body[3], float gyro_raw[3]) {
  if (acc_body) {
    int16_t acc_x = (int16_t)((raw_buffer[1] << 8) | raw_buffer[0]);
    int16_t acc_y = (int16_t)((raw_buffer[3] << 8) | raw_buffer[2]);
    int16_t acc_z = (int16_t)((raw_buffer[5] << 8) | raw_buffer[4]);

    acc_body[0] = (float)acc_x * acc_lsb_to_g;
    acc_body[1] = (float)acc_y * acc_lsb_to_g;
    acc_body[2] = (float)acc_z * acc_lsb_to_g;
    alignSensorViaRotation(acc_body, imu_orientation);
  }

  if (gyro_raw) {
    int16_t gyro_x = (int16_t)((raw_buffer[7] << 8) | raw_buffer[6]);
    int16_t gyro_y = (int16_t)((raw_buffer[9] << 8) | raw_buffer[8]);
    int16_t gyro_z = (int16_t)((raw_buffer[11] << 8) | raw_buffer[10]);

    gyro_raw[0] = (float)gyro_x;
    gyro_raw[1] = (float)gyro_y;
    gyro_raw[2] = (float)gyro_z;
  }
}

static rt_err_t imu_read_accgyro(float acc_body[3], float gyro_raw[3]) {
  uint8_t raw_buffer[14] = {0};
  rt_device_read(imu_dev, IMU_POS_ACC_GYRO, raw_buffer, sizeof(raw_buffer));
  convert_raw_to_vectors(raw_buffer, acc_body, gyro_raw);
  return RT_EOK;
}

static void startCalibration(int sensor_id) {
    if (active_sensor != -1) {
      LOG_W("another calibration is already running");
      return;
    }

    imu_calib_t *cal = &imu_calibrations[sensor_id];

    for (int i = 0; i < 3; i++) {
        calib_state.avg[i] = cal->rest_reference[i];
        calib_state.sum[i] = 0.0f;
        calib_state.backup_offset[i] = cal->offset[i];
    }
    calib_state.count = 0;
    calib_state.active = true;
    calib_state.validated = false;
    calib_state.backup_calibrated = cal->calibrated;

    cal->calibrated = false;
    active_sensor = sensor_id;

    LOG_I("%s calibration started", cal->name);
}

static void processCalibration(int sensor_id, float data[3]) {
    if (active_sensor != sensor_id || !calib_state.active) {
        return;
    }

    imu_calib_t *cal = &imu_calibrations[sensor_id];
    calib_state.count++;

    if (calib_state.count <= CALIBRATION_SAMPLES) {
      if (sensor_id == SENSOR_GYRO) {
        for (int i = 0; i < 3; i++) {
          calib_state.sum[i] += data[i];
        }
      } else {
        for (int i = 0; i < 3; i++) {
          lpf_update(&calib_state.avg[i], data[i], LPF_FACTOR);
        }
      }

        if ((calib_state.count % (CALIBRATION_SAMPLES / 4)) == 0) {
          LOG_I("%s progress: %d%%", cal->name, (calib_state.count * 100) / CALIBRATION_SAMPLES);
        }

        if (calib_state.count == CALIBRATION_SAMPLES) {
            float offset[3];
            bool valid = true;

            if (sensor_id == SENSOR_GYRO) {
              for (int i = 0; i < 3; i++) {
                calib_state.avg[i] = calib_state.sum[i] / (float)CALIBRATION_SAMPLES;
              }
            }

            for (int i = 0; i < 3; i++) {
              offset[i] = calib_state.avg[i] - cal->rest_reference[i];
              if (fabsf(offset[i]) > cal->max_offset) {
                offset[i] = (offset[i] > 0.0f) ? cal->max_offset : -cal->max_offset;
              }
                if (fabsf(offset[i]) > cal->quality_ok) {
                    valid = false;
                }
            }

            if (sensor_id == SENSOR_GYRO) {
              float total = sqrtf(offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]);
              if (total > 60.0f) {
                valid = false;
                LOG_E("gyro total bias too high: %.2f", total);
              }
            }

            if (valid) {
              for (int i = 0; i < 3; i++) {
                cal->offset[i] = offset[i];
              }
              calib_state.validated = true;

              float max_err = fmaxf(fabsf(offset[0]), fmaxf(fabsf(offset[1]), fabsf(offset[2])));
              const char* quality = (max_err <= cal->quality_good) ? "excellent" : "good";
              LOG_I("%s sample captured (%s)", cal->name, quality);
            } else {
              for (int i = 0; i < 3; i++) {
                cal->offset[i] = calib_state.backup_offset[i];
              }
              cal->calibrated = calib_state.backup_calibrated;
              calib_state.active = false;
              active_sensor = -1;
              LOG_E("%s calibration validation failed", cal->name);
            }
        }
    } else if (calib_state.validated && calib_state.count <= CALIBRATION_SAMPLES + DELAY_SAMPLES) {
      if (calib_state.count == CALIBRATION_SAMPLES + DELAY_SAMPLES) {
        rt_err_t ret = setParam(cal->param_name, cal->offset, sizeof(float) * 3);
        if (ret == RT_EOK) {
          cal->calibrated = true;
          LOG_I("%s calibration saved: [%.6f, %.6f, %.6f]", cal->name, cal->offset[0], cal->offset[1], cal->offset[2]);
        } else {
          LOG_E("%s calibration store failed: %d", cal->name, ret);
          for (int i = 0; i < 3; i++) {
            cal->offset[i] = calib_state.backup_offset[i];
          }
          cal->calibrated = calib_state.backup_calibrated;
        }
        calib_state.active = false;
        active_sensor = -1;
      }
    } else {
      calib_state.active = false;
      active_sensor = -1;
    }
}

static void loadCalibrationFromFlash(int sensor_id) {
    imu_calib_t *cal = &imu_calibrations[sensor_id];
    float offset[3] = {0.0f};

    if (getParam(cal->param_name, offset, sizeof(offset)) == RT_EOK) {
      memcpy(cal->offset, offset, sizeof(offset));
      cal->calibrated = !(offset[0] == 0.0f && offset[1] == 0.0f && offset[2] == 0.0f);
      if (cal->calibrated) {
        LOG_I("%s calibration loaded: [%.6f, %.6f, %.6f]", cal->name, cal->offset[0], cal->offset[1], cal->offset[2]);
      }
    }
}

static void resetCalibration(int sensor_id) {
    imu_calib_t *cal = &imu_calibrations[sensor_id];
    float zero[3] = {0.0f};

    memcpy(cal->offset, zero, sizeof(zero));
    cal->calibrated = false;

    if (setParam(cal->param_name, zero, sizeof(zero)) == RT_EOK) {
      LOG_I("%s calibration reset", cal->name);
    } else {
      LOG_E("%s calibration reset failed", cal->name);
    }
}

static rt_err_t run_calibration(int sensor_id) {
  if (imu_device_prepare() != RT_EOK) {
    return -RT_ERROR;
  }

  startCalibration(sensor_id);
  if (active_sensor != sensor_id) {
    return -RT_ERROR;
  }

  timer_guard = 0;
  timer_guard_limit = (CALIBRATION_SAMPLES + DELAY_SAMPLES) * 4;

  while (calib_state.active) {
    float acc_body[3] = {0.0f};
    float gyro_raw[3] = {0.0f};

    rt_err_t ret =
        imu_read_accgyro(active_sensor == SENSOR_ACC ? acc_body : NULL, active_sensor == SENSOR_GYRO ? gyro_raw : NULL);
    if (ret != RT_EOK) {
      LOG_E("read imu sample failed: %d", ret);
      calib_state.active = false;
      active_sensor = -1;
      break;
    }

    if (active_sensor == SENSOR_ACC) {
      processCalibration(SENSOR_ACC, acc_body);
    } else {
      processCalibration(SENSOR_GYRO, gyro_raw);
    }

    timer_guard++;
    if (calib_state.active && timer_guard > timer_guard_limit) {
      int sensor = active_sensor;
      if (sensor >= 0 && sensor < SENSOR_COUNT) {
        LOG_E("%s calibration timeout", imu_calibrations[sensor].name);
      } else {
        LOG_E("calibration timeout");
      }
      calib_state.active = false;
      active_sensor = -1;
      break;
    }

    rt_thread_mdelay(1);
  }

  if (!imu_calibrations[sensor_id].calibrated) {
    LOG_E("%s calibration did not complete", imu_calibrations[sensor_id].name);
    return -RT_ERROR;
  }

  LOG_I("%s final bias: [%.6f, %.6f, %.6f]", imu_calibrations[sensor_id].name, imu_calibrations[sensor_id].offset[0],
        imu_calibrations[sensor_id].offset[1], imu_calibrations[sensor_id].offset[2]);
  return RT_EOK;
}

static void print_usage(void) {
  LOG_I("Usage:");
  LOG_I("  imu_cal_general [all|acc|gyro]");
  LOG_I("  imu_cal_general reset [all|acc|gyro]");
}

static void cmdImuCalGeneral(int argc, char** argv) {
  bool do_acc = true;
  bool do_gyro = true;

  if (argc >= 2) {
    if (strcmp(argv[1], "acc") == 0) {
      do_gyro = false;
    } else if (strcmp(argv[1], "gyro") == 0) {
      do_acc = false;
    } else if (strcmp(argv[1], "all") == 0) {
      do_acc = true;
      do_gyro = true;
    } else if (strcmp(argv[1], "reset") == 0) {
      if (argc == 2 || strcmp(argv[2], "all") == 0) {
        resetCalibration(SENSOR_ACC);
        resetCalibration(SENSOR_GYRO);
      } else if (strcmp(argv[2], "acc") == 0) {
        resetCalibration(SENSOR_ACC);
      } else if (strcmp(argv[2], "gyro") == 0) {
        resetCalibration(SENSOR_GYRO);
      } else {
        print_usage();
      }
      return;
    } else {
      print_usage();
      return;
    }
  }

  if (do_acc) {
    if (run_calibration(SENSOR_ACC) != RT_EOK) {
      LOG_E("accelerometer calibration failed");
      return;
    }
  }

  if (do_gyro) {
    if (run_calibration(SENSOR_GYRO) != RT_EOK) {
      LOG_E("gyroscope calibration failed");
      return;
    }
  }

  LOG_I("IMU calibration finished");
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT_ALIAS(cmdImuCalGeneral, imu_cal_general, Calibrate IMU(acc | gyro | all));
#endif

#endif /* BSP_USING_IMU_CMD_CAL_GENERAL */
