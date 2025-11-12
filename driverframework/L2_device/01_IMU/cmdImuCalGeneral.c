#include "rtconfig.h"

#ifdef BSP_USING_IMU_CMD_CAL_GENERAL

#include "imu_data.h"

#include <math.h>
#include <stdint.h>

#include "hal/imu/imu.h"
#include "base/rt_helpers.h"
#include "uMCN.h"
#include "module/math/rotation.h"
#include "module/math/trigonometric.h"
#include "module/param/param_fydelix.h"
#include "module/system/systime.h"

#define CALIBRATION_SAMPLES 500
#define DELAY_SAMPLES 125
#define GRAVITY_VALUE 1.0f
#define LPF_FACTOR 0.92f

typedef enum {
    ALIGN_DEFAULT = 0,  // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

    // R, P, Y
    CW0_DEG = 1,         // 00,00,00
    CW90_DEG = 2,        // 00,00,01
    CW180_DEG = 3,       // 00,00,10
    CW270_DEG = 4,       // 00,00,11
    CW0_DEG_FLIP = 5,    // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    CW90_DEG_FLIP = 6,   // 00,10,01
    CW180_DEG_FLIP = 7,  // 00,10,10
    CW270_DEG_FLIP = 8,  // 00,10,11

    // Extended orientations (custom angles mapped via rotation matrix)
    CW45_DEG = 9,
    CW135_DEG = 10,
    CW225_DEG = 11,
    CW315_DEG = 12,
    CW45_DEG_FLIP = 13,
    CW135_DEG_FLIP = 14,
    CW225_DEG_FLIP = 15,
    CW315_DEG_FLIP = 16,

    ALIGN_CUSTOM = 17,  // arbitrary sensor angles, e.g. for external sensors
} sensor_align_e;

typedef enum { X, Y, Z } ORDER_e;

// ===== Type Definitions =====
// IMU calibration structure that combines static configuration and runtime state
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

static rt_device_t imu_dev;
static uint8_t imu_orientation;        // will be loaded from flash
static float gyr_data[3];              // in deg/s
static float acc_data[3];              // in m/s^2 and deg/s
static float imu_temp_data;            // in deg C
static float acc_gyro_scaled_data[6];  // [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]

static float alignment_angles[3] = {0, 0, 0};
static bool needYaw45Overlay = false;  // Flag indicating CW45 rotation needed

enum {
    SENSOR_ACC = 0,
    SENSOR_GYRO,
    SENSOR_COUNT,
};

static imu_calib_t imu_calibrations[SENSOR_COUNT] = {
    [SENSOR_ACC] =
        {
            .name = "ACC",
            .param_name = "acc_calib_offset",
            .rest_reference = {0, 0, GRAVITY_VALUE},
            .max_offset = 2.0f,
            .quality_good = 0.5f,
            .quality_ok = 1.0f,
            .offset = {0},
            .calibrated = false,
        },
    [SENSOR_GYRO] =
        {
            .name = "Gyro",
            .param_name = "gyro_calib_offset",
            .rest_reference = {0, 0, 0},
            .max_offset = 50.0f,
            .quality_good = 5.0f,
            .quality_ok = 15.0f,
            .offset = {0},
            .calibrated = false,
        },
};

#define acc_cal (imu_calibrations[SENSOR_ACC])
#define gyro_cal (imu_calibrations[SENSOR_GYRO])

// Forward declarations for calibration functions
static void lpf_update(float *current, float new_value, float factor);
static void startCalibration(int sensor_id);
static void processCalibration(int sensor_id, float data[3]);
static void loadCalibrationFromFlash(int sensor_id);
static void resetCalibration(int sensor_id);

// imu temperature mcn, not used yet
MCN_DEFINE(fImuTemp, sizeof(imu_temp_data));

rt_err_t prepare_imu_data(void) {
    imu_dev = rt_device_find("imu0");
    RT_ASSERT(imu_dev != NULL);
    RT_TRY(rt_device_open(imu_dev, RT_DEVICE_OFLAG_RDWR));

    getParam("imu_orientation", &imu_orientation, sizeof(imu_orientation));

    // Load calibrations
    loadCalibrationFromFlash(0);  // ACC
    loadCalibrationFromFlash(1);  // Gyro

    // Overlay matrix precomputed at compile-time; reset overlay flag
    needYaw45Overlay = false;

    // Backward compatibility: map old extended orientations (9-16)
    // into base 8 orientations + cw45 overlay
    if (imu_orientation >= CW45_DEG && imu_orientation <= CW315_DEG_FLIP) {
        switch (imu_orientation) {
            case CW45_DEG:
                imu_orientation = CW0_DEG;
                needYaw45Overlay = true;
                break;
            case CW135_DEG:
                imu_orientation = CW90_DEG;
                needYaw45Overlay = true;
                break;
            case CW225_DEG:
                imu_orientation = CW180_DEG;
                needYaw45Overlay = true;
                break;
            case CW315_DEG:
                imu_orientation = CW270_DEG;
                needYaw45Overlay = true;
                break;
            case CW45_DEG_FLIP:
                imu_orientation = CW0_DEG_FLIP;
                needYaw45Overlay = true;
                break;
            case CW135_DEG_FLIP:
                imu_orientation = CW90_DEG_FLIP;
                needYaw45Overlay = true;
                break;
            case CW225_DEG_FLIP:
                imu_orientation = CW180_DEG_FLIP;
                needYaw45Overlay = true;
                break;
            case CW315_DEG_FLIP:
                imu_orientation = CW270_DEG_FLIP;
                needYaw45Overlay = true;
                break;
        }
    }

    return RT_EOK;
}

// Add function to set calibration parameters
void setAccCalibration(float offset[3]) {
    for (int i = 0; i < 3; i++) {
        acc_cal.offset[i] = offset[i];
    }
    acc_cal.calibrated = true;
}

// Add function to get current calibration parameters
void getAccCalibration(float offset[3]) {
    for (int i = 0; i < 3; i++) {
        offset[i] = acc_cal.offset[i];
    }
}

// Add function to enable/disable calibration
void enableAccCalibration(bool enable) { acc_cal.calibrated = enable; }

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
        case CW45_DEG:
        case CW135_DEG:
        case CW225_DEG:
        case CW315_DEG:
        case CW45_DEG_FLIP:
        case CW135_DEG_FLIP:
        case CW225_DEG_FLIP:
        case CW315_DEG_FLIP:
            break;
    }
}

float *collect_gyr_data(void) {
    rt_size_t r_size;
    float gyro_temp[3];

    // Read scaled float data directly from driver
    r_size = rt_device_read(imu_dev, IMU_POS_GYRO_SCALED, (void *)gyro_temp, sizeof(float) * 3);
    uint32_t timestamp = systime_now_us();

    // Apply calibration offset FIRST (in sensor's native coordinate system)
    for (int i = 0; i < 3; i++) {
        gyro_temp[i] = gyro_temp[i] - (gyro_cal.calibrated ? gyro_cal.offset[i] : 0);
    }

    // THEN apply sensor alignment/rotation to body frame
    alignSensorViaRotation(gyro_temp, imu_orientation);

    // Copy to output
    for (int i = 0; i < 3; i++) {
        gyr_data[i] = gyro_temp[i];
    }

    return gyr_data;
}

float *collect_accgyr_data(void) {
    rt_device_read(imu_dev, IMU_POS_ACC_GYRO_SCALED, acc_gyro_scaled_data, sizeof(float) * 6);

    // Process ACC
    // NOTE: Accelerometer calibration MUST be done in body frame (after rotation)
    // because gravity reference [0, 0, 1g] is defined relative to body frame
    float acc_temp[3] = {acc_gyro_scaled_data[0], acc_gyro_scaled_data[1], acc_gyro_scaled_data[2]};

    // Apply sensor alignment/rotation to body frame FIRST
    alignSensorViaRotation(acc_temp, imu_orientation);

    // Process calibration sample if active (using rotated data in body frame)
    processCalibration(0, acc_temp);

    // Apply calibration offset (in body frame)
    for (int i = 0; i < 3; i++) {
        acc_data[i] = acc_temp[i] - (acc_cal.calibrated ? acc_cal.offset[i] : 0);
        acc_gyro_scaled_data[i] = acc_data[i];
    }

    // Process Gyro
    float gyro_temp[3] = {acc_gyro_scaled_data[3], acc_gyro_scaled_data[4], acc_gyro_scaled_data[5]};

    // IMPORTANT: Calibration must happen BEFORE rotation
    // Process calibration sample if active (using raw sensor data before calibration offset and rotation)
    processCalibration(
        1, gyro_temp);  // Use raw sensor data (before calibration offset and rotation) in sensor coordinate system

    // Apply calibration offset FIRST (in sensor's native coordinate system)
    for (int i = 0; i < 3; i++) {
        gyro_temp[i] = gyro_temp[i] - (gyro_cal.calibrated ? gyro_cal.offset[i] : 0);
    }

    // THEN apply sensor alignment/rotation to body frame
    alignSensorViaRotation(gyro_temp, imu_orientation);

    // Copy to output arrays
    for (int i = 0; i < 3; i++) {
        gyr_data[i] = gyro_temp[i];
        acc_gyro_scaled_data[i + 3] = gyro_temp[i];
    }

    return acc_gyro_scaled_data;
}

/*TODO: need to check the temp convertion in BMI270 datasheet*/
void collect_imu_temp_data(void) {
    rt_size_t r_size;
    // Read scaled float temperature data directly from driver
    r_size = rt_device_read(imu_dev, IMU_POS_TEMP_SCALED, (void *)&imu_temp_data, sizeof(float));
    mcn_publish(MCN_HUB(fImuTemp), &imu_temp_data);
}

// ===== Ultra-Simplified Unified Calibration System =====

#define CALIBRATION_SAMPLES 500
#define DELAY_SAMPLES 125
#define GRAVITY_VALUE 1.0f
#define LPF_FACTOR 0.92f

// Calibration runtime state (shared between sensors)
typedef struct {
    float avg[3];
    float sum[3];  // Accumulator for simple average
    int count;
    bool active;
    bool validated;
    float backup_offset[3];
    bool backup_calibrated;
} calib_runtime_t;

// Single runtime state - only one calibration can run at a time
static calib_runtime_t calib_state = {0};
static int active_sensor = -1;  // -1=none, 0=acc, 1=gyro

// imu_calibrations provides both configuration and runtime calibration state
// for each sensor. The configuration fields remain constant while offset and
// calibrated flag are updated during calibration.

// Simple low-pass filter
static void lpf_update(float *current, float new_value, float factor) {
    *current = *current * factor + new_value * (1.0f - factor);
}

// ===== Generic Calibration Functions =====

static void startCalibration(int sensor_id) {
    if (active_sensor != -1) {
        rt_kprintf("Another calibration is already running\n");
        return;
    }

    imu_calib_t *cal = &imu_calibrations[sensor_id];

    // Initialize state
    for (int i = 0; i < 3; i++) {
        calib_state.avg[i] = cal->rest_reference[i];
        calib_state.sum[i] = 0.0f;  // Initialize accumulator for gyro
    }
    calib_state.count = 0;
    calib_state.active = true;
    calib_state.validated = false;

    // Backup and disable current calibration
    for (int i = 0; i < 3; i++) {
        calib_state.backup_offset[i] = cal->offset[i];
    }
    calib_state.backup_calibrated = cal->calibrated;
    cal->calibrated = false;

    active_sensor = sensor_id;
    rt_kprintf("%s calibration started\n", cal->name);
}

void startSimpleAccCalibration(void) { startCalibration(0); }

static void processCalibration(int sensor_id, float data[3]) {
    if (active_sensor != sensor_id || !calib_state.active) {
        return;
    }

    imu_calib_t *cal = &imu_calibrations[sensor_id];
    calib_state.count++;

    // Phase 1: Sampling
    if (calib_state.count <= CALIBRATION_SAMPLES) {
        // For gyro: use simple average; for acc: use LPF
        if (sensor_id == 1) {  // Gyro
            for (int i = 0; i < 3; i++) {
                calib_state.sum[i] += data[i];
            }
        } else {  // ACC
            for (int i = 0; i < 3; i++) {
                lpf_update(&calib_state.avg[i], data[i], LPF_FACTOR);
            }
        }

        // Progress
        if ((calib_state.count % (CALIBRATION_SAMPLES / 4)) == 0) {
            rt_kprintf("%s: %d%%\n", cal->name, (calib_state.count * 100) / CALIBRATION_SAMPLES);
        }

        // Validate at end
        if (calib_state.count == CALIBRATION_SAMPLES) {
            float offset[3];
            bool valid = true;

            // Calculate final average for gyro
            if (sensor_id == 1) {  // Gyro
                for (int i = 0; i < 3; i++) {
                    calib_state.avg[i] = calib_state.sum[i] / CALIBRATION_SAMPLES;
                }
            }

            // Calculate offset
            for (int i = 0; i < 3; i++) {
                offset[i] = calib_state.avg[i] - cal->rest_reference[i];

                // Clamp
                if (fabsf(offset[i]) > cal->max_offset) {
                    offset[i] = (offset[i] > 0) ? cal->max_offset : -cal->max_offset;
                }

                // Check quality
                if (fabsf(offset[i]) > cal->quality_ok) {
                    valid = false;
                }
            }

            // Additional check for gyro total bias
            if (sensor_id == 1) {  // Gyro
                float total = sqrtf(offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]);
                if (total > 60.0f) {
                    valid = false;
                    rt_kprintf("Gyro total bias too high\n");
                }
            }

            if (valid) {
                // Apply
                for (int i = 0; i < 3; i++) {
                    cal->offset[i] = offset[i];
                }
                calib_state.validated = true;

                float max_err = fmaxf(fabsf(offset[0]), fmaxf(fabsf(offset[1]), fabsf(offset[2])));
                const char *quality = (max_err <= cal->quality_good) ? "excellent" : "good";
            } else {
                // Restore backup
                for (int i = 0; i < 3; i++) {
                    cal->offset[i] = calib_state.backup_offset[i];
                }
                cal->calibrated = calib_state.backup_calibrated;
                calib_state.active = false;
                active_sensor = -1;
                rt_kprintf("%s calib failed\n", cal->name);
            }
        }
    }
    // Phase 2: Delay before save
    else if (calib_state.validated && calib_state.count <= CALIBRATION_SAMPLES + DELAY_SAMPLES) {
        if (calib_state.count == CALIBRATION_SAMPLES + DELAY_SAMPLES) {
            // Save to flash
            if (setParam(cal->param_name, cal->offset, sizeof(float) * 3) == RT_EOK) {
                cal->calibrated = true;
            }
            calib_state.active = false;
            active_sensor = -1;
        }
    }
    // Phase 3: Timeout
    else {
        calib_state.active = false;
        active_sensor = -1;
    }
}

static void loadCalibrationFromFlash(int sensor_id) {
    imu_calib_t *cal = &imu_calibrations[sensor_id];
    float offset[3];

    if (getParam(cal->param_name, offset, sizeof(float) * 3) == RT_EOK) {
        if (offset[0] != 0 || offset[1] != 0 || offset[2] != 0) {
            for (int i = 0; i < 3; i++) {
                cal->offset[i] = offset[i];
            }
            cal->calibrated = true;
        }
    }
}

// Legacy compatibility
void applySimpleCalibration(void) {}

static void resetCalibration(int sensor_id) {
    imu_calib_t *cal = &imu_calibrations[sensor_id];
    float zero[3] = {0};

    for (int i = 0; i < 3; i++) {
        cal->offset[i] = 0;
    }
    cal->calibrated = false;

    setParam(cal->param_name, zero, sizeof(zero));
    rt_kprintf("%s calib reset\n", cal->name);
}

bool isSimpleCalibrationComplete(void) {
    // Check if acc calibration was completed (active_sensor would be -1 after completion)
    // Also check if it's not currently running
    return active_sensor != 0 && !calib_state.active && acc_cal.calibrated;
}

void resetAccCalibration(void) { resetCalibration(0); }

// ===== Gyroscope Calibration Functions =====

void setGyroCalibration(float offset[3]) {
    for (int i = 0; i < 3; i++) {
        gyro_cal.offset[i] = offset[i];
    }
    gyro_cal.calibrated = true;
}

void getGyroCalibration(float offset[3]) {
    for (int i = 0; i < 3; i++) {
        offset[i] = gyro_cal.offset[i];
    }
}

void enableGyroCalibration(bool enable) { gyro_cal.calibrated = enable; }

void resetGyroCalibration(void) { resetCalibration(1); }

// ===== Gyro Calibration Implementation =====
// Now uses the unified calibration framework

void startSimpleGyroCalibration(void) { startCalibration(1); }

bool isSimpleGyroCalibrationComplete(void) {
    // Check if gyro calibration was completed (active_sensor would be -1 after completion)
    // Also check if it's not currently running
    return active_sensor != 1 && !calib_state.active && gyro_cal.calibrated;
}

// Get CW45 rotation flag for use in PID controller
bool needsCW45Rotation(void) { return needYaw45Overlay; }

#endif /* BSP_USING_IMU_CMD_CAL_GENERAL */
