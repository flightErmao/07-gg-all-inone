#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "rtconfig.h"

#include "../../TASK_01_SENSOR/inc/sensorsTypes.h"
#include "../../../../../L2_device/01_IMU/05_BMI270_px4/inc/taskImuPub_mcn.h"
#include "../../../../../L3_peripheral/22_timConfig/inc/timestamp.h"
#include "../inc/param.h"
#include "../inc/imuCaliParam.h"

#define LOG_TAG "cmd.imu_cali"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#define IMU_CALI_SAMPLE_COUNT 256
#define IMU_CALI_ACC_LEVEL_XY_MAX_G 0.05f
#define IMU_CALI_ACC_LEVEL_Z_TOL_G 0.10f
#define IMU_CALI_GYRO_EPS_DPS 1.0f
#define IMU_CALI_SAMPLE_WAIT_TICKS rt_tick_from_millisecond(5)

static const float kAccGPerLsb = (2.0f * 16.0f) / 65536.0f;
static const float kGyroDegPerLsb = (2.0f * 2000.0f) / 65536.0f;

static rt_err_t imu_cali_collect(float acc_avg[3], float gyro_avg[3]) {
    float acc_sum[3] = {0.0f};
    float gyro_sum[3] = {0.0f};

    for (uint16_t i = 0; i < IMU_CALI_SAMPLE_COUNT; i++) {
        px4ImuMcnWait();

        sensorData_t sample = {0};
        if (px4ImuMcnAcquire(&sample) != RT_EOK) {
            LOG_E("failed to acquire imu sample");
            return -RT_ERROR;
        }

        acc_sum[0] += (float)sample.acc_raw.x;
        acc_sum[1] += (float)sample.acc_raw.y;
        acc_sum[2] += (float)sample.acc_raw.z;

        gyro_sum[0] += (float)sample.gyro_raw.x;
        gyro_sum[1] += (float)sample.gyro_raw.y;
        gyro_sum[2] += (float)sample.gyro_raw.z;

        rt_thread_delay(IMU_CALI_SAMPLE_WAIT_TICKS);
    }

    acc_avg[0] = acc_sum[0] / (float)IMU_CALI_SAMPLE_COUNT;
    acc_avg[1] = acc_sum[1] / (float)IMU_CALI_SAMPLE_COUNT;
    acc_avg[2] = acc_sum[2] / (float)IMU_CALI_SAMPLE_COUNT;

    gyro_avg[0] = gyro_sum[0] / (float)IMU_CALI_SAMPLE_COUNT;
    gyro_avg[1] = gyro_sum[1] / (float)IMU_CALI_SAMPLE_COUNT;
    gyro_avg[2] = gyro_sum[2] / (float)IMU_CALI_SAMPLE_COUNT;

    return RT_EOK;
}

static rt_err_t imu_cali_verify_level(const float acc_avg[3], const float gyro_avg[3],
                                      float acc_bias_g[3], float gyro_bias_dps[3]) {
    float acc_g[3] = {
        acc_avg[0] * kAccGPerLsb,
        acc_avg[1] * kAccGPerLsb,
        acc_avg[2] * kAccGPerLsb,
    };

    float gyro_dps[3] = {
        gyro_avg[0] * kGyroDegPerLsb,
        gyro_avg[1] * kGyroDegPerLsb,
        gyro_avg[2] * kGyroDegPerLsb,
    };

    float horiz_g = sqrtf(acc_g[0] * acc_g[0] + acc_g[1] * acc_g[1]);
    float z_err_g = fabsf(acc_g[2] - 1.0f);
    float gyro_max = fmaxf(fabsf(gyro_dps[0]), fmaxf(fabsf(gyro_dps[1]), fabsf(gyro_dps[2])));

    if (horiz_g > IMU_CALI_ACC_LEVEL_XY_MAX_G || z_err_g > IMU_CALI_ACC_LEVEL_Z_TOL_G) {
        LOG_E("imu not level: ax=%.3fg ay=%.3fg az=%.3fg", acc_g[0], acc_g[1], acc_g[2]);
        return -RT_ERROR;
    }

    if (gyro_max > IMU_CALI_GYRO_EPS_DPS) {
        LOG_E("imu still moving: gx=%.3f°/s gy=%.3f°/s gz=%.3f°/s",
              gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        return -RT_ERROR;
    }

    gyro_bias_dps[0] = gyro_dps[0];
    gyro_bias_dps[1] = gyro_dps[1];
    gyro_bias_dps[2] = gyro_dps[2];

    acc_bias_g[0] = acc_g[0];
    acc_bias_g[1] = acc_g[1];
    acc_bias_g[2] = acc_g[2] - 1.0f;

    return RT_EOK;
}

static rt_err_t imu_cali_store(const float gyro_bias_dps[3], const float acc_bias_g[3]) {
    float gyro_bias[3] = {gyro_bias_dps[0], gyro_bias_dps[1], gyro_bias_dps[2]};
    float acc_bias[3] = {acc_bias_g[0], acc_bias_g[1], acc_bias_g[2]};
    uint32_t timestamp_ms = timestamp_micros() / 1000U;
    uint32_t status = 1;

    if (setParam(IMU_CALI_PARAM_GYRO_BIAS, gyro_bias, sizeof(gyro_bias)) != RT_EOK) {
        LOG_E("store gyro bias failed");
        return -RT_ERROR;
    }

    if (setParam(IMU_CALI_PARAM_ACC_BIAS, acc_bias, sizeof(acc_bias)) != RT_EOK) {
        LOG_E("store acc bias failed");
        return -RT_ERROR;
    }

    if (setParam(IMU_CALI_PARAM_TIMESTAMP, &timestamp_ms, sizeof(timestamp_ms)) != RT_EOK) {
        LOG_E("store timestamp failed");
        return -RT_ERROR;
    }

    if (setParam(IMU_CALI_PARAM_STATUS, &status, sizeof(status)) != RT_EOK) {
        LOG_E("store status failed");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void cmd_imu_cali(int argc, char **argv) {
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    int init_ret = px4ImuMcnInit();
    if (init_ret != RT_EOK && init_ret != 0) {
        LOG_E("imu mcn init failed: %d", init_ret);
        return;
    }

    float acc_avg[3];
    float gyro_avg[3];

    if (imu_cali_collect(acc_avg, gyro_avg) != RT_EOK) {
        LOG_E("failed to collect imu samples");
        return;
    }

    float acc_bias_g[3];
    float gyro_bias_dps[3];
    if (imu_cali_verify_level(acc_avg, gyro_avg, acc_bias_g, gyro_bias_dps) != RT_EOK) {
        LOG_E("imu conditions not suitable for calibration");
        return;
    }

    if (imu_cali_store(gyro_bias_dps, acc_bias_g) != RT_EOK) {
        LOG_E("failed to store calibration parameters");
        return;
    }

    LOG_I("IMU calibration saved.");
    LOG_I("gyro bias: [%.4f, %.4f, %.4f] deg/s", gyro_bias_dps[0], gyro_bias_dps[1], gyro_bias_dps[2]);
    LOG_I("acc bias : [%.5f, %.5f, %.5f] g (z includes gravity offset)", acc_bias_g[0], acc_bias_g[1], acc_bias_g[2]);
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT_ALIAS(cmd_imu_cali, imu_cali, Calibrate IMU biases when level);
#endif


