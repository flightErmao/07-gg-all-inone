#include "ICM42688.hpp"

extern "C" {
#include "imu.h"
#include "rtconfig.h"
#include "rtdevice.h"
#include "rtthread.h"
}

#undef LOG_TAG
#define LOG_TAG "icm42688_hal"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif
#include <ulog.h>

namespace {

drvf::ICM42688 g_icm42688(0, 0);

static int8_t icm42688_read_data(imu_dev_t imu, rt_off_t pos, void *data, rt_size_t size) {
  RT_UNUSED(imu);
  RT_UNUSED(pos);

  if (data == RT_NULL || size < sizeof(drvf::IMURawData)) {
    return -RT_EINVAL;
  }

  drvf::IMURawData raw_data{};
  if (!g_icm42688.ReadRaw(raw_data)) {
    LOG_W("device %s read failed", SENSOR_NAME_ICM42688);
    return -1;
  }

  rt_memcpy(data, &raw_data, sizeof(raw_data));
  return 1;
}

static rt_err_t icm42688_control(imu_dev_t dev, int cmd, void *arg) {
  RT_UNUSED(dev);
  RT_UNUSED(cmd);
  RT_UNUSED(arg);
  return -RT_ENOSYS;
}

static const struct imu_ops icm42688_dev_ops = {
    .imu_config = RT_NULL,
    .imu_control = icm42688_control,
    .imu_read = icm42688_read_data,
};

static struct imu_device icm42688_dev = {
    .ops = &icm42688_dev_ops,
    .config =
        {
            1000,
            IMU_GYRO_MODE_NORMAL,
            1000,
            IMU_ACC_MODE_NORMAL,
            GYRO_SCALE_2000DPS,
            ACC_SCALE_16G,
            IMU_TEMP_SCALE,
            IMU_TEMP_OFFSET,
        },
};

static rt_err_t icm42688_hal_init(const char *imu_name) {
  const int ret = g_icm42688.DebugInit(false);
  if (ret != 0) {
    LOG_W("device %s registered, deferred init ret=%d", imu_name, ret);
    return RT_ERROR;
  }

  rt_err_t err = hal_imu_register(&icm42688_dev, imu_name, RT_DEVICE_FLAG_RDWR, RT_NULL);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

}  // namespace

extern "C" {

static int icm42688_init_auto(void) { return icm42688_hal_init(SENSOR_NAME_ICM42688); }

int icm42688_force_debug_init(void) { return g_icm42688.DebugInit(false); }

#ifdef BSP_USING_ICM42688_NEW
INIT_ENV_EXPORT(icm42688_init_auto);
#endif

}
