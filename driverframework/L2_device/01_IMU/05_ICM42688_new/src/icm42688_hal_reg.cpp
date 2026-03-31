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

static int8_t icm42688_read_data(imu_dev_t dev, rt_off_t pos, void *data, rt_size_t size) {
  RT_UNUSED(pos);

  if (dev == RT_NULL || data == RT_NULL || size < sizeof(drvf::IMURawData)) {
    return -RT_EINVAL;
  }

  auto *imu = static_cast<drvf::ICM42688 *>(dev->parent.user_data);
  if (imu == nullptr) {
    return -1;
  }

  drvf::IMURawData raw_data{};
  if (!imu->ReadRaw(raw_data)) {
    LOG_W("device %s read failed", dev->parent.parent.name);
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

static const struct imu_ops g_icm42688_dev_ops = {
    .imu_config = RT_NULL,
    .imu_control = icm42688_control,
    .imu_read = icm42688_read_data,
};

static imu_configure_t g_icm42688_cfg = {
    1000,
    IMU_GYRO_MODE_NORMAL,
    1000,
    IMU_ACC_MODE_NORMAL,
    GYRO_SCALE_2000DPS,
    ACC_SCALE_16G,
    IMU_TEMP_SCALE,
    IMU_TEMP_OFFSET,
};

#ifdef BSP_USING_ICM42688_NEW
static const drvf::ICM42688HwConfig g_icm42688_hw_1 = {
    SENSOR_NAME_ICM42688,
    SENSOR_SPI_NAME_ICM42688,
    SENSOR_SPI_SLAVE_NAME_ICM42688,
    SENSOR_ICM42688_SPI_CS_PIN,
    SENSOR_ICM42688_SPI_MAX_HZ,
};
static drvf::ICM42688 g_icm42688_1(0, 0, g_icm42688_hw_1);
static struct imu_device g_icm42688_dev_1 = {
    .ops = &g_icm42688_dev_ops,
    .config = g_icm42688_cfg,
};
#endif

#ifdef BSP_USING_ICM42688_NEW_2ND
static const drvf::ICM42688HwConfig g_icm42688_hw_2 = {
    SENSOR_NAME_ICM42688_2ND,
    SENSOR_SPI_NAME_ICM42688_2ND,
    SENSOR_SPI_SLAVE_NAME_ICM42688_2ND,
    SENSOR_ICM42688_SPI_CS_PIN_2ND,
    SENSOR_ICM42688_SPI_MAX_HZ_2ND,
};
static drvf::ICM42688 g_icm42688_2(1, 0, g_icm42688_hw_2);
static struct imu_device g_icm42688_dev_2 = {
    .ops = &g_icm42688_dev_ops,
    .config = g_icm42688_cfg,
};
#endif

static rt_err_t icm42688_register_one(struct imu_device *dev, drvf::ICM42688 *imu, const char *name) {
  const int ret = imu->DebugInit(false);
  if (ret != 0) {
    LOG_W("device %s registered, deferred init ret=%d", name, ret);
    return RT_ERROR;
  }

  return hal_imu_register(dev, name, RT_DEVICE_FLAG_RDWR, imu);
}

}  // namespace

extern "C" {

static int icm42688_init_auto(void) {
  rt_err_t err = RT_EOK;

#ifdef BSP_USING_ICM42688_NEW
  if (icm42688_register_one(&g_icm42688_dev_1, &g_icm42688_1, SENSOR_NAME_ICM42688) != RT_EOK) {
    err = RT_ERROR;
  }
#endif

#ifdef BSP_USING_ICM42688_NEW_2ND
  if (icm42688_register_one(&g_icm42688_dev_2, &g_icm42688_2, SENSOR_NAME_ICM42688_2ND) != RT_EOK) {
    err = RT_ERROR;
  }
#endif

  return err;
}

#if defined(BSP_USING_ICM42688_NEW) || defined(BSP_USING_ICM42688_NEW_2ND)
INIT_ENV_EXPORT(icm42688_init_auto);
#endif

}
