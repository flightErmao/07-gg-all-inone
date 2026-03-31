#include "ICM45686.hpp"

extern "C" {
#include "imu.h"
#include "rtconfig.h"
#include "rtdevice.h"
#include "rtthread.h"
}

#undef LOG_TAG
#define LOG_TAG "icm45686_hal"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif
#include <ulog.h>

namespace {

static int8_t icm45686_read_data(imu_dev_t dev, rt_off_t pos, void *data, rt_size_t size) {
  RT_UNUSED(pos);

  if (dev == RT_NULL || data == RT_NULL || size < sizeof(drvf::IMURawData)) {
    return -RT_EINVAL;
  }

  auto *imu = static_cast<drvf::ICM45686 *>(dev->parent.user_data);
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

static rt_err_t icm45686_control(imu_dev_t dev, int cmd, void *arg) {
  RT_UNUSED(dev);
  RT_UNUSED(cmd);
  RT_UNUSED(arg);
  return -RT_ENOSYS;
}

static const struct imu_ops g_icm45686_dev_ops = {
    .imu_config = RT_NULL,
    .imu_control = icm45686_control,
    .imu_read = icm45686_read_data,
};

static imu_configure_t g_icm45686_cfg = {
    1000,
    IMU_GYRO_MODE_NORMAL,
    1000,
    IMU_ACC_MODE_NORMAL,
    GYRO_SCALE_2000DPS,
    ACC_SCALE_16G,
    IMU_TEMP_SCALE,
    IMU_TEMP_OFFSET,
};

#ifdef BSP_USING_ICM45686
static const drvf::ICM45686HwConfig g_icm45686_hw_1 = {
    SENSOR_NAME_ICM45686,
    SENSOR_SPI_NAME_ICM45686,
    SENSOR_SPI_SLAVE_NAME_ICM45686,
    SENSOR_ICM45686_SPI_CS_PIN,
    SENSOR_ICM45686_SPI_MAX_HZ,
    SENSOR_ICM45686_WHOAMI_REG,
    SENSOR_ICM45686_WHOAMI_EXPECTED,
};
static drvf::ICM45686 g_icm45686_1(3, 0, g_icm45686_hw_1);
static struct imu_device g_icm45686_dev_1 = {
    .ops = &g_icm45686_dev_ops,
    .config = g_icm45686_cfg,
};
#endif

#ifdef BSP_USING_ICM45686_2ND
static const drvf::ICM45686HwConfig g_icm45686_hw_2 = {
    SENSOR_NAME_ICM45686_2ND,
    SENSOR_SPI_NAME_ICM45686_2ND,
    SENSOR_SPI_SLAVE_NAME_ICM45686_2ND,
    SENSOR_ICM45686_SPI_CS_PIN_2ND,
    SENSOR_ICM45686_SPI_MAX_HZ_2ND,
    SENSOR_ICM45686_WHOAMI_REG_2ND,
    SENSOR_ICM45686_WHOAMI_EXPECTED_2ND,
};
static drvf::ICM45686 g_icm45686_2(4, 0, g_icm45686_hw_2);
static struct imu_device g_icm45686_dev_2 = {
    .ops = &g_icm45686_dev_ops,
    .config = g_icm45686_cfg,
};
#endif

static rt_err_t icm45686_register_one(struct imu_device *dev, drvf::ICM45686 *imu, const char *name) {
  const int ret = imu->DebugInit();
  if (ret != 0) {
    LOG_W("device %s registered, deferred init ret=%d", name, ret);
    return RT_ERROR;
  }

  return hal_imu_register(dev, name, RT_DEVICE_FLAG_RDWR, imu);
}

}  // namespace

extern "C" {

static int icm45686_init_auto(void) {
  rt_err_t err = RT_EOK;

#ifdef BSP_USING_ICM45686
  if (icm45686_register_one(&g_icm45686_dev_1, &g_icm45686_1, SENSOR_NAME_ICM45686) != RT_EOK) {
    err = RT_ERROR;
  }
#endif

#ifdef BSP_USING_ICM45686_2ND
  if (icm45686_register_one(&g_icm45686_dev_2, &g_icm45686_2, SENSOR_NAME_ICM45686_2ND) != RT_EOK) {
    err = RT_ERROR;
  }
#endif

  return err;
}

#if defined(BSP_USING_ICM45686) || defined(BSP_USING_ICM45686_2ND)
INIT_ENV_EXPORT(icm45686_init_auto);
#endif

}
