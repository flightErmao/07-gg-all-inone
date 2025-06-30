#include "imu.h"

static rt_err_t hal_imu_init(struct rt_device* dev)
{
    rt_err_t ret = RT_EOK;
    imu_dev_t imu;

    RT_ASSERT(dev != RT_NULL);

    imu = (imu_dev_t)dev;

    /* apply configuration */
    if (imu->ops->imu_config) {
        ret = imu->ops->imu_config(imu, &imu->config);
    }

    return ret;
}

static rt_size_t hal_imu_read(struct rt_device* dev,
                              rt_off_t pos,
                              void* buffer,
                              rt_size_t size)
{
    rt_size_t rb = 0;
    imu_dev_t imu;

    RT_ASSERT(dev != RT_NULL);

    imu = (imu_dev_t)dev;

    if (imu->ops->imu_read && size) {
        rb = imu->ops->imu_read(imu, pos, buffer, size);
    }

    return rb;
}

rt_err_t hal_imu_register(imu_dev_t imu, const char* name, rt_uint32_t flag, void* data)
{
    rt_err_t ret;
    struct rt_device* device;

    RT_ASSERT(imu != RT_NULL);

    device = &(imu->parent);
    RT_ASSERT(imu != RT_NULL);

    device->type = RT_Device_Class_SPIDevice;
    device->ref_count = 0;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init = hal_imu_init;
    device->open = RT_NULL;
    device->close = RT_NULL;
    device->read = hal_imu_read;
    device->write = RT_NULL;
    device->control = RT_NULL;
    device->user_data = data;


    /* register a imu device */
    return rt_device_register(device, name, flag);
}