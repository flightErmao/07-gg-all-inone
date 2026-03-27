#include <rtdevice.h>
#include <rtthread.h>

extern "C" {
#include "imu.h"
}

#include "../../../../driverframework/L2_device/01_IMU/05_ICM42688_new/inc/imu/IMU.hpp"

namespace {

constexpr const char *kImuDeviceName = "ICM42688";
constexpr const char *kThreadName = "imu_dbg";
constexpr rt_uint32_t kThreadStackSize = 4096;
constexpr rt_uint8_t kThreadPriority = 18;
constexpr rt_uint32_t kThreadTick = 10;

static void imu_reader_entry(void *parameter)
{
    RT_UNUSED(parameter);

    rt_device_t imu_dev = rt_device_find(kImuDeviceName);
    if (imu_dev == RT_NULL)
    {
        rt_kprintf("[imu_dbg] device %s not found\n", kImuDeviceName);
        return;
    }

    rt_err_t err = rt_device_open(imu_dev, RT_DEVICE_OFLAG_RDONLY);
    if (err != RT_EOK)
    {
        rt_kprintf("[imu_dbg] open %s failed: %d\n", kImuDeviceName, err);
        return;
    }

    rt_kprintf("[imu_dbg] start reading %s\n", kImuDeviceName);

    while (1)
    {
        drvf::IMURawData raw_data{};
        // rt_ssize_t read_count = 
        rt_device_read(imu_dev, IMU_POS_ACC_GYRO, &raw_data, sizeof(raw_data));

        // if (read_count > 0)
        // {
        //     rt_kprintf(
        //         "[imu_dbg] ts=%llu valid=%d "
        //         "acc_raw=[%d,%d,%d] gyro_raw=[%d,%d,%d] temp_raw=%d "
        //         "acc=[%d.%03d,%d.%03d,%d.%03d] "
        //         "gyro=[%d.%03d,%d.%03d,%d.%03d] temp=%d.%03d fifo=%u packets=%u\n",
        //         raw_data.timestamp_us,
        //         raw_data.vaild ? 1 : 0,
        //         raw_data.raw_accel[0], raw_data.raw_accel[1], raw_data.raw_accel[2],
        //         raw_data.raw_gyro[0], raw_data.raw_gyro[1], raw_data.raw_gyro[2],
        //         raw_data.raw_temp,
        //         (int)raw_data.accel[0], (int)(raw_data.accel[0] >= 0 ? raw_data.accel[0] * 1000 : -raw_data.accel[0] * 1000) % 1000,
        //         (int)raw_data.accel[1], (int)(raw_data.accel[1] >= 0 ? raw_data.accel[1] * 1000 : -raw_data.accel[1] * 1000) % 1000,
        //         (int)raw_data.accel[2], (int)(raw_data.accel[2] >= 0 ? raw_data.accel[2] * 1000 : -raw_data.accel[2] * 1000) % 1000,
        //         (int)raw_data.gyro[0], (int)(raw_data.gyro[0] >= 0 ? raw_data.gyro[0] * 1000 : -raw_data.gyro[0] * 1000) % 1000,
        //         (int)raw_data.gyro[1], (int)(raw_data.gyro[1] >= 0 ? raw_data.gyro[1] * 1000 : -raw_data.gyro[1] * 1000) % 1000,
        //         (int)raw_data.gyro[2], (int)(raw_data.gyro[2] >= 0 ? raw_data.gyro[2] * 1000 : -raw_data.gyro[2] * 1000) % 1000,
        //         (int)raw_data.temperature,
        //         (int)(raw_data.temperature >= 0 ? raw_data.temperature * 1000 : -raw_data.temperature * 1000) % 1000,
        //         raw_data.fifo_count,
        //         raw_data.packet_count);
        // }
        // else
        // {
        //     rt_kprintf("[imu_dbg] read %s failed: %d\n", kImuDeviceName, (int)read_count);
        // }

        rt_thread_mdelay(1);
    }
}

} // namespace

extern "C" int imu_reader_thread_start(void)
{
    rt_thread_t tid = rt_thread_create(kThreadName,
                                       imu_reader_entry,
                                       RT_NULL,
                                       kThreadStackSize,
                                       kThreadPriority,
                                       kThreadTick);

    if (tid == RT_NULL)
    {
        rt_kprintf("[imu_dbg] create thread failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(tid);
    return RT_EOK;
}
