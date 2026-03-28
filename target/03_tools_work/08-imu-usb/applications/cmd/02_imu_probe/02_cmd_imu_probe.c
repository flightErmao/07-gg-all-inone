#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>

#include <rtconfig.h>

#include "02_cmd_imu_probe.h"

/* Reserve names for future 2~4 IMU instances while keeping IMU1 bound to the
 * current real device registration. */
#ifndef SENSOR_NAME_IMU1
#define SENSOR_NAME_IMU1 SENSOR_NAME_ICM42688
#endif

#ifndef SENSOR_NAME_IMU2
#define SENSOR_NAME_IMU2 "ICM42688_2"
#endif

#ifndef SENSOR_NAME_IMU3
#define SENSOR_NAME_IMU3 "ICM42688_3"
#endif

#ifndef SENSOR_NAME_IMU4
#define SENSOR_NAME_IMU4 "ICM42688_4"
#endif

static const char *cmd_imu_probe_device_name(int imu_index)
{
    switch (imu_index)
    {
    case 1:
        return SENSOR_NAME_IMU1;
    case 2:
        return SENSOR_NAME_IMU2;
    case 3:
        return SENSOR_NAME_IMU3;
    case 4:
        return SENSOR_NAME_IMU4;
    default:
        return RT_NULL;
    }
}

static int cmd_imu_probe_report_one(int imu_index)
{
    const char *device_name = cmd_imu_probe_device_name(imu_index);
    rt_device_t device;

    if (device_name == RT_NULL)
    {
        rt_kprintf("ACK cmd=imu_probe%d received\r\n", imu_index);
        rt_kprintf("IMU_PROBE imu=%d status=invalid_index\r\n", imu_index);
        rt_kprintf("RESULT cmd=imu_probe%d status=invalid_index\r\n", imu_index);
        return -RT_ERROR;
    }

    device = rt_device_find(device_name);

    rt_kprintf("ACK cmd=imu_probe%d received\r\n", imu_index);
    rt_kprintf("IMU_PROBE imu=%d name=%s status=%s\r\n",
               imu_index,
               device_name,
               device != RT_NULL ? "ready" : "not_found");
    rt_kprintf("RESULT cmd=imu_probe%d status=%s\r\n",
               imu_index,
               device != RT_NULL ? "ok" : "not_found");
    return (device != RT_NULL) ? RT_EOK : -RT_ENOSYS;
}

int cmd_imu_probe_exec(int imu_index)
{
    return cmd_imu_probe_report_one(imu_index);
}

int cmd_imu_probe_all(void)
{
    int imu_index;
    int ok_count = 0;

    rt_kprintf("ACK cmd=imu_probe_all received\r\n");

    for (imu_index = 1; imu_index <= 4; imu_index++)
    {
        const char *device_name = cmd_imu_probe_device_name(imu_index);
        rt_device_t device = (device_name != RT_NULL) ? rt_device_find(device_name) : RT_NULL;

        if (device != RT_NULL)
        {
            ok_count++;
        }

        rt_kprintf("IMU_PROBE imu=%d name=%s status=%s\r\n",
                   imu_index,
                   device_name != RT_NULL ? device_name : "unknown",
                   device != RT_NULL ? "ready" : "not_found");
    }

    rt_kprintf("RESULT cmd=imu_probe_all status=ok found=%d total=4\r\n", ok_count);
    return RT_EOK;
}

static int imu_probe1(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_imu_probe_exec(1);
}
MSH_CMD_EXPORT(imu_probe1, probe imu1 by registered device name);

static int imu_probe2(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_imu_probe_exec(2);
}
MSH_CMD_EXPORT(imu_probe2, probe imu2 by registered device name);

static int imu_probe3(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_imu_probe_exec(3);
}
MSH_CMD_EXPORT(imu_probe3, probe imu3 by registered device name);

static int imu_probe4(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_imu_probe_exec(4);
}
MSH_CMD_EXPORT(imu_probe4, probe imu4 by registered device name);

static int imu_probe_all(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_imu_probe_all();
}
MSH_CMD_EXPORT(imu_probe_all, probe all imu slots);
