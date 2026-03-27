#include <rtthread.h>
#include <stdlib.h>

#include "fatfs_sdcard_port.h"

#define APP_FAKE_IMU_TEST_LINES_DEF   64U
#define APP_FAKE_IMU_TEST_LINES_MAX   4096U

int fake_imu_sd_test(int argc, char **argv)
{
    rt_uint32_t lines = APP_FAKE_IMU_TEST_LINES_DEF;
    rt_bool_t overwrite = RT_TRUE;
    rt_uint32_t written_lines = 0;
    rt_err_t result;

    if (argc > 1)
    {
        long value = atol(argv[1]);

        if ((value <= 0) || (value > (long)APP_FAKE_IMU_TEST_LINES_MAX))
        {
            rt_kprintf("RESULT cmd=fake_imu_sd_test status=error reason=invalid_lines min=1 max=%lu\r\n",
                       (unsigned long)APP_FAKE_IMU_TEST_LINES_MAX);
            return -RT_ERROR;
        }

        lines = (rt_uint32_t)value;
    }

    if (argc > 2)
    {
        overwrite = (atoi(argv[2]) == 0) ? RT_FALSE : RT_TRUE;
    }

    rt_kprintf("ACK cmd=fake_imu_sd_test received lines=%lu overwrite=%d\r\n",
               (unsigned long)lines,
               overwrite);

    if (fatfs_sdcard_mount() != RT_EOK)
    {
        rt_kprintf("RESULT cmd=fake_imu_sd_test status=error reason=mount_failed\r\n");
        return -RT_ERROR;
    }

    result = fatfs_sdcard_write_fake_imu_test(lines, overwrite, &written_lines);
    if (result != RT_EOK)
    {
        rt_kprintf("RESULT cmd=fake_imu_sd_test status=error reason=write_failed written=%lu file=IMU_TEST.CSV\r\n",
                   (unsigned long)written_lines);
        return result;
    }

    rt_kprintf("RESULT cmd=fake_imu_sd_test status=ok file=IMU_TEST.CSV lines=%lu overwrite=%d\r\n",
               (unsigned long)written_lines,
               overwrite);
    return RT_EOK;
}
MSH_CMD_EXPORT(fake_imu_sd_test, write fake imu data into IMU_TEST.CSV);
