#include <rtthread.h>

#include "imu_reader_thread.h"
#include "test_arw.h"

int test_arw_start(void)
{
    return imu_reader_thread_start_for_test("arw");
}

int test_arw_start_with_duration(rt_uint32_t duration_ms)
{
    int result = imu_reader_thread_set_duration_ms(duration_ms);

    if (result != RT_EOK)
    {
        return result;
    }

    return imu_reader_thread_start_for_test("arw");
}

int test_arw_stop(void)
{
    return imu_reader_thread_stop();
}
