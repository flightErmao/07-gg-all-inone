#include <rtthread.h>

#include "imu_reader_thread.h"
#include "test_arw.h"

int test_arw_start(void)
{
    return imu_reader_thread_start_for_test("arw");
}

int test_arw_stop(void)
{
    return imu_reader_thread_stop();
}
