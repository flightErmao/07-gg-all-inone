#include <rtthread.h>
#include <finsh.h>

#include "03_cmd_test_control.h"
#include "imu_reader_thread.h"
#include "session_manager.h"

int cmd_test_control_start(const char *test_name)
{
    return session_start_by_name(test_name);
}

int cmd_test_control_stop(void)
{
    return session_stop();
}

int cmd_test_control_mark_temperature(rt_int32_t temp_c)
{
    return session_mark_temperature(temp_c);
}

int cmd_test_control_noise_prepare(void)
{
    int detected = imu_reader_thread_probe_count();

    rt_kprintf("ACK cmd=noise_test_prepare received\r\n");
    rt_kprintf("NOISE_TEST detected=%d duration_s=10 recording=%d\r\n",
               detected,
               imu_reader_thread_is_recording());
    rt_kprintf("RESULT cmd=noise_test_prepare status=%s\r\n",
               detected > 0 ? "ok" : "not_found");
    return (detected > 0) ? RT_EOK : -RT_ENOSYS;
}

int cmd_test_control_noise_start(void)
{
    int result;
    int detected = imu_reader_thread_probe_count();

    rt_kprintf("ACK cmd=noise_test_start received\r\n");
    if (detected <= 0)
    {
        rt_kprintf("RESULT cmd=noise_test_start status=not_found detected=0\r\n");
        return -RT_ENOSYS;
    }

    result = cmd_test_control_start("arw");
    rt_kprintf("RESULT cmd=noise_test_start status=%s detected=%d duration_s=10 dir=%s file=%s index=%lu\r\n",
               result == RT_EOK ? "accepted" : "error",
               detected,
               imu_reader_thread_output_dir(),
               imu_reader_thread_output_path(),
               (unsigned long)imu_reader_thread_output_index());
    return result;
}

int cmd_test_control_noise_status(void)
{
    rt_kprintf("ACK cmd=noise_test_status received\r\n");
    rt_kprintf("NOISE_TEST_STATUS recording=%d frames=%lu duration_s=%lu dir=%s file=%s index=%lu flushes=%lu max_gap_ms=%lu\r\n",
               imu_reader_thread_is_recording(),
               (unsigned long)imu_reader_thread_recorded_frames(),
               (unsigned long)(imu_reader_thread_duration_ms() / 1000U),
               imu_reader_thread_output_dir(),
               imu_reader_thread_output_path(),
               (unsigned long)imu_reader_thread_output_index(),
               (unsigned long)imu_reader_thread_flush_count(),
               (unsigned long)imu_reader_thread_max_gap_ms());
    rt_kprintf("RESULT cmd=noise_test_status status=ok\r\n");
    return RT_EOK;
}

int cmd_test_control_noise_stop(void)
{
    int result = cmd_test_control_stop();

    rt_kprintf("ACK cmd=noise_test_stop received\r\n");
    rt_kprintf("RESULT cmd=noise_test_stop status=%s frames=%lu dir=%s file=%s index=%lu\r\n",
               result == RT_EOK ? "ok" : "error",
               (unsigned long)imu_reader_thread_recorded_frames(),
               imu_reader_thread_output_dir(),
               imu_reader_thread_output_path(),
               (unsigned long)imu_reader_thread_output_index());
    return result;
}

static int noise_test_prepare(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_test_control_noise_prepare();
}
MSH_CMD_EXPORT(noise_test_prepare, prepare 10s imu noise test);

static int noise_test_start(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_test_control_noise_start();
}
MSH_CMD_EXPORT(noise_test_start, start 10s imu noise test recording);

static int noise_test_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_test_control_noise_status();
}
MSH_CMD_EXPORT(noise_test_status, get 10s imu noise test progress);

static int noise_test_stop(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return cmd_test_control_noise_stop();
}
MSH_CMD_EXPORT(noise_test_stop, stop 10s imu noise test recording);
