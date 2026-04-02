#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>

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
    rt_uint32_t duration_s = imu_reader_thread_duration_ms() / 1000U;

    rt_kprintf("ACK cmd=noise_test_prepare received\r\n");
    rt_kprintf("NOISE_TEST detected=%d duration_s=%lu recording=%d\r\n",
               detected,
               (unsigned long)duration_s,
               imu_reader_thread_is_recording());
    rt_kprintf("RESULT cmd=noise_test_prepare status=%s\r\n",
               detected > 0 ? "ok" : "not_found");
    return (detected > 0) ? RT_EOK : -RT_ENOSYS;
}

int cmd_test_control_noise_start(rt_uint32_t duration_ms)
{
    int result;
    int detected = imu_reader_thread_probe_count();
    rt_uint32_t duration_s = duration_ms / 1000U;

    rt_kprintf("ACK cmd=noise_test_start received\r\n");
    if (detected <= 0)
    {
        rt_kprintf("RESULT cmd=noise_test_start status=not_found detected=0\r\n");
        return -RT_ENOSYS;
    }

    result = session_start_by_name_with_duration("arw", duration_ms);
    rt_kprintf("RESULT cmd=noise_test_start status=%s detected=%d duration_s=%lu dir=%s file=%s index=%lu\r\n",
               result == RT_EOK ? "accepted" : "error",
               detected,
               (unsigned long)duration_s,
               imu_reader_thread_output_dir(),
               imu_reader_thread_output_path(),
               (unsigned long)imu_reader_thread_output_index());
    return result;
}

int cmd_test_control_noise_status(void)
{
    int recording = imu_reader_thread_is_recording();
    int last_error = imu_reader_thread_last_error();
    int last_run_ok = imu_reader_thread_last_run_ok();

    rt_kprintf("ACK cmd=noise_test_status received\r\n");
    rt_kprintf("NOISE_TEST_STATUS recording=%d frames=%lu duration_s=%lu file=%s last_error=%d last_run_ok=%d\r\n",
               recording,
               (unsigned long)imu_reader_thread_recorded_frames(),
               (unsigned long)(imu_reader_thread_duration_ms() / 1000U),
               imu_reader_thread_output_path(),
               last_error,
               last_run_ok);
    rt_kprintf("RESULT cmd=noise_test_status status=%s\r\n",
               ((recording == 0) && (last_error != RT_EOK)) ? "error" : "ok");
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
    rt_uint32_t minutes = 0U;
    rt_uint32_t seconds = 10U;
    rt_uint32_t total_seconds;

    if (argc > 1)
    {
        long value = atol(argv[1]);

        if (value < 0)
        {
            rt_kprintf("RESULT cmd=noise_test_start status=invalid_minutes\r\n");
            return -RT_EINVAL;
        }
        minutes = (rt_uint32_t)value;
    }

    if (argc > 2)
    {
        long value = atol(argv[2]);

        if ((value < 0) || (value > 59))
        {
            rt_kprintf("RESULT cmd=noise_test_start status=invalid_seconds\r\n");
            return -RT_EINVAL;
        }
        seconds = (rt_uint32_t)value;
    }

    total_seconds = (minutes * 60U) + seconds;
    if (total_seconds == 0U)
    {
        rt_kprintf("RESULT cmd=noise_test_start status=invalid_duration\r\n");
        return -RT_EINVAL;
    }

    return cmd_test_control_noise_start(total_seconds * 1000U);
}
MSH_CMD_EXPORT(noise_test_start, start imu noise test recording: noise_test_start [minutes] [seconds]);

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
