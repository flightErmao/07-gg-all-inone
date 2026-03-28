#include <rtthread.h>
#include <rtdevice.h>
#include <rtconfig.h>
#include <finsh.h>
#include <string.h>

extern "C" {
#include "fatfs_sdcard_port.h"
#include "file_naming.h"
#include "imu.h"
#include "session_manager.h"
#include "usb_mode_manager.h"
}

#include "imu/IMU.hpp"
#include "imu_reader_thread.h"

#define APP_IMU_RINGBUF_SIZE          (96U * 1024U)
#define APP_IMU_RINGBUF_HALF_SIZE     (APP_IMU_RINGBUF_SIZE / 2U)
#define APP_IMU_FRAME_MAGIC_HEAD      0x55AAU
#define APP_IMU_MAX_COUNT             4
#define APP_IMU_RECORD_DURATION_MS    (10U * 1000U)
#define APP_IMU_POLL_PERIOD_MS        2U
#define APP_IMU_OUTPUT_DIR_MAX_LEN    32U
#define APP_IMU_OUTPUT_PATH_MAX_LEN   64U

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

typedef struct imu_file_writer
{
    FIL file;
    rt_bool_t mounted_here;
    rt_uint32_t flush_count;
} imu_file_writer_t;

typedef struct imu_writer_runtime
{
    rt_thread_t thread;
    rt_sem_t sem;
    rt_uint8_t active_half;
    rt_uint32_t half_offset;
    rt_uint32_t half_bytes[2];
    rt_uint8_t ready_mask;
    rt_bool_t poll_finished;
    rt_bool_t running;
    rt_uint32_t frame_count;
    rt_uint32_t max_gap_ms;
    rt_tick_t last_poll_tick;
    imu_file_writer_t file_writer;
} imu_writer_runtime_t;

typedef struct imu_slot
{
    rt_device_t device;
    const char *name;
    rt_uint8_t index;
    rt_bool_t detected;
} imu_slot_t;

typedef struct imu_poll_ctx
{
    rt_thread_t poll_thread;
    rt_mutex_t lock;
    rt_bool_t started;
    rt_bool_t stop_requested;
    rt_bool_t recording;
    rt_bool_t last_run_ok;
    rt_uint32_t duration_ms;
    rt_uint32_t detected_count;
    rt_uint32_t recorded_frames;
    rt_uint32_t output_index;
    int last_error;
    char test_name[16];
    char output_dir[APP_IMU_OUTPUT_DIR_MAX_LEN];
    char output_path[APP_IMU_OUTPUT_PATH_MAX_LEN];
    imu_slot_t slots[APP_IMU_MAX_COUNT];
    imu_writer_runtime_t writer;
} imu_poll_ctx_t;

static rt_uint8_t g_imu_ringbuf[APP_IMU_RINGBUF_SIZE];
static imu_poll_ctx_t g_imu_poll_ctx;

enum
{
    APP_IMU_FRAME_SIZE = 32
};

static const char *imu_poll_slot_name(rt_uint8_t index)
{
    switch (index)
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

static void imu_poll_lock(void)
{
    rt_mutex_take(g_imu_poll_ctx.lock, RT_WAITING_FOREVER);
}

static void imu_poll_unlock(void)
{
    rt_mutex_release(g_imu_poll_ctx.lock);
}

static int imu_poll_ensure_init(void)
{
    if (g_imu_poll_ctx.started == RT_TRUE)
    {
        return RT_EOK;
    }

    g_imu_poll_ctx.lock = rt_mutex_create("imupoll", RT_IPC_FLAG_PRIO);
    if (g_imu_poll_ctx.lock == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    g_imu_poll_ctx.duration_ms = APP_IMU_RECORD_DURATION_MS;
    g_imu_poll_ctx.last_error = 0;
    g_imu_poll_ctx.writer.sem = rt_sem_create("imuwsem", 0, RT_IPC_FLAG_PRIO);
    if (g_imu_poll_ctx.writer.sem == RT_NULL)
    {
        rt_mutex_delete(g_imu_poll_ctx.lock);
        g_imu_poll_ctx.lock = RT_NULL;
        return -RT_ENOMEM;
    }
    rt_strncpy(g_imu_poll_ctx.test_name, "arw", sizeof(g_imu_poll_ctx.test_name) - 1U);
    g_imu_poll_ctx.test_name[sizeof(g_imu_poll_ctx.test_name) - 1U] = '\0';
    g_imu_poll_ctx.started = RT_TRUE;
    return RT_EOK;
}

static rt_uint8_t *imu_poll_half_base(rt_uint8_t half_index)
{
    return &g_imu_ringbuf[(rt_size_t)half_index * APP_IMU_RINGBUF_HALF_SIZE];
}

static rt_uint16_t imu_poll_crc16_ccitt(const void *data, rt_uint32_t length)
{
    const rt_uint8_t *p = (const rt_uint8_t *)data;
    rt_uint16_t crc = 0U;
    rt_uint32_t i;

    for (i = 0; i < length; ++i)
    {
        int bit;

        crc ^= (rt_uint16_t)p[i] << 8;
        for (bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (rt_uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void imu_poll_write_u16_le(rt_uint8_t *dst, rt_uint16_t value)
{
    dst[0] = (rt_uint8_t)(value & 0xFFU);
    dst[1] = (rt_uint8_t)((value >> 8) & 0xFFU);
}

static void imu_poll_write_u64_le(rt_uint8_t *dst, rt_uint64_t value)
{
    int index;

    for (index = 0; index < 8; ++index)
    {
        dst[index] = (rt_uint8_t)((value >> (index * 8)) & 0xFFU);
    }
}

static rt_int16_t imu_poll_read_s16_be(const rt_uint8_t *src)
{
    return (rt_int16_t)(((rt_uint16_t)src[0] << 8) | src[1]);
}

static rt_int16_t imu_poll_div_round_s32(rt_int32_t sum, rt_uint16_t count)
{
    if (count == 0U)
    {
        return 0;
    }

    if (sum >= 0)
    {
        return (rt_int16_t)((sum + ((rt_int32_t)count / 2)) / (rt_int32_t)count);
    }

    return (rt_int16_t)((sum - ((rt_int32_t)count / 2)) / (rt_int32_t)count);
}

static int imu_poll_writer_open(imu_file_writer_t *writer, rt_bool_t overwrite)
{
    rt_memset(writer, 0, sizeof(*writer));

    if (usb_mode_manager_current_mode() != USB_APP_MODE_CDC)
    {
        return -RT_ERROR;
    }

    if (fatfs_sdcard_is_mounted() != RT_TRUE)
    {
        if (fatfs_sdcard_mount() != RT_EOK)
        {
            return -RT_ERROR;
        }
        writer->mounted_here = RT_TRUE;
    }

    if ((g_imu_poll_ctx.output_path[0] == '\0') || (g_imu_poll_ctx.output_dir[0] == '\0'))
    {
        if (file_naming_make_next_bin_path(g_imu_poll_ctx.test_name,
                                           g_imu_poll_ctx.output_dir,
                                           sizeof(g_imu_poll_ctx.output_dir),
                                           g_imu_poll_ctx.output_path,
                                           sizeof(g_imu_poll_ctx.output_path),
                                           &g_imu_poll_ctx.output_index) != RT_EOK)
        {
            if (writer->mounted_here == RT_TRUE)
            {
                fatfs_sdcard_unmount();
            }
            return -RT_ERROR;
        }
    }

    if (fatfs_sdcard_open_raw_file(&writer->file, g_imu_poll_ctx.output_path, overwrite) != RT_EOK)
    {
        if (writer->mounted_here == RT_TRUE)
        {
            fatfs_sdcard_unmount();
        }
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void imu_poll_writer_close(imu_file_writer_t *writer)
{
    if (writer == RT_NULL)
    {
        return;
    }

    fatfs_sdcard_close_raw_file(&writer->file, RT_TRUE);

    if (writer->mounted_here == RT_TRUE)
    {
        fatfs_sdcard_unmount();
    }
}

static void imu_poll_writer_abort(imu_file_writer_t *writer)
{
    if (writer == RT_NULL)
    {
        return;
    }

    fatfs_sdcard_close_raw_file(&writer->file, RT_FALSE);

    if (writer->mounted_here == RT_TRUE)
    {
        fatfs_sdcard_unmount();
    }
}

static int imu_poll_queue_active_half_locked(void)
{
    imu_writer_runtime_t *writer = &g_imu_poll_ctx.writer;
    rt_uint8_t ready_bit;
    rt_uint8_t next_half;

    if (writer->half_offset == 0U)
    {
        return RT_EOK;
    }

    ready_bit = (rt_uint8_t)(1U << writer->active_half);
    if ((writer->ready_mask & ready_bit) != 0U)
    {
        return -RT_EFULL;
    }

    writer->half_bytes[writer->active_half] = writer->half_offset;
    writer->ready_mask = (rt_uint8_t)(writer->ready_mask | ready_bit);
    writer->half_offset = 0U;
    next_half = (rt_uint8_t)(writer->active_half ^ 1U);

    if ((writer->ready_mask & (1U << next_half)) != 0U)
    {
        return -RT_EFULL;
    }

    writer->active_half = next_half;
    rt_sem_release(writer->sem);
    return RT_EOK;
}

static int imu_poll_writer_push_frame(const rt_uint8_t *frame)
{
    imu_writer_runtime_t *writer = &g_imu_poll_ctx.writer;
    rt_uint8_t *half_base;

    if (frame == RT_NULL)
    {
        return -RT_EINVAL;
    }

    imu_poll_lock();

    if ((writer->half_offset + APP_IMU_FRAME_SIZE) > APP_IMU_RINGBUF_HALF_SIZE)
    {
        if (imu_poll_queue_active_half_locked() != RT_EOK)
        {
            imu_poll_unlock();
            return -RT_EFULL;
        }
    }

    half_base = imu_poll_half_base(writer->active_half);
    rt_memcpy(half_base + writer->half_offset, frame, APP_IMU_FRAME_SIZE);
    writer->half_offset += APP_IMU_FRAME_SIZE;
    writer->frame_count++;
    g_imu_poll_ctx.recorded_frames = writer->frame_count;

    if (writer->half_offset == APP_IMU_RINGBUF_HALF_SIZE)
    {
        if (imu_poll_queue_active_half_locked() != RT_EOK)
        {
            imu_poll_unlock();
            return -RT_EFULL;
        }
    }

    imu_poll_unlock();
    return RT_EOK;
}

static void imu_poll_refresh_slots(void)
{
    rt_uint32_t count = 0;
    int index;

    for (index = 0; index < APP_IMU_MAX_COUNT; ++index)
    {
        imu_slot_t *slot = &g_imu_poll_ctx.slots[index];

        slot->index = (rt_uint8_t)(index + 1);
        slot->name = imu_poll_slot_name(slot->index);
        slot->device = (slot->name != RT_NULL) ? rt_device_find(slot->name) : RT_NULL;
        slot->detected = (slot->device != RT_NULL) ? RT_TRUE : RT_FALSE;
        if (slot->detected == RT_TRUE)
        {
            rt_device_init(slot->device);
            if (slot->device->ref_count == 0)
            {
                (void)rt_device_open(slot->device, RT_DEVICE_OFLAG_RDONLY);
            }
            count++;
        }
    }

    g_imu_poll_ctx.detected_count = count;
}

static void imu_poll_build_frame(rt_uint8_t *frame,
                                 rt_uint8_t imu_index,
                                 rt_uint32_t seq,
                                 const drvf::IMURawData *raw_data)
{
    rt_uint8_t same_flag = 0U;
    rt_uint8_t fifo_header = 0U;
    rt_uint16_t packet_size = raw_data->packet_size;
    rt_uint16_t packet_count = 0U;
    rt_uint16_t fifo_timestamp = 0U;
    rt_int16_t avg_accel[3] = { 0, 0, 0 };
    rt_int16_t avg_gyro[3] = { 0, 0, 0 };
    rt_int16_t avg_temp = 0;
    rt_int32_t acc_sum[3] = { 0, 0, 0 };
    rt_int32_t gyro_sum[3] = { 0, 0, 0 };
    rt_int32_t temp_sum = 0;
    rt_int32_t header_sum = 0;
    rt_uint16_t parsed_count = 0U;
    rt_uint16_t i;
    rt_uint16_t j;

    if ((packet_size == 0U) || (packet_size > drvf::kImuMaxPacketSize))
    {
        packet_size = drvf::kImuMaxPacketSize;
    }
    packet_count = (rt_uint16_t)(raw_data->fifo_count / packet_size);
    if (packet_count > drvf::kImuMaxPacketCount)
    {
        packet_count = drvf::kImuMaxPacketCount;
    }

    for (i = 0; i < packet_count; ++i)
    {
        const rt_uint8_t *pi = &raw_data->fifo_data[(rt_size_t)i * packet_size];
        rt_int16_t acc_i[3];
        rt_int16_t gyro_i[3];

        if (packet_size < 14U)
        {
            break;
        }

        acc_i[0] = imu_poll_read_s16_be(&pi[0x01]);
        acc_i[1] = imu_poll_read_s16_be(&pi[0x03]);
        acc_i[2] = imu_poll_read_s16_be(&pi[0x05]);
        gyro_i[0] = imu_poll_read_s16_be(&pi[0x07]);
        gyro_i[1] = imu_poll_read_s16_be(&pi[0x09]);
        gyro_i[2] = imu_poll_read_s16_be(&pi[0x0B]);

        acc_sum[0] += acc_i[0];
        acc_sum[1] += acc_i[1];
        acc_sum[2] += acc_i[2];
        gyro_sum[0] += gyro_i[0];
        gyro_sum[1] += gyro_i[1];
        gyro_sum[2] += gyro_i[2];
        temp_sum += (rt_int8_t)pi[0x0D];
        header_sum += pi[0];
        parsed_count++;

        for (j = (rt_uint16_t)(i + 1U); j < packet_count; ++j)
        {
            const rt_uint8_t *pj = &raw_data->fifo_data[(rt_size_t)j * packet_size];
            rt_int16_t acc_j[3];
            rt_int16_t gyro_j[3];
            rt_uint16_t ts_i;
            rt_uint16_t ts_j;

            acc_j[0] = imu_poll_read_s16_be(&pj[0x01]);
            acc_j[1] = imu_poll_read_s16_be(&pj[0x03]);
            acc_j[2] = imu_poll_read_s16_be(&pj[0x05]);
            gyro_j[0] = imu_poll_read_s16_be(&pj[0x07]);
            gyro_j[1] = imu_poll_read_s16_be(&pj[0x09]);
            gyro_j[2] = imu_poll_read_s16_be(&pj[0x0B]);
            ts_i = (packet_size >= 16U) ? (rt_uint16_t)(((rt_uint16_t)pi[0x0E] << 8) | pi[0x0F]) : 0U;
            ts_j = (packet_size >= 16U) ? (rt_uint16_t)(((rt_uint16_t)pj[0x0E] << 8) | pj[0x0F]) : 0U;

            if (acc_i[0] == acc_j[0]) { same_flag |= (rt_uint8_t)(1U << 5); }
            if (acc_i[1] == acc_j[1]) { same_flag |= (rt_uint8_t)(1U << 4); }
            if (acc_i[2] == acc_j[2]) { same_flag |= (rt_uint8_t)(1U << 3); }
            if (gyro_i[0] == gyro_j[0]) { same_flag |= (rt_uint8_t)(1U << 2); }
            if (gyro_i[1] == gyro_j[1]) { same_flag |= (rt_uint8_t)(1U << 1); }
            if (gyro_i[2] == gyro_j[2]) { same_flag |= (rt_uint8_t)(1U << 0); }
            if (pi[0] == pj[0]) { same_flag |= (rt_uint8_t)(1U << 7); }
            if (ts_i == ts_j) { same_flag |= (rt_uint8_t)(1U << 6); }
        }
    }

    if (parsed_count > 0U)
    {
        const rt_uint8_t *plast = &raw_data->fifo_data[(rt_size_t)(parsed_count - 1U) * packet_size];

        avg_accel[0] = imu_poll_div_round_s32(acc_sum[0], parsed_count);
        avg_accel[1] = imu_poll_div_round_s32(acc_sum[1], parsed_count);
        avg_accel[2] = imu_poll_div_round_s32(acc_sum[2], parsed_count);
        avg_gyro[0] = imu_poll_div_round_s32(gyro_sum[0], parsed_count);
        avg_gyro[1] = imu_poll_div_round_s32(gyro_sum[1], parsed_count);
        avg_gyro[2] = imu_poll_div_round_s32(gyro_sum[2], parsed_count);
        avg_temp = imu_poll_div_round_s32(temp_sum, parsed_count);
        fifo_header = (rt_uint8_t)imu_poll_div_round_s32(header_sum, parsed_count);
        if (packet_size >= 16U)
        {
            fifo_timestamp = (rt_uint16_t)(((rt_uint16_t)plast[0x0E] << 8) | plast[0x0F]);
        }
    }
    packet_count = parsed_count;

    rt_memset(frame, 0, APP_IMU_FRAME_SIZE);
    imu_poll_write_u16_le(&frame[0], APP_IMU_FRAME_MAGIC_HEAD);
    RT_UNUSED(imu_index);
    RT_UNUSED(seq);
    frame[2] = same_flag;
    imu_poll_write_u64_le(&frame[3], raw_data->timestamp_us);
    imu_poll_write_u16_le(&frame[11], packet_count);
    frame[13] = fifo_header;
    imu_poll_write_u16_le(&frame[14], fifo_timestamp);
    imu_poll_write_u16_le(&frame[16], (rt_uint16_t)avg_accel[0]);
    imu_poll_write_u16_le(&frame[18], (rt_uint16_t)avg_accel[1]);
    imu_poll_write_u16_le(&frame[20], (rt_uint16_t)avg_accel[2]);
    imu_poll_write_u16_le(&frame[22], (rt_uint16_t)avg_gyro[0]);
    imu_poll_write_u16_le(&frame[24], (rt_uint16_t)avg_gyro[1]);
    imu_poll_write_u16_le(&frame[26], (rt_uint16_t)avg_gyro[2]);
    imu_poll_write_u16_le(&frame[28], (rt_uint16_t)avg_temp);
    imu_poll_write_u16_le(&frame[30], imu_poll_crc16_ccitt(&frame[2], APP_IMU_FRAME_SIZE - 4U));
}

static void imu_poll_update_gap_locked(rt_tick_t now_tick)
{
    imu_writer_runtime_t *writer = &g_imu_poll_ctx.writer;

    if (writer->last_poll_tick != 0U)
    {
        rt_tick_t delta_tick = now_tick - writer->last_poll_tick;
        rt_uint32_t gap_ms = (rt_uint32_t)((delta_tick * 1000U) / RT_TICK_PER_SECOND);

        if (gap_ms > writer->max_gap_ms)
        {
            writer->max_gap_ms = gap_ms;
        }
    }

    writer->last_poll_tick = now_tick;
}

static int imu_poll_writer_fetch_half(rt_uint8_t *half_index, rt_uint32_t *bytes, rt_bool_t *done)
{
    imu_writer_runtime_t *writer = &g_imu_poll_ctx.writer;

    *done = RT_FALSE;
    *bytes = 0U;
    *half_index = 0U;

    imu_poll_lock();

    if ((writer->ready_mask & 0x01U) != 0U)
    {
        *half_index = 0U;
    }
    else if ((writer->ready_mask & 0x02U) != 0U)
    {
        *half_index = 1U;
    }
    else if (writer->poll_finished == RT_TRUE)
    {
        *done = RT_TRUE;
    }
    else
    {
        imu_poll_unlock();
        return RT_EEMPTY;
    }

    if (*done == RT_FALSE)
    {
        *bytes = writer->half_bytes[*half_index];
        writer->half_bytes[*half_index] = 0U;
        writer->ready_mask = (rt_uint8_t)(writer->ready_mask & (rt_uint8_t)~(1U << *half_index));
    }

    imu_poll_unlock();
    return RT_EOK;
}

static void imu_poll_writer_thread_entry(void *parameter)
{
    imu_writer_runtime_t *writer = &g_imu_poll_ctx.writer;
    int write_result = RT_EOK;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_uint8_t half_index = 0U;
        rt_uint32_t bytes = 0U;
        rt_bool_t done = RT_FALSE;
        int fetch_result;

        if (rt_sem_take(writer->sem, rt_tick_from_millisecond(100U)) != RT_EOK)
        {
            continue;
        }

        while (1)
        {
            fetch_result = imu_poll_writer_fetch_half(&half_index, &bytes, &done);
            if (fetch_result == RT_EEMPTY)
            {
                break;
            }

            if (done == RT_TRUE)
            {
                imu_poll_writer_close(&writer->file_writer);
                imu_poll_lock();
                writer->running = RT_FALSE;
                writer->thread = RT_NULL;
                g_imu_poll_ctx.recording = RT_FALSE;
                imu_poll_unlock();
                session_complete_current();
                return;
            }

            if ((bytes > 0U) &&
                (fatfs_sdcard_write_raw_file(&writer->file_writer.file,
                                             imu_poll_half_base(half_index),
                                             bytes,
                                             RT_FALSE) < 0))
            {
                write_result = -RT_ERROR;
                imu_poll_writer_abort(&writer->file_writer);
                imu_poll_lock();
                writer->running = RT_FALSE;
                writer->thread = RT_NULL;
                g_imu_poll_ctx.recording = RT_FALSE;
                g_imu_poll_ctx.last_error = write_result;
                imu_poll_unlock();
                session_complete_current();
                return;
            }

            imu_poll_lock();
            writer->file_writer.flush_count++;
            imu_poll_unlock();
        }
    }
}

static int imu_poll_run_recording(void)
{
    imu_writer_runtime_t *writer = &g_imu_poll_ctx.writer;
    rt_tick_t start_tick;
    rt_uint32_t seq = 0;
    int result = RT_EOK;

    if (g_imu_poll_ctx.detected_count == 0U)
    {
        return -RT_ENOSYS;
    }

    if (imu_poll_writer_open(&writer->file_writer, RT_TRUE) != RT_EOK)
    {
        return -RT_ERROR;
    }

    imu_poll_lock();
    writer->active_half = 0U;
    writer->half_offset = 0U;
    writer->half_bytes[0] = 0U;
    writer->half_bytes[1] = 0U;
    writer->ready_mask = 0U;
    writer->poll_finished = RT_FALSE;
    writer->running = RT_TRUE;
    writer->frame_count = 0U;
    writer->file_writer.flush_count = 0U;
    writer->max_gap_ms = 0U;
    writer->last_poll_tick = 0U;
    imu_poll_unlock();

    start_tick = rt_tick_get();

    while (1)
    {
        int index;
        rt_tick_t now = rt_tick_get();

        imu_poll_lock();
        imu_poll_update_gap_locked(now);
        if (g_imu_poll_ctx.stop_requested == RT_TRUE)
        {
            imu_poll_unlock();
            break;
        }
        imu_poll_unlock();

        if ((rt_tick_t)(now - start_tick) >= rt_tick_from_millisecond(g_imu_poll_ctx.duration_ms))
        {
            break;
        }

        for (index = 0; index < APP_IMU_MAX_COUNT; ++index)
        {
            imu_slot_t *slot = &g_imu_poll_ctx.slots[index];

            if (slot->detected == RT_FALSE)
            {
                continue;
            }

            drvf::IMURawData raw_data{};
            rt_uint8_t frame[APP_IMU_FRAME_SIZE];
            rt_ssize_t read_count = rt_device_read(slot->device, IMU_POS_ACC_GYRO, &raw_data, sizeof(raw_data));

            if (read_count <= 0)
            {
                continue;
            }

            seq++;
            imu_poll_build_frame(frame, slot->index, seq, &raw_data);
            if (imu_poll_writer_push_frame(frame) != RT_EOK)
            {
                result = -RT_EFULL;
                goto __exit;
            }
        }

        rt_thread_mdelay(APP_IMU_POLL_PERIOD_MS);
    }

__exit:
    imu_poll_lock();
    if ((result == RT_EOK) || (writer->half_offset > 0U))
    {
        if (imu_poll_queue_active_half_locked() != RT_EOK)
        {
            result = -RT_EFULL;
        }
    }
    writer->poll_finished = RT_TRUE;
    imu_poll_unlock();
    rt_sem_release(writer->sem);

    return result;
}

static void imu_poll_thread_entry(void *parameter)
{
    int result;

    RT_UNUSED(parameter);

    result = imu_poll_run_recording();

    imu_poll_lock();
    g_imu_poll_ctx.last_run_ok = (result == RT_EOK) ? RT_TRUE : RT_FALSE;
    g_imu_poll_ctx.last_error = result;
    g_imu_poll_ctx.stop_requested = RT_FALSE;
    g_imu_poll_ctx.poll_thread = RT_NULL;
    imu_poll_unlock();
}

extern "C" int imu_reader_thread_probe_count(void)
{
    if (imu_poll_ensure_init() != RT_EOK)
    {
        return 0;
    }

    imu_poll_lock();
    imu_poll_refresh_slots();
    imu_poll_unlock();
    return (int)g_imu_poll_ctx.detected_count;
}

extern "C" int imu_reader_thread_start(void)
{
    return imu_reader_thread_start_for_test("arw");
}

extern "C" int imu_reader_thread_start_for_test(const char *test_name)
{
    rt_bool_t mounted_here = RT_FALSE;

    if (imu_poll_ensure_init() != RT_EOK)
    {
        return -RT_ENOMEM;
    }

    imu_poll_lock();
    imu_poll_refresh_slots();
    if (g_imu_poll_ctx.recording == RT_TRUE)
    {
        imu_poll_unlock();
        return -RT_EBUSY;
    }

    if (g_imu_poll_ctx.detected_count == 0U)
    {
        imu_poll_unlock();
        return -RT_ENOSYS;
    }

    g_imu_poll_ctx.stop_requested = RT_FALSE;
    g_imu_poll_ctx.recording = RT_TRUE;
    g_imu_poll_ctx.last_run_ok = RT_FALSE;
    g_imu_poll_ctx.last_error = RT_EOK;
    g_imu_poll_ctx.recorded_frames = 0U;
    g_imu_poll_ctx.output_index = 0U;
    while (rt_sem_trytake(g_imu_poll_ctx.writer.sem) == RT_EOK)
    {
    }
    rt_memset(g_imu_poll_ctx.output_dir, 0, sizeof(g_imu_poll_ctx.output_dir));
    rt_memset(g_imu_poll_ctx.output_path, 0, sizeof(g_imu_poll_ctx.output_path));
    rt_strncpy(g_imu_poll_ctx.test_name,
               (test_name != RT_NULL) ? test_name : "arw",
               sizeof(g_imu_poll_ctx.test_name) - 1U);
    g_imu_poll_ctx.test_name[sizeof(g_imu_poll_ctx.test_name) - 1U] = '\0';

    if (fatfs_sdcard_is_mounted() != RT_TRUE)
    {
        if (fatfs_sdcard_mount() != RT_EOK)
        {
            g_imu_poll_ctx.recording = RT_FALSE;
            imu_poll_unlock();
            return -RT_ERROR;
        }
        mounted_here = RT_TRUE;
    }

    if (file_naming_make_next_bin_path(g_imu_poll_ctx.test_name,
                                       g_imu_poll_ctx.output_dir,
                                       sizeof(g_imu_poll_ctx.output_dir),
                                       g_imu_poll_ctx.output_path,
                                       sizeof(g_imu_poll_ctx.output_path),
                                       &g_imu_poll_ctx.output_index) != RT_EOK)
    {
        if (mounted_here == RT_TRUE)
        {
            fatfs_sdcard_unmount();
        }
        g_imu_poll_ctx.recording = RT_FALSE;
        imu_poll_unlock();
        return -RT_ERROR;
    }

    if (mounted_here == RT_TRUE)
    {
        fatfs_sdcard_unmount();
    }

    g_imu_poll_ctx.writer.thread = rt_thread_create("imuwrite",
                                                    imu_poll_writer_thread_entry,
                                                    RT_NULL,
                                                    4096,
                                                    15,
                                                    10);
    if (g_imu_poll_ctx.writer.thread == RT_NULL)
    {
        g_imu_poll_ctx.recording = RT_FALSE;
        imu_poll_unlock();
        return -RT_ENOMEM;
    }

    g_imu_poll_ctx.poll_thread = rt_thread_create("imupoll",
                                                  imu_poll_thread_entry,
                                                  RT_NULL,
                                                  8192,
                                                  14,
                                                  10);
    if (g_imu_poll_ctx.poll_thread == RT_NULL)
    {
        g_imu_poll_ctx.writer.thread = RT_NULL;
        g_imu_poll_ctx.recording = RT_FALSE;
        imu_poll_unlock();
        return -RT_ENOMEM;
    }

    rt_thread_startup(g_imu_poll_ctx.writer.thread);
    rt_thread_startup(g_imu_poll_ctx.poll_thread);
    imu_poll_unlock();
    return RT_EOK;
}

extern "C" int imu_reader_thread_stop(void)
{
    if (imu_poll_ensure_init() != RT_EOK)
    {
        return -RT_ENOMEM;
    }

    imu_poll_lock();
    g_imu_poll_ctx.stop_requested = RT_TRUE;
    imu_poll_unlock();
    return RT_EOK;
}

extern "C" int imu_reader_thread_is_recording(void)
{
    int recording;

    if (imu_poll_ensure_init() != RT_EOK)
    {
        return 0;
    }

    imu_poll_lock();
    recording = (g_imu_poll_ctx.recording == RT_TRUE) ? 1 : 0;
    imu_poll_unlock();
    return recording;
}

extern "C" rt_uint32_t imu_reader_thread_recorded_frames(void)
{
    rt_uint32_t frames;

    if (imu_poll_ensure_init() != RT_EOK)
    {
        return 0U;
    }

    imu_poll_lock();
    frames = g_imu_poll_ctx.recorded_frames;
    imu_poll_unlock();
    return frames;
}

extern "C" rt_uint32_t imu_reader_thread_duration_ms(void)
{
    return g_imu_poll_ctx.duration_ms;
}

extern "C" const char *imu_reader_thread_output_path(void)
{
    return (g_imu_poll_ctx.output_path[0] != '\0') ? g_imu_poll_ctx.output_path : "0:/03_arw/000.BIN";
}

extern "C" const char *imu_reader_thread_output_dir(void)
{
    return (g_imu_poll_ctx.output_dir[0] != '\0') ? g_imu_poll_ctx.output_dir : "0:/03_arw";
}

extern "C" rt_uint32_t imu_reader_thread_output_index(void)
{
    return g_imu_poll_ctx.output_index;
}

extern "C" rt_uint32_t imu_reader_thread_flush_count(void)
{
    rt_uint32_t flushes;

    if (imu_poll_ensure_init() != RT_EOK)
    {
        return 0U;
    }

    imu_poll_lock();
    flushes = g_imu_poll_ctx.writer.file_writer.flush_count;
    imu_poll_unlock();
    return flushes;
}

extern "C" rt_uint32_t imu_reader_thread_max_gap_ms(void)
{
    rt_uint32_t max_gap_ms;

    if (imu_poll_ensure_init() != RT_EOK)
    {
        return 0U;
    }

    imu_poll_lock();
    max_gap_ms = g_imu_poll_ctx.writer.max_gap_ms;
    imu_poll_unlock();
    return max_gap_ms;
}

static int imu_poll_probe(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("RESULT cmd=imu_poll_probe status=ok detected=%d\r\n", imu_reader_thread_probe_count());
    return RT_EOK;
}
MSH_CMD_EXPORT(imu_poll_probe, probe imu devices for poll thread);

static int imu_poll_start_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("RESULT cmd=imu_poll_start status=%d detected=%d\r\n",
               imu_reader_thread_start(),
               imu_reader_thread_probe_count());
    return RT_EOK;
}
MSH_CMD_EXPORT(imu_poll_start_cmd, start real imu poll thread);

static int imu_poll_stop_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("RESULT cmd=imu_poll_stop status=%d frames=%lu\r\n",
               imu_reader_thread_stop(),
               (unsigned long)imu_reader_thread_recorded_frames());
    return RT_EOK;
}
MSH_CMD_EXPORT(imu_poll_stop_cmd, stop real imu poll thread);
