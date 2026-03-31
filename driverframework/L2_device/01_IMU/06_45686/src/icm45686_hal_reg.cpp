#include "ICM45686.hpp"

extern "C" {
#include "imu.h"
#include "finsh.h"
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

drvf::ICM45686 g_icm45686(3, 0);

static int8_t icm45686_read_data(imu_dev_t imu, rt_off_t pos, void *data, rt_size_t size) {
  RT_UNUSED(imu);
  RT_UNUSED(pos);

  if (data == RT_NULL || size < sizeof(drvf::IMURawData)) {
    return -RT_EINVAL;
  }

  drvf::IMURawData raw_data{};
  if (!g_icm45686.ReadRaw(raw_data)) {
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

static const struct imu_ops icm45686_dev_ops = {
    .imu_config = RT_NULL,
    .imu_control = icm45686_control,
    .imu_read = icm45686_read_data,
};

static struct imu_device icm45686_dev = {
    .ops = &icm45686_dev_ops,
    .config =
        {
            1000,
            IMU_GYRO_MODE_NORMAL,
            1000,
            IMU_ACC_MODE_NORMAL,
            GYRO_SCALE_2000DPS,
            ACC_SCALE_16G,
            128,
            25.0f,
        },
};

static rt_err_t icm45686_hal_init(const char *imu_name) {
  const int ret = g_icm45686.DebugInit();
  if (ret != 0) {
    return -RT_ERROR;
  }

  return hal_imu_register(&icm45686_dev, imu_name, RT_DEVICE_FLAG_RDWR, RT_NULL);
}

}  // namespace

extern "C" {

static int icm45686_init_auto(void) { return icm45686_hal_init(SENSOR_NAME_ICM45686); }

int icm45686_force_probe_init(void) { return g_icm45686.DebugInit(); }

static int icm45686_debug_init_cmd(int argc, char **argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  rt_kprintf("RESULT cmd=icm45686_debug_init status=%d\r\n", g_icm45686.DebugInit());
  return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(icm45686_debug_init_cmd, icm45686_debug_init, rerun icm45686 init and dump key registers);

static int icm45686_dump_regs_cmd(int argc, char **argv) {
  RT_UNUSED(argc);
  RT_UNUSED(argv);
  g_icm45686.DumpKeyRegisters("shell_dump");
  rt_kprintf("RESULT cmd=icm45686_dump_regs status=ok\r\n");
  return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(icm45686_dump_regs_cmd, icm45686_dump_regs, dump icm45686 key registers);

static int icm45686_read_once_cmd(int argc, char **argv) {
  drvf::IMURawData raw_data{};
  uint8_t raw_ui[14] = {0};
  rt_uint16_t packet_tail = 0U;
  rt_uint16_t next_packet_tail = 0U;
  rt_uint16_t packet_delta = 0U;
  rt_uint16_t dump_len = 0U;
  RT_UNUSED(argc);
  RT_UNUSED(argv);

  if (!g_icm45686.ReadRaw(raw_data)) {
    g_icm45686.DumpKeyRegisters("read_once_fail");
    rt_kprintf("RESULT cmd=icm45686_read_once status=fail\r\n");
    return -RT_ERROR;
  }

  if (raw_data.packet_size >= 16U && raw_data.fifo_count >= raw_data.packet_size) {
    const rt_uint16_t tail_index = (rt_uint16_t)(raw_data.packet_size - 2U);
    packet_tail = (rt_uint16_t)(raw_data.fifo_data[tail_index] | ((rt_uint16_t)raw_data.fifo_data[tail_index + 1U] << 8));
  }
  if (raw_data.packet_size >= 16U && raw_data.fifo_count >= (rt_uint16_t)(raw_data.packet_size * 2U)) {
    const rt_uint16_t next_tail_index = (rt_uint16_t)((raw_data.packet_size * 2U) - 2U);
    next_packet_tail =
        (rt_uint16_t)(raw_data.fifo_data[next_tail_index] | ((rt_uint16_t)raw_data.fifo_data[next_tail_index + 1U] << 8));
    packet_delta = (rt_uint16_t)(next_packet_tail - packet_tail);
  }
  dump_len = raw_data.fifo_count >= raw_data.packet_size ? raw_data.packet_size : raw_data.fifo_count;

  rt_kprintf(
      "RESULT cmd=icm45686_read_once status=ok fifo_count=%u packet_size=%u head=%02X %02X %02X %02X tail=%02X %02X "
      "tail_u16_le=%u next_tail_u16_le=%u packet_delta_u16=%u\r\n",
             raw_data.fifo_count,
             raw_data.packet_size,
             raw_data.fifo_data[0],
             raw_data.fifo_data[1],
             raw_data.fifo_data[2],
             raw_data.fifo_data[3],
             raw_data.fifo_data[(raw_data.packet_size >= 2U) ? (raw_data.packet_size - 2U) : 0U],
             raw_data.fifo_data[(raw_data.packet_size >= 1U) ? (raw_data.packet_size - 1U) : 0U],
             packet_tail,
             next_packet_tail,
             packet_delta);
  if (dump_len > 0U) {
    rt_kprintf("FIFO16");
    for (rt_uint16_t index = 0; index < dump_len; ++index) {
      rt_kprintf(" %02X", raw_data.fifo_data[index]);
    }
    rt_kprintf("\r\n");
  }
  if (g_icm45686.ReadUiSnapshot(raw_ui, sizeof(raw_ui))) {
    rt_kprintf("UI14");
    for (rt_size_t index = 0; index < sizeof(raw_ui); ++index) {
      rt_kprintf(" %02X", raw_ui[index]);
    }
    rt_kprintf("\r\n");
  }
  return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(icm45686_read_once_cmd, icm45686_read_once, read one icm45686 fifo snapshot);

#ifdef BSP_USING_ICM45686
INIT_ENV_EXPORT(icm45686_init_auto);
#endif

}
