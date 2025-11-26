#include "acc_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "acc_cmd"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include <finsh.h>
#include "param.h"
}

#ifdef PROJECT_BF_ACC_EN

// 加速度计校准命令
static void acc_calibrate(int argc, char** argv) {
  AccBf& acc = AccBf::instance();

  rt_err_t ret = acc.startCalibration();
  if (ret == RT_EOK) {
    LOG_I("Accelerometer calibration started");
    rt_kprintf("Accelerometer calibration started. Please keep the aircraft still.\n");

    // 等待校准完成
    uint32_t timeout_ms = PROJECT_BF_ACC_CALIBRATION_DURATION_MS + 500;  // 额外500ms缓冲
    rt_thread_mdelay(timeout_ms);

    // 等待校准完成（通过检查校准状态）
    uint32_t check_count = 0;
    while (!acc.isCalibrationComplete() && check_count < (timeout_ms / 10)) {
      rt_thread_mdelay(10);
      check_count++;
    }

    // 获取校准结果
    float acc_zero[3] = {0.0f, 0.0f, 0.0f};
    acc.getAccZero(acc_zero);

    // 保存到参数系统
    if (setParam("acc_zero_x", &acc_zero[0], sizeof(acc_zero[0])) == RT_EOK &&
        setParam("acc_zero_y", &acc_zero[1], sizeof(acc_zero[1])) == RT_EOK &&
        setParam("acc_zero_z", &acc_zero[2], sizeof(acc_zero[2])) == RT_EOK) {
      LOG_I("Accelerometer calibration completed and saved: %.3f, %.3f, %.3f", 
            acc_zero[0], acc_zero[1], acc_zero[2]);
      rt_kprintf("Accelerometer calibration completed and saved.\n");
      rt_kprintf("Zero offset: X=%.3f, Y=%.3f, Z=%.3f\n", acc_zero[0], acc_zero[1], acc_zero[2]);
    } else {
      LOG_E("Failed to save accelerometer calibration");
      rt_kprintf("Calibration completed but failed to save to parameters.\n");
    }
  } else {
    LOG_E("Failed to start accelerometer calibration: %d", ret);
    rt_kprintf("Failed to start accelerometer calibration.\n");
  }
}
MSH_CMD_EXPORT(acc_calibrate, Accelerometer calibration command);

// 获取加速度计校准值命令
static void acc_get_zero(int argc, char** argv) {
  AccBf& acc = AccBf::instance();

  float acc_zero[3] = {0.0f, 0.0f, 0.0f};
  acc.getAccZero(acc_zero);

  rt_kprintf("Accelerometer zero offset:\n");
  rt_kprintf("  X: %.3f\n", acc_zero[0]);
  rt_kprintf("  Y: %.3f\n", acc_zero[1]);
  rt_kprintf("  Z: %.3f\n", acc_zero[2]);
}
MSH_CMD_EXPORT(acc_get_zero, Get accelerometer zero offset);

// 设置加速度计 Trim 值命令
static void acc_set_trim(int argc, char** argv) {
  if (argc < 3) {
    rt_kprintf("Usage: acc_set_trim <roll> <pitch>\n");
    rt_kprintf("  roll:  Roll trim value in degrees\n");
    rt_kprintf("  pitch: Pitch trim value in degrees\n");
    return;
  }

  float roll_trim = atof(argv[1]);
  float pitch_trim = atof(argv[2]);

  if (setParam("acc_trim_roll", &roll_trim, sizeof(roll_trim)) == RT_EOK &&
      setParam("acc_trim_pitch", &pitch_trim, sizeof(pitch_trim)) == RT_EOK) {
    LOG_I("Accelerometer trim set: roll=%.2f, pitch=%.2f", roll_trim, pitch_trim);
    rt_kprintf("Accelerometer trim set: roll=%.2f, pitch=%.2f\n", roll_trim, pitch_trim);
  } else {
    LOG_E("Failed to set accelerometer trim");
    rt_kprintf("Failed to set accelerometer trim.\n");
  }
}
MSH_CMD_EXPORT(acc_set_trim, Set accelerometer trim values);

#endif /* PROJECT_BF_ACC_EN */

