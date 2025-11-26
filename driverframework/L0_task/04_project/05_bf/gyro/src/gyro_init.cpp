#include "gyro_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "gyro_init"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "param.h"
#include "bfImuFilterParam.h"
#include "timestamp.h"
#include "../common/inc/init_sync.h"  // For initSyncWait, initSyncNotify
#include "../imu/inc/bmi270_class.h"
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

extern "C" {
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
}

// IMU 采样频率（Hz）- 假设从 IMU 发布频率推算
#ifndef PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ
#define PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ 800.0f
#endif

// Filter slot definitions (for gyroInitLowpassFilterLpf)
#define FILTER_LPF1 0
#define FILTER_LPF2 1

// IMU downsample filter cutoff frequency (Hz)
#define GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ 200.0f

bool gyro::gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime) {
  // 参考 gyro_init.c:127-197
  filterApplyFnPtr* lowpassFilterApplyFn = nullptr;
  gyroLowpassFilter_t* lowpassFilter = nullptr;
  uint8_t* filter_type = nullptr;
  bool* filter_enabled = nullptr;

  switch (slot) {
    case FILTER_LPF1:
      lowpassFilterApplyFn = &lpf1_apply_fn_;
      lowpassFilter = lpf1_filter_;
      filter_type = &lpf1_type_;
      filter_enabled = &lpf1_enabled_;
      break;

    case FILTER_LPF2:
      lowpassFilterApplyFn = &lpf2_apply_fn_;
      lowpassFilter = lpf2_filter_;
      filter_type = &lpf2_type_;
      filter_enabled = &lpf2_enabled_;
      break;

    default:
      return false;
  }

  bool ret = false;

  // Establish some common constants
  const uint32_t gyroFrequencyNyquist = 1000000 / 2 / looptime;
  const float gyroDt = looptime * 1e-6f;

  // Dereference the pointer to null before checking valid cutoff and filter type
  *lowpassFilterApplyFn = nullFilterApply;
  *filter_enabled = false;

  // If lowpass cutoff has been specified
  if (lpfHz) {
    *filter_type = type;
    switch (type) {
      case FILTER_PT1:
        *lowpassFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
        for (int axis = 0; axis < 3; axis++) {
          pt1FilterInit(&lowpassFilter[axis].pt1Filter, pt1FilterGain(lpfHz, gyroDt));
        }
        ret = true;
        *filter_enabled = true;
        break;
      case FILTER_BIQUAD:
        if (lpfHz <= gyroFrequencyNyquist) {
#ifdef USE_DYN_LPF
          *lowpassFilterApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1;
#else
          *lowpassFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
#endif
          for (int axis = 0; axis < 3; axis++) {
            biquadFilterInitLPF(&lowpassFilter[axis].biquadFilter, lpfHz, looptime);
          }
          ret = true;
          *filter_enabled = true;
        }
        break;
      case FILTER_PT2:
        *lowpassFilterApplyFn = (filterApplyFnPtr)pt2FilterApply;
        for (int axis = 0; axis < 3; axis++) {
          pt2FilterInit(&lowpassFilter[axis].pt2Filter, pt2FilterGain(lpfHz, gyroDt));
        }
        ret = true;
        *filter_enabled = true;
        break;
      case FILTER_PT3:
        *lowpassFilterApplyFn = (filterApplyFnPtr)pt3FilterApply;
        for (int axis = 0; axis < 3; axis++) {
          pt3FilterInit(&lowpassFilter[axis].pt3Filter, pt3FilterGain(lpfHz, gyroDt));
        }
        ret = true;
        *filter_enabled = true;
        break;
    }
  }
  return ret;
}

void gyro::gyroInitFilters() {
  // 参考 gyro_init.c:229-266
  uint16_t gyro_lpf1_init_hz = 0;
  uint8_t gyro_lpf1_type = 0;
  uint8_t gyro_lpf2_type = 0;
  uint16_t gyro_lpf2_static_hz = 0;
  uint16_t gyro_soft_notch_hz_1 = 0;
  uint16_t gyro_soft_notch_cutoff_1 = 0;
  uint16_t gyro_soft_notch_hz_2 = 0;
  uint16_t gyro_soft_notch_cutoff_2 = 0;

  // 读取 LPF1 参数
  if (getParam("filter_gyro_lpf1_type", &gyro_lpf1_type, sizeof(gyro_lpf1_type)) == RT_EOK &&
      getParam("filter_gyro_lpf1_static_hz", &gyro_lpf1_init_hz, sizeof(gyro_lpf1_init_hz)) == RT_EOK) {
    // 读取动态滤波器参数
#ifdef USE_DYN_LPF
    uint16_t gyro_lpf1_dyn_min_hz = 0;
    if (getParam("filter_gyro_lpf1_dyn_min_hz", &gyro_lpf1_dyn_min_hz, sizeof(gyro_lpf1_dyn_min_hz)) == RT_EOK) {
      if (gyro_lpf1_dyn_min_hz > 0) {
        gyro_lpf1_init_hz = gyro_lpf1_dyn_min_hz;
      }
    }
#endif
  }

  // 初始化 LPF1 滤波器
  gyroInitLowpassFilterLpf(FILTER_LPF1, gyro_lpf1_type, gyro_lpf1_init_hz, target_looptime_us_);

  // 读取并初始化 LPF2 滤波器
  if (getParam("filter_gyro_lpf2_type", &gyro_lpf2_type, sizeof(gyro_lpf2_type)) == RT_EOK &&
      getParam("filter_gyro_lpf2_static_hz", &gyro_lpf2_static_hz, sizeof(gyro_lpf2_static_hz)) == RT_EOK) {
    downsample_filter_enabled_ =
        gyroInitLowpassFilterLpf(FILTER_LPF2, gyro_lpf2_type, gyro_lpf2_static_hz, sample_looptime_us_);
  }

  // 初始化 Notch1 滤波器
  if (getParam("filter_gyro_soft_notch_hz_1", &gyro_soft_notch_hz_1, sizeof(gyro_soft_notch_hz_1)) == RT_EOK &&
      getParam("filter_gyro_soft_notch_cutoff_1", &gyro_soft_notch_cutoff_1, sizeof(gyro_soft_notch_cutoff_1)) ==
          RT_EOK) {
    // TODO: 实现 calculateNyquistAdjustedNotchHz
    // notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);
    if (gyro_soft_notch_hz_1 > 0 && gyro_soft_notch_cutoff_1 > 0) {
      float freq_hz = static_cast<float>(gyro_soft_notch_hz_1);
      float cutoff_hz = static_cast<float>(gyro_soft_notch_cutoff_1);
      uint32_t refresh_rate_us = target_looptime_us_;
      const float notchQ = filterGetNotchQ(freq_hz, cutoff_hz);

      notch1_apply_fn_ = (filterApplyFnPtr)biquadFilterApply;
      for (int axis = 0; axis < 3; axis++) {
        biquadFilterInit(&notch1_filter_[axis], freq_hz, refresh_rate_us, notchQ, FILTER_NOTCH, 1.0f);
      }
      notch1_enabled_ = true;
    }
  }

  // 初始化 Notch2 滤波器
  if (getParam("filter_gyro_soft_notch_hz_2", &gyro_soft_notch_hz_2, sizeof(gyro_soft_notch_hz_2)) == RT_EOK &&
      getParam("filter_gyro_soft_notch_cutoff_2", &gyro_soft_notch_cutoff_2, sizeof(gyro_soft_notch_cutoff_2)) ==
          RT_EOK) {
    // TODO: 实现 calculateNyquistAdjustedNotchHz
    // notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);
    if (gyro_soft_notch_hz_2 > 0 && gyro_soft_notch_cutoff_2 > 0) {
      float freq_hz = static_cast<float>(gyro_soft_notch_hz_2);
      float cutoff_hz = static_cast<float>(gyro_soft_notch_cutoff_2);
      uint32_t refresh_rate_us = target_looptime_us_;
      const float notchQ = filterGetNotchQ(freq_hz, cutoff_hz);

      notch2_apply_fn_ = (filterApplyFnPtr)biquadFilterApply;
      for (int axis = 0; axis < 3; axis++) {
        biquadFilterInit(&notch2_filter_[axis], freq_hz, refresh_rate_us, notchQ, FILTER_NOTCH, 1.0f);
      }
      notch2_enabled_ = true;
    }
  }

#ifdef USE_DYN_LPF
  // 初始化动态 LPF 参数
  uint16_t gyro_lpf1_dyn_min_hz = 0;
  if (getParam("filter_gyro_lpf1_dyn_min_hz", &gyro_lpf1_dyn_min_hz, sizeof(gyro_lpf1_dyn_min_hz)) == RT_EOK) {
    if (gyro_lpf1_dyn_min_hz > 0) {
      dyn_lpf_min_ = gyro_lpf1_dyn_min_hz;

      uint16_t gyro_lpf1_dyn_max_hz = 0;
      if (getParam("filter_gyro_lpf1_dyn_max_hz", &gyro_lpf1_dyn_max_hz, sizeof(gyro_lpf1_dyn_max_hz)) == RT_EOK) {
        dyn_lpf_max_ = gyro_lpf1_dyn_max_hz;
      } else {
        dyn_lpf_max_ = 250;  // 默认值
      }

      uint8_t gyro_lpf1_dyn_expo = 0;
      if (getParam("filter_gyro_lpf1_dyn_expo", &gyro_lpf1_dyn_expo, sizeof(gyro_lpf1_dyn_expo)) == RT_EOK) {
        dyn_lpf_curve_expo_ = gyro_lpf1_dyn_expo;
      } else {
        dyn_lpf_curve_expo_ = 5;  // 默认值
      }

      // 根据 LPF1 类型设置动态滤波器类型
      switch (gyro_lpf1_type) {
          case FILTER_PT1:
            dyn_lpf_filter_ = 1;  // DYN_LPF_PT1
            break;
          case FILTER_BIQUAD:
            dyn_lpf_filter_ = 2;  // DYN_LPF_BIQUAD
            break;
          case FILTER_PT2:
            dyn_lpf_filter_ = 3;  // DYN_LPF_PT2
            break;
          case FILTER_PT3:
            dyn_lpf_filter_ = 4;  // DYN_LPF_PT3
            break;
          default:
            dyn_lpf_filter_ = 0;  // DYN_LPF_NONE
            break;
      }
    } else {
      dyn_lpf_filter_ = 0;  // DYN_LPF_NONE
    }
  }
#endif

#ifdef USE_DYN_NOTCH_FILTER
  // TODO: 实现动态 notch 滤波器初始化
  // dynNotchInit(dynNotchConfig(), target_looptime_us_);
#endif

  // 步骤8: 初始化第三个低通滤波器 - PT1（用于姿态估计）
  // 位置：gyroFiltering() - src/main/sensors/gyro.c:541
  // 使用：gyro.imuGyroFilter
  // 截止频率：从参数系统读取 filter_gyro_imu_downsample_cutoff_hz（默认 200 Hz）
  // 输出：gyroFilteredDownsampled[axis]（给 attitude 使用）
  // gyroFilteredDownsampled[axis] = pt1FilterApply(&gyro.imuGyroFilter[axis], gyro.gyroADCf[axis]);
  uint16_t gyro_imu_downsample_cutoff_hz = 200;  // 默认 200Hz
  if (getParam("filter_gyro_imu_downsample_cutoff_hz", &gyro_imu_downsample_cutoff_hz,
               sizeof(gyro_imu_downsample_cutoff_hz)) != RT_EOK) {
    gyro_imu_downsample_cutoff_hz = 200;  // 如果参数不存在，使用默认值 200Hz
  }
  const float k = pt1FilterGain(static_cast<float>(gyro_imu_downsample_cutoff_hz), target_looptime_us_ * 1e-6f);
  for (int axis = 0; axis < 3; axis++) {
    pt1FilterInit(&imu_gyro_filter_[axis], k);
  }
  imu_gyro_filter_enabled_ = true;
  LOG_I("IMU downsample filter initialized: cutoff=%.1f Hz", static_cast<float>(gyro_imu_downsample_cutoff_hz));
}

void gyro::initFilters() {
  // 调用新的 gyroInitFilters 函数（保持向后兼容）
  gyroInitFilters();
}

void gyro::initDownsampleFilter() {
  // 这个函数现在只用于初始化 LPF2 降采样相关的逻辑
  // 第三个低通滤波器（imuGyroFilter）的初始化已经在 gyroInitFilters() 中完成
  // 保留此函数以保持接口一致性，但实际初始化逻辑已移到 gyroInitFilters()
}

void gyro::setTargetLooptime(uint8_t pid_denom) {
  // 参考 gyro.c:750-759 中的 gyroSetTargetLooptime
  // activePidLoopDenom = pidDenom;
  // if (gyro.sampleRateHz) {
  //     gyro.sampleLooptime = 1e6f / gyro.sampleRateHz;
  //     gyro.targetLooptime = activePidLoopDenom * 1e6f / gyro.sampleRateHz;
  // } else {
  //     gyro.sampleLooptime = 0;
  //     gyro.targetLooptime = 0;
  // }

  if (sample_rate_hz_ > 0) {
    sample_looptime_us_ = static_cast<uint32_t>(1e6f / static_cast<float>(sample_rate_hz_));
    target_looptime_us_ = pid_denom * sample_looptime_us_;
    LOG_I("Target looptime set: pid_denom=%u, sample_rate=%u Hz, sample_looptime=%u us, target_looptime=%u us",
          pid_denom, sample_rate_hz_, sample_looptime_us_, target_looptime_us_);
  } else {
    sample_looptime_us_ = 0;
    target_looptime_us_ = 0;
    LOG_W("Cannot set target looptime: sample_rate_hz_ is 0");
  }
}

// 在线程入口函数中执行的初始化（依赖其他线程状态）
void gyro::initInThreadEntry() {
  // 在线程调度器启动后，等待 BMI270 初始化完成（使用初始化同步机制）
  // 这部分初始化依赖其他线程状态，必须在线程入口函数中执行
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG0_HIGH();  // Debug pin: Start waiting for BMI270 initialization
#endif

  rt_err_t ret = initSyncWait(INIT_SYNC_BMI270, 1000);  // 等待最多1秒

#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG0_LOW();  // Debug pin: End waiting for BMI270 initialization
#endif

  // 从 BMI270 单例获取 IMU 采样频率和周期（参考 gyro.c 的初始化）
  bf_bmi270::BMI270& bmi270 = bf_bmi270::BMI270::instance();
  float sample_rate_hz = 0.0f;
  float sample_dt = 0.0f;

  if (ret == RT_EOK && bmi270.initialized()) {
    sample_rate_hz = bmi270.getGyroSampleRateHz();
    sample_dt = bmi270.getGyroSampleDt();
    LOG_I("IMU sample rate: %.1f Hz, dt: %.6f s", sample_rate_hz, sample_dt);
  } else {
    // 如果等待超时或BMI270未初始化，使用默认值
    sample_rate_hz = PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ;
    sample_dt = 1.0f / sample_rate_hz;
    LOG_W("BMI270 not ready, using default sample rate: %.1f Hz", sample_rate_hz);
  }

  // 初始化 gyro_t 映射的变量（参考 gyro.c 中的初始化逻辑）
  // 参考 gyro.c:734-740 中 gyro.sampleRateHz 的设置
  sample_rate_hz_ = static_cast<uint16_t>(sample_rate_hz);

  // 注意：scale_ 已移除，因为 mcn 发布的数据已经是缩放后的（在 accgyro_spi_bmi270.cpp 中已应用 GYRO_SCALE_2000DPS）

  // 读取 pid_process_denom 参数并设置目标循环时间（参考 gyro.c:750-759 中的 gyroSetTargetLooptime）
  uint8_t pid_process_denom = 1;  // 默认值
  if (getParam("pid_process_denom", &pid_process_denom, sizeof(pid_process_denom)) != RT_EOK) {
    // 如果参数不存在，使用默认值
    pid_process_denom = 1;
  }

  // 设置目标循环时间（参考 gyroSetTargetLooptime）
  setTargetLooptime(pid_process_denom);

  // 初始化加速度采样频率（如果需要）
  acc_sample_rate_hz_ = sample_rate_hz_;  // 默认与陀螺仪相同

  // 初始化滤波器（从参数系统读取配置，参考 gyroInitFilters）
  // 注意：这些初始化依赖 sample_rate_hz_ 和 target_looptime_us_，所以必须在这里执行
  gyroInitFilters();

  // 初始化降采样滤波器（参考 gyro_init.c:262-265）
  // 注意：这些初始化依赖 target_looptime_us_，所以必须在这里执行
  initDownsampleFilter();

  // 通知 Gyro Filter 初始化完成
  initSyncNotify(INIT_SYNC_GYRO_FILTER);
}
