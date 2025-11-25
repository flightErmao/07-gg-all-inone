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

#include "bfGyroLpfFilter.hpp"

// IMU 采样频率（Hz）- 假设从 IMU 发布频率推算
#ifndef PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ
#define PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ 800.0f
#endif

void RateCtrlAngularVelocity::initFilters() {
  // 使用从成员变量获取的采样频率和周期
  float sample_rate_hz = static_cast<float>(sample_rate_hz_);

  // 如果无效，使用默认值
  if (sample_rate_hz <= 0.0f) {
    sample_rate_hz = PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ;
    sample_rate_hz_ = static_cast<uint16_t>(sample_rate_hz);
  }

  // 参考 gyroInitFilters() 的实现方式
  // 步骤1: 读取 LPF1 参数并初始化（参考 gyro_init.c:229-244）
  uint8_t gyro_lpf1_type = 0;
  uint16_t gyro_lpf1_static_hz = 0;
  uint16_t gyro_lpf1_dyn_min_hz = 0;

  if (getParam("filter_gyro_lpf1_type", &gyro_lpf1_type, sizeof(gyro_lpf1_type)) == RT_EOK &&
      getParam("filter_gyro_lpf1_static_hz", &gyro_lpf1_static_hz, sizeof(gyro_lpf1_static_hz)) == RT_EOK) {
    // 读取动态滤波器参数
    if (getParam("filter_gyro_lpf1_dyn_min_hz", &gyro_lpf1_dyn_min_hz, sizeof(gyro_lpf1_dyn_min_hz)) == RT_EOK) {
      // 如果动态滤波器参数大于0，使用动态最小值作为初始频率（参考 gyro_init.c:231-236）
      if (gyro_lpf1_dyn_min_hz > 0) {
        gyro_lpf1_static_hz = gyro_lpf1_dyn_min_hz;
        LOG_I("Dynamic LPF1 enabled, using dyn_min_hz: %u Hz", gyro_lpf1_dyn_min_hz);
      }
    }

    // 初始化 LPF1 滤波器（参考 gyro_init.c:239-244）
    if (gyro_lpf1_static_hz > 0) {
      BfLpfFilterType filter_type = BfLpfFilterType::PT1;  // 默认 PT1
      switch (gyro_lpf1_type) {
        case 1:
          filter_type = BfLpfFilterType::BIQUAD;
          break;
        case 2:
          filter_type = BfLpfFilterType::PT2;
          break;
        case 3:
          filter_type = BfLpfFilterType::PT3;
          break;
        default:
          filter_type = BfLpfFilterType::PT1;
          break;
      }

      if (lpf1_filter_.init(filter_type, static_cast<float>(gyro_lpf1_static_hz), sample_rate_hz)) {
        lpf1_filter_.setEnabled(true);
        LOG_I("LPF1 filter initialized: type=%u, cutoff=%u Hz, sample_rate=%.1f Hz", gyro_lpf1_type, gyro_lpf1_static_hz,
              sample_rate_hz);
      } else {
        LOG_W("LPF1 filter init failed: type=%u, cutoff=%u Hz", gyro_lpf1_type, gyro_lpf1_static_hz);
      }
    }
  }

  // 步骤2: 读取并初始化 LPF2 滤波器（参考 gyro_init.c:246-251）
  uint8_t gyro_lpf2_type = 0;
  uint16_t gyro_lpf2_static_hz = 0;

  if (getParam("filter_gyro_lpf2_type", &gyro_lpf2_type, sizeof(gyro_lpf2_type)) == RT_EOK &&
      getParam("filter_gyro_lpf2_static_hz", &gyro_lpf2_static_hz, sizeof(gyro_lpf2_static_hz)) == RT_EOK) {
    // 初始化 LPF2 滤波器
    if (gyro_lpf2_type > 0 && gyro_lpf2_static_hz > 0) {
      BfLpfFilterType filter_type = BfLpfFilterType::PT1;  // 默认 PT1
      switch (gyro_lpf2_type) {
        case 1:
          filter_type = BfLpfFilterType::BIQUAD;
          break;
        case 2:
          filter_type = BfLpfFilterType::PT2;
          break;
        case 3:
          filter_type = BfLpfFilterType::PT3;
          break;
        default:
          filter_type = BfLpfFilterType::PT1;
          break;
      }

      if (lpf2_filter_.init(filter_type, static_cast<float>(gyro_lpf2_static_hz), sample_rate_hz)) {
        lpf2_filter_.setEnabled(true);
        downsample_filter_enabled_ = true;  // LPF2 开启时使用滤波降采样（对应 gyro.downsampleFilterEnabled）
        LOG_I("LPF2 filter initialized: type=%u, cutoff=%u Hz, sample_rate=%.1f Hz", gyro_lpf2_type,
              gyro_lpf2_static_hz, sample_rate_hz);
      } else {
        LOG_W("LPF2 filter init failed: type=%u, cutoff=%u Hz", gyro_lpf2_type, gyro_lpf2_static_hz);
      }
    }
  }

  // 初始化动态 LPF 参数（参考 gyro_init.c:199-226 中的 dynLpfFilterInit）
#ifdef USE_DYN_LPF
  // 注意：gyro_lpf1_dyn_min_hz 已在上面声明，这里直接使用
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
      // 注意：gyro_lpf1_type 已在上面声明，这里直接使用
      if (gyro_lpf1_type > 0) {
        switch (gyro_lpf1_type) {
          case 1:
            dyn_lpf_filter_ = 1;
            break;  // DYN_LPF_PT1
          case 2:
            dyn_lpf_filter_ = 2;
            break;  // DYN_LPF_BIQUAD
          case 3:
            dyn_lpf_filter_ = 3;
            break;  // DYN_LPF_PT2
          case 4:
            dyn_lpf_filter_ = 4;
            break;  // DYN_LPF_PT3
          default:
            dyn_lpf_filter_ = 0;
            break;  // DYN_LPF_NONE
        }
      }
    } else {
      dyn_lpf_filter_ = 0;  // DYN_LPF_NONE
    }
  }
#endif

  // 步骤3: 初始化 Notch1 滤波器（参考 gyro_init.c:253）
  uint16_t gyro_soft_notch_hz_1 = 0;
  uint16_t gyro_soft_notch_cutoff_1 = 0;

  if (getParam("filter_gyro_soft_notch_hz_1", &gyro_soft_notch_hz_1, sizeof(gyro_soft_notch_hz_1)) == RT_EOK &&
      getParam("filter_gyro_soft_notch_cutoff_1", &gyro_soft_notch_cutoff_1, sizeof(gyro_soft_notch_cutoff_1)) ==
          RT_EOK) {
    if (gyro_soft_notch_hz_1 > 0 && gyro_soft_notch_cutoff_1 > 0) {
      float freq_hz = static_cast<float>(gyro_soft_notch_hz_1);
      float cutoff_hz = static_cast<float>(gyro_soft_notch_cutoff_1);
      if (notch1_filter_.init(freq_hz, cutoff_hz, sample_rate_hz)) {
        notch1_filter_.setEnabled(true);
        LOG_I("Notch1 filter initialized: freq=%u Hz, cutoff=%u Hz", gyro_soft_notch_hz_1, gyro_soft_notch_cutoff_1);
      } else {
        LOG_W("Notch1 filter init failed: freq=%u Hz, cutoff=%u Hz", gyro_soft_notch_hz_1, gyro_soft_notch_cutoff_1);
      }
    }
  }

  // 步骤4: 初始化 Notch2 滤波器（参考 gyro_init.c:254）
  uint16_t gyro_soft_notch_hz_2 = 0;
  uint16_t gyro_soft_notch_cutoff_2 = 0;

  if (getParam("filter_gyro_soft_notch_hz_2", &gyro_soft_notch_hz_2, sizeof(gyro_soft_notch_hz_2)) == RT_EOK &&
      getParam("filter_gyro_soft_notch_cutoff_2", &gyro_soft_notch_cutoff_2, sizeof(gyro_soft_notch_cutoff_2)) ==
          RT_EOK) {
    if (gyro_soft_notch_hz_2 > 0 && gyro_soft_notch_cutoff_2 > 0) {
      float freq_hz = static_cast<float>(gyro_soft_notch_hz_2);
      float cutoff_hz = static_cast<float>(gyro_soft_notch_cutoff_2);
      if (notch2_filter_.init(freq_hz, cutoff_hz, sample_rate_hz)) {
        notch2_filter_.setEnabled(true);
        LOG_I("Notch2 filter initialized: freq=%u Hz, cutoff=%u Hz", gyro_soft_notch_hz_2, gyro_soft_notch_cutoff_2);
      } else {
        LOG_W("Notch2 filter init failed: freq=%u Hz, cutoff=%u Hz", gyro_soft_notch_hz_2, gyro_soft_notch_cutoff_2);
      }
    }
  }

  // TODO: 初始化动态 notch 滤波器（参考 gyro_init.c:258-259）
  // 需要 dynNotchConfig_t 结构体，可以从参数系统读取或使用默认值
  // dyn_notch_filter_.init(config, target_looptime_us);
}

void RateCtrlAngularVelocity::initDownsampleFilter() {
  // 参考 gyro_init.c:262-265 中的 imuGyroFilter 初始化
  // const float k = pt1FilterGain(GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ, gyro.targetLooptime * 1e-6f);
  // for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
  //     pt1FilterInit(&gyro.imuGyroFilter[axis], k);
  // }

  const float GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ = 200.0f;  // 对应 gyro.h:52
  float target_looptime_s = target_looptime_us_ * 1e-6f;

  if (target_looptime_s > 0.0f && target_looptime_us_ > 0) {
    // 计算采样频率（参考 gyro_init.c，使用 targetLooptime）
    float sample_rate_hz = 1.0f / target_looptime_s;
    float cutoff_hz = GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ;

    for (int i = 0; i < 3; i++) {
      if (imu_gyro_filter_[i].init(BfLpfFilterType::PT1, cutoff_hz, sample_rate_hz)) {
        imu_gyro_filter_[i].setEnabled(true);
      }
    }

    LOG_I("IMU downsample filter initialized: cutoff=%.1f Hz, sample_rate=%.1f Hz", cutoff_hz, sample_rate_hz);
  }
}

void RateCtrlAngularVelocity::setTargetLooptime(uint8_t pid_denom) {
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
void RateCtrlAngularVelocity::initInThreadEntry() {
  // 在线程调度器启动后，等待 BMI270 初始化完成（使用初始化同步机制）
  // 这部分初始化依赖其他线程状态，必须在线程入口函数中执行
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG1_HIGH();  // Debug pin: Start waiting for BMI270 initialization
#endif

  rt_err_t ret = initSyncWait(INIT_SYNC_BMI270, 5000);  // 等待最多5秒

#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG1_LOW();  // Debug pin: End waiting for BMI270 initialization
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
  // 注意：这些初始化依赖 sample_rate_hz_，所以必须在这里执行
  initFilters();

  // 初始化降采样滤波器（参考 gyro_init.c:262-265）
  // 注意：这些初始化依赖 target_looptime_us_，所以必须在这里执行
  initDownsampleFilter();

  // 通知 Gyro Filter 初始化完成
  initSyncNotify(INIT_SYNC_GYRO_FILTER);
}

