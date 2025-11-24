#include "gyro_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "gyro_filter"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "debugPin.h"
}

#include <cstring>

#include "bfImuFilterInit.h"
#include "../imu/inc/bmi270_class.h"
#include "bfSensorAlignment.hpp"
#include "bfNotchFilter.hpp"
#include "bfDynNotchFilter.hpp"
#include "../log/inc/mlog_gyro.hpp"
#include "gyro_class.h"

extern "C" {
#include "param.h"
#include "bfImuFilterParam.h"
// #include "rpmFilter.h"  // TODO: RPM 滤波器暂未实现，先注释
#include "timestamp.h"
#include "uMCN.h"
}

/* 定义滤波后陀螺仪数据话题（在本文件内完成定义与发布） */
MCN_DEFINE(gyro, sizeof(gyro_filtered_msg_t));

// MCN echo 函数（参考 accgyro_spi_bmi270.cpp 中的 imu_raw_echo）
static int gyro_filtered_echo(void* parameter) {
  gyro_filtered_msg_t gyro_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &gyro_data) != RT_EOK) {
    return -1;
  }

  LOG_I("seq: %lu, gyro_filtered: %.2f, %.2f, %.2f, gyro_adc: %.2f, %.2f, %.2f", gyro_data.seq,
        gyro_data.gyro_filtered[0], gyro_data.gyro_filtered[1], gyro_data.gyro_filtered[2], gyro_data.gyro_adc[0],
        gyro_data.gyro_adc[1], gyro_data.gyro_adc[2]);
  return 0;
}

// IMU 采样频率（Hz）- 假设从 IMU 发布频率推算
#ifndef PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ
#define PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ 800.0f
#endif

// RateCtrlAngularVelocity 单例实现
RateCtrlAngularVelocity& RateCtrlAngularVelocity::instance() {
  static RateCtrlAngularVelocity instance_obj;
  return instance_obj;
}

// GyroCalibration 实现
GyroCalibration::GyroCalibration()
    : calibration_complete_(false),
      calibrating_(false),
      calibration_cycles_remaining_(0),
      calibration_cycles_total_(0),
      movement_threshold_(0.0f),
      yaw_offset_centidegrees_(0) {
  std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
  std::memset(gyro_zero_, 0, sizeof(gyro_zero_));
}

void GyroCalibration::startCalibration(float sample_rate_hz, uint32_t calibration_duration_ms, float movement_threshold,
                                       int16_t yaw_offset_centidegrees) {
  calibration_complete_ = false;
  calibrating_ = true;
  movement_threshold_ = movement_threshold;
  yaw_offset_centidegrees_ = yaw_offset_centidegrees;

  // 计算校准周期数：(duration_ms * sample_rate_hz) / 1000
  calibration_cycles_total_ = static_cast<uint32_t>((calibration_duration_ms * sample_rate_hz) / 1000.0f);
  if (calibration_cycles_total_ < 1) {
    calibration_cycles_total_ = 1;
  }
  calibration_cycles_remaining_ = calibration_cycles_total_;

  std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
  for (int i = 0; i < 3; i++) {
    calibration_var_[i].clear();
    gyro_zero_[i] = 0.0f;
  }
}

bool GyroCalibration::isOnFirstCycle() const { return calibration_cycles_remaining_ == calibration_cycles_total_; }

bool GyroCalibration::isOnFinalCycle() const { return calibration_cycles_remaining_ == 1; }

bool GyroCalibration::updateCalibration(const float gyro_raw[3]) {
  if (!calibrating_ || calibration_complete_) {
    return false;
  }

  bool cal_failed = false;

  for (int axis = 0; axis < 3; axis++) {
    // 在第一个周期重置统计
    if (isOnFirstCycle()) {
      calibration_sum_[axis] = 0.0f;
      calibration_var_[axis].clear();
      gyro_zero_[axis] = 0.0f;
    }

    // 累加值和统计标准差
    calibration_sum_[axis] += gyro_raw[axis];
    calibration_var_[axis].push(gyro_raw[axis]);

    // 在最后一个周期计算零偏值
    if (isOnFinalCycle()) {
      float stddev = calibration_var_[axis].standardDeviation();

      // 检查标准差（运动检测）
      if (movement_threshold_ > 0.0f && stddev > movement_threshold_) {
        cal_failed = true;
      } else {
        // 计算零偏值（平均值）
        gyro_zero_[axis] = calibration_sum_[axis] / calibration_cycles_total_;

        // 对 yaw 轴应用偏移（centidegrees 转成 float）
        if (axis == 2) {  // Z 轴是 yaw
          gyro_zero_[axis] -= (static_cast<float>(yaw_offset_centidegrees_) / 100.0f);
        }
      }
    }
  }

  // 如果检测到运动，重新开始校准
  if (cal_failed) {
    calibration_cycles_remaining_ = calibration_cycles_total_;
    std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
    for (int i = 0; i < 3; i++) {
      calibration_var_[i].clear();
    }
    return false;
  }

  // 在最后一个周期，校准完成
  if (isOnFinalCycle()) {
    calibrating_ = false;
    calibration_complete_ = true;
    calibration_cycles_remaining_ = 0;
    return true;  // 校准完成
  }

  // 减少剩余周期数
  calibration_cycles_remaining_--;
  return false;  // 校准进行中
}

void GyroCalibration::getGyroZero(float gyro_zero[3]) const { std::memcpy(gyro_zero, gyro_zero_, sizeof(gyro_zero_)); }

void GyroCalibration::setGyroZero(const float gyro_zero[3]) {
  std::memcpy(gyro_zero_, gyro_zero, sizeof(gyro_zero_));
  // 设置零偏值后，标记为已校准完成
  calibration_complete_ = true;
  calibrating_ = false;
}

void GyroCalibration::applyZeroOffset(const float gyro_raw[3], float gyro_corrected[3]) const {
  if (!calibration_complete_) {
    // 如果未校准完成，直接返回原始值
    std::memcpy(gyro_corrected, gyro_raw, sizeof(float) * 3);
    return;
  }

  for (int i = 0; i < 3; i++) {
    gyro_corrected[i] = gyro_raw[i] - gyro_zero_[i];
  }
}

RateCtrlAngularVelocity::RateCtrlAngularVelocity()
    :  // 从 gyro_t 映射的简单变量初始化（参考 gyro_t 的 FAST_DATA_ZERO_INIT）
      sample_rate_hz_(0),
      target_looptime_us_(0),
      sample_looptime_us_(0),
      sample_count_(0),
      downsample_filter_enabled_(false),
      gyro_enabled_bitmask_(0),
      gyro_debug_mode_(0),
      gyro_has_overflow_protection_(true),
      use_multi_gyro_debugging_(false),
      gyro_debug_axis_(0),
      acc_sample_rate_hz_(0),
#ifdef USE_DYN_LPF
      dyn_lpf_filter_(0),
      dyn_lpf_min_(0),
      dyn_lpf_max_(0),
      dyn_lpf_curve_expo_(0),
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
      overflow_axis_mask_(0),
#endif
      // 其他辅助成员
      gyro_align_(BfSensorAlignment::ALIGN_DEFAULT),
      use_custom_matrix_(false),
      calibration_started_(false),
      // 线程相关
      thread_(RT_NULL),
      thread_inited_(false),
      // MCN 订阅和发布相关
      imu_event_(RT_NULL),
      imu_node_(RT_NULL),
      gyro_filtered_hub_(nullptr) {
  // 清零数组
  std::memset(&thread_obj_, 0, sizeof(thread_obj_));
  std::memset(thread_stack_, 0, sizeof(thread_stack_));
  std::memset(gyro_adc_, 0, sizeof(gyro_adc_));
  std::memset(gyro_adcf_, 0, sizeof(gyro_adcf_));
  std::memset(sample_sum_, 0, sizeof(sample_sum_));
  std::memset(rotation_matrix_, 0, sizeof(rotation_matrix_));
}

rt_err_t RateCtrlAngularVelocity::init() {
  if (thread_inited_) {
    LOG_W("RateCtrlAngularVelocity already initialized");
    return RT_EOK;
  }

  // 创建 MCN 事件信号量（用于 mcn_poll_sync）
  if (imu_event_ == RT_NULL) {
    imu_event_ = rt_sem_create("gyro_evt", 0, RT_IPC_FLAG_FIFO);
    if (imu_event_ == RT_NULL) {
      LOG_E("create imu event semaphore failed");
      return -RT_ERROR;
    }
  }

  // 订阅 imu MCN 节点（传入 event 用于同步等待）
  imu_node_ = mcn_subscribe(MCN_HUB(imu), imu_event_, RT_NULL);
  if (imu_node_ == RT_NULL) {
    LOG_E("subscribe imu topic failed");
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }
  LOG_I("Subscribed to imu MCN topic");

  // 获取 gyro MCN hub（用于发布滤波后的数据）
  gyro_filtered_hub_ = MCN_HUB(gyro);
  if (gyro_filtered_hub_ == nullptr) {
    LOG_E("get gyro hub failed");
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(imu), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return -RT_ERROR;
  }

  // 激活 gyro MCN 主题（必须调用，否则 mcn_publish 会失败）
  // 参考 accgyro_spi_bmi270.cpp:544 的 imu 主题激活方式
  rt_err_t advertise_ret = mcn_advertise(gyro_filtered_hub_, gyro_filtered_echo);
  if (advertise_ret != RT_EOK && advertise_ret != -RT_EBUSY) {
    // RT_EOK: 成功激活
    // -RT_EBUSY: 已经激活过了（可以忽略）
    // 其他值: 激活失败（内存不足等）
    LOG_E("gyro_filtered advertise failed: %d", advertise_ret);
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(imu), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return advertise_ret;
  }
  LOG_I("gyro MCN topic advertised");

  // 初始化 mlog_gyro（使用单例）
  bf_mlog::MlogGyro* mlog_gyro = bf_mlog::MlogGyro::getInstance();
  mlog_gyro->init();
  // 从参数系统读取 mlog_gyro_en 参数并设置使能状态
  uint8_t mlog_gyro_en = 0;
  if (getParam("mlog_gyro_en", &mlog_gyro_en, sizeof(mlog_gyro_en)) == RT_EOK) {
    mlog_gyro->setParamEnabled(mlog_gyro_en != 0);
    LOG_I("Mlog gyro enabled: %u", mlog_gyro_en);
  } else {
    // 如果参数不存在，使用默认值（禁用）
    mlog_gyro->setParamEnabled(false);
    LOG_W("Mlog gyro parameter not found, disabled by default");
  }

  // 从 BMI270 单例获取 IMU 采样频率和周期（参考 gyro.c 的初始化）
  bf_bmi270::BMI270& bmi270 = bf_bmi270::BMI270::instance();
  float sample_rate_hz = 0.0f;
  float sample_dt = 0.0f;

  // 等待 BMI270 初始化完成（可能 gyro 启动比 bmi270 早，需要等待）
  const uint32_t wait_interval_ms = 100;  // 每次等待 100ms
  const uint32_t max_wait_count = 10;     // 最多等待 10 次（即 1 秒）
  uint32_t wait_count = 0;
  bool bmi270_ready = false;

  while (wait_count < max_wait_count) {
    if (bmi270.initialized()) {
      bmi270_ready = true;
      break;
    }
    wait_count++;
    LOG_I("Waiting for BMI270 initialization... (%u/%u)", wait_count, max_wait_count);
    rt_thread_mdelay(wait_interval_ms);
  }

  if (bmi270_ready) {
    sample_rate_hz = bmi270.getGyroSampleRateHz();
    sample_dt = bmi270.getGyroSampleDt();
    LOG_I("IMU sample rate: %.1f Hz, dt: %.6f s", sample_rate_hz, sample_dt);
  } else {
    // 如果等待 10 次后 BMI270 仍未初始化，使用默认值
    sample_rate_hz = PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ;
    sample_dt = 1.0f / sample_rate_hz;
    LOG_W("BMI270 not initialized after waiting %u ms, using default sample rate: %.1f Hz",
          wait_count * wait_interval_ms, sample_rate_hz);
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

  // 初始化调试和状态相关变量（参考 gyro.c:592-594）
  gyro_has_overflow_protection_ = true;  // 默认有溢出保护
  gyro_debug_mode_ = 0;                  // DEBUG_NONE

  use_multi_gyro_debugging_ = false;
  gyro_debug_axis_ = 0;  // FD_ROLL (需要根据实际枚举值调整)

  // 初始化滤波器（从参数系统读取配置，参考 gyroInitFilters）
  initFilters();

  // 初始化降采样滤波器（参考 gyro_init.c:262-265）
  initDownsampleFilter();

  // 注意：gyro 零偏值在运行时计算，不保存到参数系统
  // 每次启动时都会重新校准

  // 创建静态线程
  rt_err_t err = rt_thread_init(&thread_obj_, "gyro_filter", RateCtrlAngularVelocity::threadEntry, this, thread_stack_,
                                PROJECT_BF_GYRO_FILTER_THREAD_STACK_SIZE, PROJECT_BF_GYRO_FILTER_THREAD_PRIORITY,
                                PROJECT_BF_GYRO_FILTER_THREAD_TIMESLICE);

  if (err != RT_EOK) {
    LOG_E("RateCtrlAngularVelocity thread init failed: %d", err);
    if (imu_node_ != RT_NULL) {
      mcn_unsubscribe(MCN_HUB(imu), imu_node_);
      imu_node_ = RT_NULL;
    }
    if (imu_event_ != RT_NULL) {
      rt_sem_delete(imu_event_);
      imu_event_ = RT_NULL;
    }
    return err;
  }

  thread_ = &thread_obj_;
  thread_inited_ = true;

  // 启动线程
  rt_thread_startup(thread_);
  LOG_I("RateCtrlAngularVelocity thread started");

  LOG_I("RateCtrlAngularVelocity initialized");
  return RT_EOK;
}

void RateCtrlAngularVelocity::initFilters() {
  // 使用从成员变量获取的采样频率和周期
  float sample_rate_hz = static_cast<float>(sample_rate_hz_);

  // 如果无效，使用默认值
  if (sample_rate_hz <= 0.0f) {
    sample_rate_hz = PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ;
    sample_rate_hz_ = static_cast<uint16_t>(sample_rate_hz);
  }

  // 使用目标循环时间（微秒）- 用于滤波器初始化
  // 注意：target_looptime_us 目前未直接使用，但保留用于未来可能的动态滤波器
  // uint32_t target_looptime_us = target_looptime_us_;

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
    if (gyro_lpf1_type > 0 && gyro_lpf1_static_hz > 0) {
      BfLpfFilterType filter_type = static_cast<BfLpfFilterType>(gyro_lpf1_type);
      float cutoff_hz = static_cast<float>(gyro_lpf1_static_hz);
      if (lpf1_filter_.init(filter_type, cutoff_hz, sample_rate_hz)) {
        lpf1_filter_.setEnabled(true);
        LOG_I("LPF1 filter initialized: type=%u, cutoff=%u Hz, sample_rate=%.1f Hz", gyro_lpf1_type,
              gyro_lpf1_static_hz, sample_rate_hz);
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
      BfLpfFilterType filter_type = static_cast<BfLpfFilterType>(gyro_lpf2_type);
      float cutoff_hz = static_cast<float>(gyro_lpf2_static_hz);
      if (lpf2_filter_.init(filter_type, cutoff_hz, sample_rate_hz)) {
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

void RateCtrlAngularVelocity::threadEntry(void* parameter) {
  if (parameter == RT_NULL) {
    return;
  }

  RateCtrlAngularVelocity* instance = static_cast<RateCtrlAngularVelocity*>(parameter);
  instance->threadLoop();
}

void RateCtrlAngularVelocity::threadLoop() {
  imu_raw_msg_t imu_data;

  LOG_I("RateCtrlAngularVelocity thread loop started");

  while (true) {
    // 阻塞等待 MCN 发布
    if (mcn_poll_sync(imu_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      // 复制数据
      if (mcn_copy(MCN_HUB(imu), imu_node_, &imu_data) == RT_EOK) {
        // 处理 IMU 数据
        processImuData(&imu_data);
        // Debug Pin: 拉低，表示滤波处理完成
      }
    }
  }
}

void RateCtrlAngularVelocity::processImuData(const imu_raw_msg_t* imu_data) {
  if (imu_data == RT_NULL) {
    return;
  }
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG0_HIGH();
#endif
  float gyro_raw[3] = {imu_data->gyro[0], imu_data->gyro[1], imu_data->gyro[2]};

  // 如果没有校准，开始校准
  if (!calibration_started_ && !gyro_calibration_.isCalibrationComplete()) {
    // 从 Kconfig 获取校准参数
    uint32_t calibration_duration_ms = PROJECT_BF_GYRO_FILTER_CALIBRATION_DURATION_MS;
    float movement_threshold = static_cast<float>(PROJECT_BF_GYRO_FILTER_MOVEMENT_THRESHOLD);
    int16_t yaw_offset = static_cast<int16_t>(PROJECT_BF_GYRO_FILTER_OFFSET_YAW);

    gyro_calibration_.startCalibration(
        sample_rate_hz_ > 0 ? static_cast<float>(sample_rate_hz_) : PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ,
        calibration_duration_ms, movement_threshold, yaw_offset);
    calibration_started_ = true;
    LOG_I("Starting gyro calibration: duration=%u ms, threshold=%.1f", calibration_duration_ms, movement_threshold);
  }

  // 如果正在校准，更新校准
  if (gyro_calibration_.isCalibrating()) {
    bool cal_complete = gyro_calibration_.updateCalibration(gyro_raw);

    if (cal_complete) {
      // 校准完成，零偏值在运行时使用，不保存到参数系统
      float gyro_zero[3];
      gyro_calibration_.getGyroZero(gyro_zero);
      LOG_I("Gyro calibration complete! Zero=[%.3f, %.3f, %.3f] (runtime only, not saved)", gyro_zero[0], gyro_zero[1],
            gyro_zero[2]);
      // 校准完成后，继续执行后续的处理和发布逻辑
    } else {
      // 校准进行中，不处理数据也不发布，直接返回
      LOG_D("Calibrating... (no data published during calibration)");
      return;  // 校准期间不发布数据
    }
  }

  // 只有校准完成后才进行滤波处理和发布
  if (!gyro_calibration_.isCalibrationComplete()) {
    // 如果还未开始校准或校准未完成，不发布数据
    return;
  }

  // 步骤1: 应用零偏值校正（去零飘）
  float gyro_corrected[3];
  gyro_calibration_.applyZeroOffset(gyro_raw, gyro_corrected);

  // 步骤2: 应用传感器旋转（如果需要）
  float gyro_rotated[3];
  std::memcpy(gyro_rotated, gyro_corrected, sizeof(gyro_corrected));

  if (use_custom_matrix_) {
    BfSensorAlignmentUtil::alignViaMatrix(gyro_rotated, rotation_matrix_);
  } else {
    BfSensorAlignmentUtil::alignViaRotation(gyro_rotated, gyro_align_);
  }

  // 步骤3: 降采样（参考 gyro.c:457-468 中的 gyroUpdate 降采样逻辑）
  // 将旋转后的数据存储到 gyro_adc_（对应 gyro.gyroADC）
  std::memcpy(gyro_adc_, gyro_rotated, sizeof(gyro_rotated));

  float gyro_downsampled[3];
  if (downsample_filter_enabled_) {
    // 使用 LPF2 滤波器进行降采样（参考 gyro.c:457-461）
    // gyro.sampleSum[X] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[X], gyro.gyroADC[X]);
    if (lpf2_filter_.isEnabled()) {
      lpf2_filter_.apply(gyro_adc_, gyro_downsampled);
      // 将滤波结果存储到 sample_sum_（对应 gyro.sampleSum）
      std::memcpy(sample_sum_, gyro_downsampled, sizeof(gyro_downsampled));
    } else {
      std::memcpy(gyro_downsampled, gyro_adc_, sizeof(gyro_adc_));
    }
  } else {
    // 使用简单平均值进行降采样（参考 gyro.c:462-467）
    // gyro.sampleSum[X] += gyro.gyroADC[X];
    // gyro.sampleCount++;
    for (int i = 0; i < 3; i++) {
      sample_sum_[i] += gyro_adc_[i];
    }
    sample_count_++;

    // 这里暂时不进行平均，等待累积到一定数量后再平均
    // 为了简化，先直接使用当前值
    std::memcpy(gyro_downsampled, gyro_adc_, sizeof(gyro_adc_));

    // TODO: 实现累积平均逻辑（参考 gyro_filter_impl.c:39-42）
    // if (gyro.sampleCount) {
    //     gyroADCf = gyro.sampleSum[axis] / gyro.sampleCount;
    // }
  }

  // 步骤4: 应用完整的滤波链（参考 gyro_filter_impl.c）
  // 将结果存储到 gyro_adcf_（对应 gyro.gyroADCf）
  applyFilterChain(gyro_downsampled, gyro_adcf_);
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG0_LOW();
#endif

  // 重置降采样计数器（参考 gyro_filter_impl.c:86）
  if (!downsample_filter_enabled_) {
    sample_count_ = 0;
    std::memset(sample_sum_, 0, sizeof(sample_sum_));
  }

  // 发布滤波后的陀螺仪数据到 MCN
  gyro_filtered_msg_t filtered_msg;
  std::memcpy(filtered_msg.gyro_filtered, gyro_adcf_, sizeof(gyro_adcf_));
  std::memcpy(filtered_msg.gyro_adc, gyro_adc_, sizeof(gyro_adc_));
  filtered_msg.seq = imu_data->seq;

  if (gyro_filtered_hub_ != nullptr) {
    rt_err_t publish_result = mcn_publish(gyro_filtered_hub_, &filtered_msg);
    if (publish_result != RT_EOK) {
      LOG_E("Failed to publish gyro_filtered data: %d", publish_result);
    }
  } else {
    LOG_E("gyro hub is null, cannot publish data");
  }

  // 推送陀螺仪数据到 mlog（参考 aMlogStabilze.c:208-216）
  // 记录滤波前后的陀螺仪数据
  uint32_t timestamp = timestamp_micros();
  bf_mlog::MlogGyro::getInstance()->pushGyroData(imu_data->seq, timestamp, gyro_adc_, gyro_adcf_);

  // Debug log（可选，如果需要的话）
  // LOG_D("seq:%u angular_velocity(%.3f, %.3f, %.3f)",
  //     imu_data->seq,
  //     gyro_adcf_[0],
  //     gyro_adcf_[1],
  //     gyro_adcf_[2]);
}

void RateCtrlAngularVelocity::applyFilterChain(const float input[3], float output[3]) {
  // 参考 gyro_filter_impl.c 的滤波顺序
  float filtered[3];
  std::memcpy(filtered, input, sizeof(float) * 3);

  // TODO: 应用 RPM 滤波器（如果使能）
  // rpmFilter(gyroData, rpmData, filtered);

  // 应用静态 notch 滤波器
  if (notch1_filter_.isEnabled()) {
    float temp[3];
    notch1_filter_.apply(filtered, temp);
    std::memcpy(filtered, temp, sizeof(float) * 3);
  }

  if (notch2_filter_.isEnabled()) {
    float temp[3];
    notch2_filter_.apply(filtered, temp);
    std::memcpy(filtered, temp, sizeof(float) * 3);
  }

  // 应用 LPF1 滤波器
  if (lpf1_filter_.isEnabled()) {
    float temp[3];
    lpf1_filter_.apply(filtered, temp);
    std::memcpy(filtered, temp, sizeof(float) * 3);
  }

  // 应用动态 notch 滤波器（如果激活）
  if (dyn_notch_filter_.isActive()) {
    // 推送样本用于频率分析
    for (int i = 0; i < 3; i++) {
      dyn_notch_filter_.push(i, filtered[i]);
    }

    // 更新动态 notch 滤波器
    dyn_notch_filter_.update();

    // 应用动态 notch 滤波器
    dyn_notch_filter_.apply3Axis(filtered, filtered);
  }

  std::memcpy(output, filtered, sizeof(float) * 3);
}

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_GYRO_FILTER_EN
extern "C" {
static int gyro_filter_init_wrapper(void) {
  RateCtrlAngularVelocity& instance = RateCtrlAngularVelocity::instance();
  rt_err_t ret = instance.init();
  if (ret == RT_EOK) {
    LOG_I("GyroFilter auto-init success");
  } else {
    LOG_E("GyroFilter auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(gyro_filter_init_wrapper);
}
#endif
