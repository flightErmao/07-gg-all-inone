#include "gyro_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "gyro"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "debugPin.h"
#include "timestamp.h"
}

#include <cstring>

#include "bfSensorAlignment.hpp"
#include "bfNotchFilter.hpp"
#include "bfDynNotchFilter.hpp"

// MCN 定义和 echo 函数已移到 gyro_mcn.cpp

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

  // 初始化 MCN（订阅和发布）
  rt_err_t ret = initMcn();
  if (ret != RT_EOK) {
    return ret;
  }

  // 初始化 Mlog
  ret = initMlog();
  if (ret != RT_EOK) {
    cleanupMcnSubscriptions();
    return ret;
  }

  // 注意：依赖 BMI270 的初始化已移到 threadEntry 中，在线程调度器启动后执行
  // 这里只初始化不依赖其他线程的部分

  // 初始化调试和状态相关变量（参考 gyro.c:592-594）
  gyro_has_overflow_protection_ = true;  // 默认有溢出保护
  gyro_debug_mode_ = 0;                  // DEBUG_NONE

  use_multi_gyro_debugging_ = false;
  gyro_debug_axis_ = 0;  // FD_ROLL (需要根据实际枚举值调整)

  // 注意：gyro 零偏值在运行时计算，不保存到参数系统
  // 每次启动时都会重新校准

  // 创建静态线程
  rt_err_t err = rt_thread_init(&thread_obj_, "gyro", RateCtrlAngularVelocity::threadEntry, this, thread_stack_,
                                PROJECT_BF_GYRO_FILTER_THREAD_STACK_SIZE, PROJECT_BF_GYRO_FILTER_THREAD_PRIORITY,
                                PROJECT_BF_GYRO_FILTER_THREAD_TIMESLICE);

  if (err != RT_EOK) {
    LOG_E("RateCtrlAngularVelocity thread init failed: %d", err);
    cleanupMcnSubscriptions();
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

// initFilters(), initDownsampleFilter(), setTargetLooptime() 已移到 gyro_init.cpp

void RateCtrlAngularVelocity::threadEntry(void* parameter) {
  if (parameter == RT_NULL) {
    return;
  }

  RateCtrlAngularVelocity* instance = static_cast<RateCtrlAngularVelocity*>(parameter);
  
  // 在线程调度器启动后，执行依赖其他线程状态的初始化
  // 这部分初始化已移到 initInThreadEntry() 中
  instance->initInThreadEntry();
  
  // 进入主循环
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

  // 发布滤波后的陀螺仪数据到 MCN（已移到 gyro_mcn.cpp）
  publishGyroFiltered(imu_data);

  // 推送陀螺仪数据到 mlog（已移到 gyro_mlog.cpp）
  pushGyroDataToMlog(imu_data);

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
