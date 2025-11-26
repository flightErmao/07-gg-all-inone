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

#ifdef USE_DYN_NOTCH_FILTER
#include "gyro_dnf.h"  // For GyroDynNotch class
#endif

extern "C" {
#include "sensor_alignment.h"  // For sensor alignment
#include "vector.h"            // For matrix operations
}
extern "C" {
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
}

// MCN 定义和 echo 函数已移到 gyro_mcn.cpp

// IMU 采样频率（Hz）- 假设从 IMU 发布频率推算
#ifndef PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ
#define PROJECT_BF_GYRO_FILTER_IMU_SAMPLE_RATE_HZ 800.0f
#endif

// gyro 单例实现
gyro& gyro::instance() {
  static gyro instance_obj;
  return instance_obj;
}

gyro::gyro()
    :  // 从 gyro_t 映射的简单变量初始化（参考 gyro_t 的 FAST_DATA_ZERO_INIT）
      sample_rate_hz_(0),
      target_looptime_us_(0),
      sample_looptime_us_(0),
      sample_count_(0),
      downsample_filter_enabled_(false),
#ifdef USE_DYN_LPF
      dyn_lpf_filter_(0),
      dyn_lpf_min_(0),
      dyn_lpf_max_(0),
      dyn_lpf_curve_expo_(0),
#endif
      // 滤波器初始化
      lpf1_apply_fn_(nullFilterApply),
      lpf1_type_(FILTER_PT1),
      lpf1_enabled_(false),
      lpf2_apply_fn_(nullFilterApply),
      lpf2_type_(FILTER_PT1),
      lpf2_enabled_(false),
      notch1_apply_fn_(nullFilterApply),
      notch1_enabled_(false),
      notch2_apply_fn_(nullFilterApply),
      notch2_enabled_(false),
#ifdef USE_DYN_NOTCH_FILTER
      dyn_notch_filter_(nullptr),
      dyn_notch_enabled_(false),
#endif
      imu_gyro_filter_enabled_(false),
      // 其他辅助成员
      gyro_align_(ALIGN_DEFAULT),
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
  std::memset(&rotation_matrix_, 0, sizeof(rotation_matrix_));
  std::memset(lpf1_filter_, 0, sizeof(lpf1_filter_));
  std::memset(lpf2_filter_, 0, sizeof(lpf2_filter_));
  std::memset(notch1_filter_, 0, sizeof(notch1_filter_));
  std::memset(notch2_filter_, 0, sizeof(notch2_filter_));
  std::memset(imu_gyro_filter_, 0, sizeof(imu_gyro_filter_));
  std::memset(gyroFilteredDownsampled_, 0, sizeof(gyroFilteredDownsampled_));

#ifdef USE_DYN_NOTCH_FILTER
  // 动态陷波滤波器使用指针，在 initInThreadEntry 中创建
  dyn_notch_filter_ = nullptr;
#endif
}

rt_err_t gyro::init() {
  if (thread_inited_) {
    LOG_W("gyro already initialized");
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

  rt_err_t err = rt_thread_init(&thread_obj_, "gyro", gyro::threadEntry, this, thread_stack_,
                                PROJECT_BF_GYRO_FILTER_THREAD_STACK_SIZE, PROJECT_BF_GYRO_FILTER_THREAD_PRIORITY,
                                PROJECT_BF_GYRO_FILTER_THREAD_TIMESLICE);

  if (err != RT_EOK) {
    LOG_E("gyro thread init failed: %d", err);
    cleanupMcnSubscriptions();
    return err;
  }

  thread_ = &thread_obj_;
  thread_inited_ = true;

  // 启动线程
  rt_thread_startup(thread_);
  LOG_I("gyro thread started");

  LOG_I("gyro initialized");
  return RT_EOK;
}

void gyro::threadEntry(void* parameter) {
  if (parameter == RT_NULL) {
    return;
  }

  gyro* instance = static_cast<gyro*>(parameter);

  // 在线程调度器启动后，执行依赖其他线程状态的初始化
  // 这部分初始化已移到 initInThreadEntry() 中
  instance->initInThreadEntry();
  
  // 进入主循环
  instance->threadLoop();
}

void gyro::threadLoop() {
  imu_raw_msg_t imu_data;

  LOG_I("gyro thread loop started");

  while (true) {
    if (mcn_poll_sync(imu_node_, RT_WAITING_FOREVER) == RT_TRUE) {
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
      DEBUG_PIN_DEBUG0_HIGH();
#endif
      if (mcn_copy(MCN_HUB(imu), imu_node_, &imu_data) == RT_EOK) {
        processImuData(&imu_data);
#ifdef PROJECT_BF_GYRO_FILTER_DEBUG_PIN_EN
        DEBUG_PIN_DEBUG0_LOW();
#endif
      }
    }
  }
}

void gyro::processImuData(const imu_raw_msg_t* imu_data) {
  if (imu_data == RT_NULL) {
    return;
  }
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
    // 使用自定义旋转矩阵
    vector3_t vec_in = {gyro_rotated[0], gyro_rotated[1], gyro_rotated[2]};
    vector3_t vec_out;
    matrixVectorMul(&vec_out, &rotation_matrix_, &vec_in);
    gyro_rotated[0] = vec_out.x;
    gyro_rotated[1] = vec_out.y;
    gyro_rotated[2] = vec_out.z;
  } else {
    // 使用标准对齐方式
    buildAlignmentFromStandardAlignment(&gyro_align_rpy_, gyro_align_);
    buildRotationMatrixFromAngles(&rotation_matrix_, &gyro_align_rpy_);
    vector3_t vec_in = {gyro_rotated[0], gyro_rotated[1], gyro_rotated[2]};
    vector3_t vec_out;
    matrixVectorMul(&vec_out, &rotation_matrix_, &vec_in);
    gyro_rotated[0] = vec_out.x;
    gyro_rotated[1] = vec_out.y;
    gyro_rotated[2] = vec_out.z;
  }

  // 步骤1: 原始数据采集 - 存储到 gyro_adc_（对应 gyro.gyroADC[axis]）
  std::memcpy(gyro_adc_, gyro_rotated, sizeof(gyro_rotated));

  // 步骤2: 第一个低通滤波器 - LPF2（降采样，如果启用）
  // 位置：gyroUpdate() - src/main/sensors/gyro.c:457-461
  // 如果启用：使用 gyro.lowpass2Filter 进行降采样
  // 如果未启用：使用简单平均
  // 输出：gyro.sampleSum[axis]
  float gyro_downsampled[3];
  if (downsample_filter_enabled_ && lpf2_enabled_) {
    // 使用 LPF2 滤波器进行降采样（参考 gyro.c:457-461）
    // gyro.sampleSum[X] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[X], gyro.gyroADC[X]);
    for (int i = 0; i < 3; i++) {
      gyro_downsampled[i] = lpf2_apply_fn_((filter_t*)&lpf2_filter_[i], gyro_adc_[i]);
    }
    // 将滤波结果存储到 sample_sum_（对应 gyro.sampleSum）
    std::memcpy(sample_sum_, gyro_downsampled, sizeof(gyro_downsampled));
  } else {
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

  // 步骤3-7: 应用完整的滤波链（参考 gyro_filter_impl.c）
  // 顺序：RPM滤波器 -> Notch1 -> Notch2 -> LPF1 -> 动态Notch
  // 输出：gyro.gyroADCf[axis]
  applyFilterChain(gyro_downsampled, gyro_adcf_);

  // 步骤8: 第三个低通滤波器 - PT1（用于姿态估计）
  // 位置：gyroFiltering() - src/main/sensors/gyro.c:541
  // 使用：gyro.imuGyroFilter
  // 截止频率：200 Hz (GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ)
  // 输出：gyroFilteredDownsampled[axis]（给 attitude 使用）
  // gyroFilteredDownsampled[axis] = pt1FilterApply(&gyro.imuGyroFilter[axis], gyro.gyroADCf[axis]);
  if (imu_gyro_filter_enabled_) {
    for (int i = 0; i < 3; i++) {
      gyroFilteredDownsampled_[i] = pt1FilterApply(&imu_gyro_filter_[i], gyro_adcf_[i]);
    }
  } else {
    // 如果滤波器未启用，直接使用 gyro_adcf_
    std::memcpy(gyroFilteredDownsampled_, gyro_adcf_, sizeof(gyroFilteredDownsampled_));
  }

  // 步骤9: 更新动态陷波滤波器（在滤波链的末尾）
  // 位置：参考 Betaflight，dynNotchUpdate 在滤波链的最后调用
  // 这用于检测频率并更新滤波器参数
#ifdef USE_DYN_NOTCH_FILTER
  if (dyn_notch_enabled_ && dyn_notch_filter_ != nullptr && dyn_notch_filter_->isActive()) {
    dyn_notch_filter_->update();
  }
#endif

  // 重置降采样计数器（参考 gyro_filter_impl.c:86）
  if (!downsample_filter_enabled_) {
    sample_count_ = 0;
    std::memset(sample_sum_, 0, sizeof(sample_sum_));
  }

  publishGyroFiltered(imu_data);

  pushGyroDataToMlog(imu_data);
}

void gyro::applyFilterChain(const float input[3], float output[3]) {
  // 参考 gyro_filter_impl.c 的滤波顺序
  // 顺序：RPM滤波器 -> Notch1 -> Notch2 -> LPF1 -> 动态Notch
  float filtered[3];
  std::memcpy(filtered, input, sizeof(float) * 3);

  // 步骤3: RPM滤波器（可选）
  // 位置：gyro_filter_impl.c:48-50
  // TODO: 应用 RPM 滤波器（如果使能）
  // rpmFilter(gyroData, rpmData, filtered);

  // 步骤4: 静态陷波滤波器1
  // 位置：gyro_filter_impl.c:56
  if (notch1_enabled_) {
    for (int i = 0; i < 3; i++) {
      filtered[i] = notch1_apply_fn_((filter_t*)&notch1_filter_[i], filtered[i]);
    }
  }

  // 步骤5: 静态陷波滤波器2
  // 位置：gyro_filter_impl.c:57
  if (notch2_enabled_) {
    for (int i = 0; i < 3; i++) {
      filtered[i] = notch2_apply_fn_((filter_t*)&notch2_filter_[i], filtered[i]);
    }
  }

  // 步骤6: 第二个低通滤波器 - LPF1（静态或动态）
  // 位置：gyro_filter_impl.c:58
  // 使用：gyro.lowpassFilterApplyFn
  // 可以是静态或动态（根据油门调整）
  // 输出：gyro.gyroADCf[axis]
  if (lpf1_enabled_) {
    for (int i = 0; i < 3; i++) {
      filtered[i] = lpf1_apply_fn_((filter_t*)&lpf1_filter_[i], filtered[i]);
    }
  }

  // 步骤7: 动态陷波滤波器（可选）
  // 位置：gyro_filter_impl.c:63-78
  // 注意：在 Betaflight 中，dynNotchPush 和 dynNotchFilter 在滤波链中调用
  // 而 dynNotchUpdate 在滤波链的末尾调用（在 processImuData 中）
#ifdef USE_DYN_NOTCH_FILTER
  if (dyn_notch_enabled_ && dyn_notch_filter_ != nullptr && dyn_notch_filter_->isActive()) {
    // 推送样本用于频率分析（在滤波链中）
    for (int i = 0; i < 3; i++) {
      dyn_notch_filter_->push(i, filtered[i]);
    }
    
    // 应用动态 notch 滤波器（在滤波链中）
    for (int i = 0; i < 3; i++) {
      filtered[i] = dyn_notch_filter_->filter(i, filtered[i]);
    }
  }
#endif

  std::memcpy(output, filtered, sizeof(float) * 3);
}

#ifdef USE_DYN_LPF
// Dynamic throttle curve function (same as Betaflight)
static float dynThrottle(float throttle) {
  return throttle * (1.0f - (throttle * throttle) / 3.0f) * 1.5f;
}

// Dynamic LPF cutoff frequency calculation (same as Betaflight)
// Reference: ref/pid.c dynLpfCutoffFreq()
static float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
  const float expof = expo / 10.0f;
  const float curve = throttle * (1.0f - throttle) * expof + throttle;
  return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

// Dynamic LPF gyro update function (same as Betaflight)
void gyro::dynLpfGyroUpdate(float throttle) {
  if (dyn_lpf_filter_ != 0) {  // DYN_LPF_NONE = 0
    float cutoffFreq;
    if (dyn_lpf_curve_expo_ > 0) {
      cutoffFreq = dynLpfCutoffFreq(throttle, dyn_lpf_min_, dyn_lpf_max_, dyn_lpf_curve_expo_);
    } else {
      cutoffFreq = std::fmax(dynThrottle(throttle) * dyn_lpf_max_, dyn_lpf_min_);
    }
    
    const float gyroDt = target_looptime_us_ * 1e-6f;
    switch (dyn_lpf_filter_) {
      case 1:  // DYN_LPF_PT1
        for (int axis = 0; axis < 3; axis++) {
          pt1FilterUpdateCutoff(&lpf1_filter_[axis].pt1Filter, pt1FilterGain(cutoffFreq, gyroDt));
        }
        break;
      case 2:  // DYN_LPF_BIQUAD
        for (int axis = 0; axis < 3; axis++) {
          biquadFilterUpdateLPF(&lpf1_filter_[axis].biquadFilter, cutoffFreq, target_looptime_us_);
        }
        break;
      case 3:  // DYN_LPF_PT2
        for (int axis = 0; axis < 3; axis++) {
          pt2FilterUpdateCutoff(&lpf1_filter_[axis].pt2Filter, pt2FilterGain(cutoffFreq, gyroDt));
        }
        break;
      case 4:  // DYN_LPF_PT3
        for (int axis = 0; axis < 3; axis++) {
          pt3FilterUpdateCutoff(&lpf1_filter_[axis].pt3Filter, pt3FilterGain(cutoffFreq, gyroDt));
        }
        break;
    }
  }
}
#endif

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_GYRO_FILTER_EN
extern "C" {
static int gyro_filter_init_wrapper(void) {
  gyro& instance = gyro::instance();
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
