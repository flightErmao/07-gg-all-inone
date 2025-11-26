#ifndef GYRO_FILTER_H__
#define GYRO_FILTER_H__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include "imu_mcn.h"
#include "gyro_mcn.h"
#include "filter.h"  // For filter functions: pt1FilterInit, pt2FilterInit, pt3FilterInit, biquadFilterInit, etc.
}

extern "C" {
#include "sensor_alignment.h"  // For sensor alignment
#include "vector.h"            // For matrix operations
}
#include "../log/inc/mlog_gyro.hpp"

// 简化的标准差统计结构（用于校准运动检测）
class DeviationStats {
 public:
  DeviationStats() : sum_(0.0f), sum_sq_(0.0f), count_(0) {}

  void clear() {
    sum_ = 0.0f;
    sum_sq_ = 0.0f;
    count_ = 0;
  }

  void push(float value) {
    sum_ += value;
    sum_sq_ += value * value;
    count_++;
  }

  float standardDeviation() const {
    if (count_ < 2) {
      return 0.0f;
    }
    float mean = sum_ / count_;
    float variance = (sum_sq_ / count_) - (mean * mean);
    return (variance > 0.0f) ? std::sqrt(variance) : 0.0f;
  }

 private:
  float sum_;
  float sum_sq_;
  uint32_t count_;
};

// 陀螺仪校准类（从 gyro.c 提取的校准逻辑）
class GyroCalibration {
 public:
  GyroCalibration();

  // 开始校准
  void startCalibration(float sample_rate_hz, uint32_t calibration_duration_ms, float movement_threshold,
                        int16_t yaw_offset_centidegrees);

  // 更新校准（每帧调用，传入原始陀螺仪数据）
  bool updateCalibration(const float gyro_raw[3]);

  // 检查校准是否完成
  bool isCalibrationComplete() const { return calibration_complete_; }

  // 检查是否正在校准
  bool isCalibrating() const { return calibrating_; }

  // 获取零偏值
  void getGyroZero(float gyro_zero[3]) const;

  // 设置零偏值（用于从参数系统加载）
  void setGyroZero(const float gyro_zero[3]);

  // 应用零偏值到原始数据
  void applyZeroOffset(const float gyro_raw[3], float gyro_corrected[3]) const;

 private:
  bool calibration_complete_;
  bool calibrating_;
  uint32_t calibration_cycles_remaining_;
  uint32_t calibration_cycles_total_;
  float calibration_sum_[3];           // 每个轴的累加值
  DeviationStats calibration_var_[3];  // 每个轴的标准差统计
  float gyro_zero_[3];                 // 零偏值
  float movement_threshold_;
  int16_t yaw_offset_centidegrees_;

  bool isOnFirstCycle() const;
  bool isOnFinalCycle() const;
};

// Forward declaration for dynamic notch filter (defined in gyro_dnf.h)
#ifdef USE_DYN_NOTCH_FILTER
class GyroDynNotch;
#endif

// gyro 是 gyro_t 的 C++ 版本
// 简单变量直接映射为成员变量，复杂成员变量对象化
class gyro {
 public:
  // 单例模式：获取唯一实例
  static gyro& instance();

  gyro();

  rt_err_t init();

  // MCN 相关函数
  rt_err_t initMcn();
  void cleanupMcnSubscriptions();
  void publishGyroFiltered(const imu_raw_msg_t* imu_data);
  
  // MCN 订阅和发布封装函数
  rt_err_t subscribeImu();
  rt_err_t advertiseGyroFiltered();

  // Mlog 相关函数
  rt_err_t initMlog();
  void pushGyroDataToMlog(const imu_raw_msg_t* imu_data);

  // 获取滤波后的角速度数据（对应 gyro.gyroADCf）
  void getFilteredGyro(float gyro_filtered[3]) const { std::memcpy(gyro_filtered, gyro_adcf_, sizeof(gyro_adcf_)); }

  // 获取对齐后的角速度数据（对应 gyro.gyroADC）
  void getGyroAdc(float gyro_adc[3]) const { std::memcpy(gyro_adc, gyro_adc_, sizeof(gyro_adc_)); }

#ifdef USE_DYN_LPF
  // 动态 LPF 更新函数
  void dynLpfGyroUpdate(float throttle);
#endif

  // 线程入口函数（静态）
  static void threadEntry(void* parameter);

 private:
  gyro(const gyro&) = delete;
  gyro& operator=(const gyro&) = delete;

  // 线程主循环
  void threadLoop();

  // 处理 IMU 数据（原 handleWork 的逻辑）
  void processImuData(const imu_raw_msg_t* imu_data);

  // ========== 从 gyro_t 映射的简单变量 ==========

  // 采样频率和时间（对应 gyro.sampleRateHz, targetLooptime, sampleLooptime）
  uint16_t sample_rate_hz_;      // gyro.sampleRateHz
  uint32_t target_looptime_us_;  // gyro.targetLooptime (微秒)
  uint32_t sample_looptime_us_;  // gyro.sampleLooptime (微秒)
  // 注意：scale_ 已移除，因为 mcn 发布的数据已经是缩放后的（在 accgyro_spi_bmi270.cpp 中已应用 GYRO_SCALE_2000DPS）

  // 角速度数据（对应 gyro.gyroADC, gyro.gyroADCf）
  float gyro_adc_[3];   // gyro.gyroADC[XYZ_AXIS_COUNT] - 对齐、校准但未滤波的数据（已缩放）
  float gyro_adcf_[3];  // gyro.gyroADCf[XYZ_AXIS_COUNT] - 滤波后的角速度数据

  // 降采样相关（对应 gyro.sampleCount, sampleSum, downsampleFilterEnabled）
  uint8_t sample_count_;            // gyro.sampleCount - 陀螺仪采样计数器
  float sample_sum_[3];             // gyro.sampleSum[XYZ_AXIS_COUNT] - 用于降采样的累加样本
  bool downsample_filter_enabled_;  // gyro.downsampleFilterEnabled - 是否使用 LPF2 降采样


#ifdef USE_DYN_LPF
  // 动态 LPF1 参数（对应 gyro.dynLpfFilter, dynLpfMin, dynLpfMax, dynLpfCurveExpo）
  uint8_t dyn_lpf_filter_;      // gyro.dynLpfFilter
  uint16_t dyn_lpf_min_;        // gyro.dynLpfMin
  uint16_t dyn_lpf_max_;        // gyro.dynLpfMax
  uint8_t dyn_lpf_curve_expo_;  // gyro.dynLpfCurveExpo
#endif


  // ========== 复杂成员变量对象化 ==========

  // 低通滤波器（对应 gyro.lowpassFilter, lowpass2Filter）
  // 使用 union 结构，类似 pid 中的 dtermLowpass_t
  typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
    pt2Filter_t pt2Filter;
    pt3Filter_t pt3Filter;
  } gyroLowpassFilter_t;

  // LPF1 滤波器（对应 gyro.lowpassFilter[XYZ_AXIS_COUNT]）
  filterApplyFnPtr lpf1_apply_fn_;      // LPF1 filter apply function pointer
  gyroLowpassFilter_t lpf1_filter_[3];  // LPF1 filter state [X, Y, Z]
  uint8_t lpf1_type_;                   // LPF1 filter type (FILTER_PT1, FILTER_BIQUAD, FILTER_PT2, FILTER_PT3)
  bool lpf1_enabled_;                   // LPF1 filter enabled flag

  // LPF2 滤波器（对应 gyro.lowpass2Filter[XYZ_AXIS_COUNT]）
  filterApplyFnPtr lpf2_apply_fn_;      // LPF2 filter apply function pointer
  gyroLowpassFilter_t lpf2_filter_[3];  // LPF2 filter state [X, Y, Z]
  uint8_t lpf2_type_;                   // LPF2 filter type (FILTER_PT1, FILTER_BIQUAD, FILTER_PT2, FILTER_PT3)
  bool lpf2_enabled_;                   // LPF2 filter enabled flag

  // Notch 滤波器（对应 gyro.notchFilter1, notchFilter2）
  // 原本：biquadFilter_t notchFilter1[XYZ_AXIS_COUNT] + filterApplyFnPtr
  filterApplyFnPtr notch1_apply_fn_;  // Notch1 filter apply function pointer
  biquadFilter_t notch1_filter_[3];   // Notch1 filter state [X, Y, Z]
  bool notch1_enabled_;               // Notch1 filter enabled flag

  filterApplyFnPtr notch2_apply_fn_;  // Notch2 filter apply function pointer
  biquadFilter_t notch2_filter_[3];   // Notch2 filter state [X, Y, Z]
  bool notch2_enabled_;               // Notch2 filter enabled flag

#ifdef USE_DYN_NOTCH_FILTER
  // 动态 notch 滤波器（对应 dynNotchInit/dynNotchFilter 函数）
  // Forward declaration is at file scope (above class definition)
  GyroDynNotch* dyn_notch_filter_;  // Dynamic notch filter instance (pointer to avoid include in header)
  bool dyn_notch_enabled_;           // Dynamic notch filter enabled flag
#endif

  // 第三个低通滤波器：PT1（用于姿态估计）
  // 对应 gyro.imuGyroFilter[XYZ_AXIS_COUNT]
  // 对 gyro_adcf_ 进行 PT1 滤波，输出给 attitude 使用
  // 截止频率：200 Hz (GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ)
  pt1Filter_t imu_gyro_filter_[3];    // gyro.imuGyroFilter[XYZ_AXIS_COUNT]
  bool imu_gyro_filter_enabled_;      // IMU gyro filter enabled flag
  float gyroFilteredDownsampled_[3];  // 输出：gyroFilteredDownsampled[axis]（给 attitude 使用）

  // ========== 其他辅助成员 ==========

  // 传感器对齐
  sensor_align_e gyro_align_;         // Sensor alignment enum
  sensorAlignment_t gyro_align_rpy_;  // Sensor alignment angles (in deciderees)
  matrix33_t rotation_matrix_;        // 自定义旋转矩阵（如果使用）
  bool use_custom_matrix_;

  // Gyro calibration（对应 gyro.gyroSensor[].calibration）
  GyroCalibration gyro_calibration_;
  bool calibration_started_;

  // 线程相关
  rt_thread_t thread_;
  struct rt_thread thread_obj_;
  rt_uint8_t thread_stack_[4096];  // 最大栈大小，实际使用大小由 Kconfig 配置
  bool thread_inited_;

  // MCN 订阅和发布相关
  rt_sem_t imu_event_;          // MCN 事件信号量（用于 mcn_poll_sync）
  McnNode_t imu_node_;          // MCN 订阅节点（imu_raw）
  McnHub_t gyro_filtered_hub_;  // MCN 发布 hub（gyro_filtered）

  // ========== 内部方法 ==========

  // 初始化低通滤波器（参考 gyro_init.c:127-197）
  // slot: FILTER_LPF1 (0) 或 FILTER_LPF2 (1)
  // type: FILTER_PT1, FILTER_BIQUAD, FILTER_PT2, FILTER_PT3
  // lpfHz: 截止频率（Hz）
  // looptime: 循环时间（微秒）
  // 返回: true 如果初始化成功，false 否则
  bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime);

  // 初始化所有滤波器（参考 gyro_init.c:229-266）
  void gyroInitFilters();

  // 应用完整的滤波链（参考 gyro_filter_impl.c）
  void applyFilterChain(const float input[3], float output[3]);

  // 初始化降采样滤波器（参考 gyroInitFilters 中的 imuGyroFilter 初始化）
  void initDownsampleFilter();

  // 设置目标循环时间（参考 gyro.c:750-759 中的 gyroSetTargetLooptime）
  void setTargetLooptime(uint8_t pid_denom);

  // 在线程入口中执行的初始化（依赖其他线程状态）
  void initInThreadEntry();
};

#endif /* GYRO_FILTER_H__ */
