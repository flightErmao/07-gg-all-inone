#ifndef ACC_CLASS_H__
#define ACC_CLASS_H__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include <rtconfig.h>  // For PROJECT_BF_ACC_THREAD_STACK_SIZE
#include "imu_mcn.h"
#include "acc_mcn.h"
#include "timestamp.h"
#include "../common/inc/init_sync.h"
#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

extern "C" {
#include "filter.h"  // For filter functions: pt2FilterInit, pt2FilterApply, etc.
#include "sensor_alignment.h"  // For sensor alignment
#include "vector.h"  // For matrix operations
}

// 加速度计校准类
class AccCalibration {
 public:
  AccCalibration();

  // 开始校准
  void startCalibration(float sample_rate_hz, uint32_t calibration_duration_ms);

  // 更新校准（每帧调用，传入原始加速度计数据）
  bool updateCalibration(const float acc_raw[3]);

  // 检查校准是否完成
  bool isCalibrationComplete() const { return calibration_complete_; }

  // 检查是否正在校准
  bool isCalibrating() const { return calibrating_; }

  // 获取零偏值
  void getAccZero(float acc_zero[3]) const;

  // 设置零偏值（用于从参数系统加载）
  void setAccZero(const float acc_zero[3]);

  // 应用零偏值到原始数据
  void applyZeroOffset(const float acc_raw[3], float acc_corrected[3]) const;

 private:
  bool calibration_complete_;
  bool calibrating_;
  uint32_t calibration_cycles_remaining_;
  uint32_t calibration_cycles_total_;
  float calibration_sum_[3];  // 每个轴的累加值
  float acc_zero_[3];          // 零偏值
};

// 加速度计处理类
class AccBf {
 public:
  // 单例模式：获取唯一实例
  static AccBf& instance();

  AccBf();
  ~AccBf();

  rt_err_t init();
  rt_err_t startThread();

  // MCN 相关函数
  rt_err_t initMcn();
  void cleanupMcnSubscriptions();
  void publishAccFiltered(const imu_raw_msg_t* imu_data);

  // 获取处理后的加速度数据
  void getFilteredAcc(float acc_filtered[3]) const {
    std::memcpy(acc_filtered, acc_filtered_, sizeof(acc_filtered_));
  }

  // 获取对齐后的加速度数据
  void getAccAdc(float acc_adc[3]) const {
    std::memcpy(acc_adc, acc_adc_, sizeof(acc_adc_));
  }

  // 校准命令接口
  rt_err_t startCalibration();

  // 获取校准零偏值（用于命令和参数保存）
  void getAccZero(float acc_zero[3]) const {
    acc_calibration_.getAccZero(acc_zero);
  }

  // 检查校准是否完成
  bool isCalibrationComplete() const {
    return acc_calibration_.isCalibrationComplete();
  }

 private:
  AccBf(const AccBf&) = delete;
  AccBf& operator=(const AccBf&) = delete;

  // 线程入口函数（静态）
  static void threadEntry(void* parameter);

  // 线程主循环
  void threadLoop();

  // 处理 IMU 数据
  void processAccData(const imu_raw_msg_t* imu_data);

  // 初始化滤波器
  void initFilters();

  // 应用处理链：对齐 → 校准 → Trim → PT2滤波
  void applyProcessingChain(const float acc_raw[3], float acc_output[3]);

  // ========== 成员变量 ==========

  // 加速度数据
  float acc_adc_[3];       // 对齐、校准但未Trim和滤波的数据
  float acc_filtered_[3];  // 处理后的加速度数据（对齐 → 校准 → Trim → PT2滤波）

  // Trim 值（用于姿态估计时的角度修正）
  float acc_trim_[3];  // [roll, pitch, yaw] trim 值（度）

  // 采样频率
  uint16_t sample_rate_hz_;

  // PT2 滤波器（用于加速度计滤波）
  pt2Filter_t pt2_filter_[3];  // PT2 filter for each axis [X, Y, Z]
  bool pt2_filter_enabled_;    // PT2 filter enabled flag

  // 传感器对齐
  sensor_align_e acc_align_;   // Sensor alignment enum
  sensorAlignment_t acc_align_rpy_;  // Sensor alignment angles (in deciderees)
  matrix33_t rotation_matrix_;  // 自定义旋转矩阵（如果使用）
  bool use_custom_matrix_;

  // 加速度计校准
  AccCalibration acc_calibration_;
  bool calibration_started_;

  // 线程相关
  rt_thread_t thread_;
  struct rt_thread thread_obj_;
  rt_uint8_t thread_stack_[PROJECT_BF_ACC_THREAD_STACK_SIZE];
  bool thread_inited_;

  // MCN 订阅和发布相关
  rt_sem_t imu_event_;        // MCN 事件信号量（用于 mcn_poll_sync）
  McnNode_t imu_node_;        // MCN 订阅节点（imu_raw）
  McnHub_t acc_filtered_hub_;  // MCN 发布 hub（acc_filtered）

  // 初始化完成标志
  bool initialized_;

  // 在线程入口中执行的初始化（依赖其他线程状态）
  void initInThreadEntry();
};

#endif /* ACC_CLASS_H__ */

