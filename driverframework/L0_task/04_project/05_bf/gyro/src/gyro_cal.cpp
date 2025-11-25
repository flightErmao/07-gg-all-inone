#include "gyro_class.h"

#include <cstring>

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


