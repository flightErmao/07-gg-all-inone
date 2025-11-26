#ifndef GYRO_DNF_H__
#define GYRO_DNF_H__

#include <stdint.h>
#include <stdbool.h>
#include <rtconfig.h>

// Support both USE_DYN_NOTCH_FILTER and CONFIG_USE_DYN_NOTCH_FILTER
#if defined(USE_DYN_NOTCH_FILTER) || defined(CONFIG_USE_DYN_NOTCH_FILTER)

// Forward declarations for C types
extern "C" {
#include "filter.h"  // For biquadFilter_t, FILTER_NOTCH
}

// Include sdft.h - we need the complete sdft_t definition for the member array
// sdft.h already handles C++ compatibility internally with __cplusplus checks
#include "sdft.h"    // For sdft_t, SDFT_BIN_COUNT

#define DYN_NOTCH_COUNT_MAX 7
#define DYN_NOTCH_SMOOTH_HZ 4
#define DYN_NOTCH_CALC_TICKS (3 * 4)  // 3 axes and 4 steps per axis
#define DYN_NOTCH_UPDATE_MIN_HZ 2000

// 状态机步骤
typedef enum {
  STEP_WINDOW = 0,
  STEP_DETECT_PEAKS,
  STEP_CALC_FREQUENCIES,
  STEP_UPDATE_FILTERS,
  STEP_COUNT
} dynNotchStep_e;

// 峰值结构
typedef struct {
  int bin;
  float value;
} dynNotchPeak_t;

// 动态陷波滤波器类
class GyroDynNotch {
 public:
  GyroDynNotch();
  ~GyroDynNotch() = default;

  // 初始化动态陷波滤波器
  // config: 配置参数（从参数系统读取）
  // targetLooptimeUs: 目标循环时间（微秒）
  void init(uint16_t dyn_notch_q, uint16_t dyn_notch_min_hz, uint16_t dyn_notch_max_hz,
            uint8_t dyn_notch_count, uint32_t targetLooptimeUs);

  // 推送样本（在滤波链中调用）
  void push(int axis, float sample);

  // 更新动态陷波滤波器（在滤波链中调用）
  void update();

  // 应用动态陷波滤波器（在滤波链中调用）
  float filter(int axis, float value);

  // 检查是否激活
  bool isActive() const { return count_ > 0; }

  // 获取最大中心频率（用于调试）
  int getMaxCenterFreq() const { return max_center_freq_; }

  // 重置最大中心频率
  void resetMaxCenterFreq() { max_center_freq_ = 0; }

 private:
  // 处理状态机（内部函数）
  void process();

  // 动态陷波滤波器参数
  float q_;                    // Q 因子
  float min_hz_;               // 最小频率（Hz）
  float max_hz_;               // 最大频率（Hz）
  int count_;                   // 陷波滤波器数量
  uint32_t looptime_us_;        // 循环时间（微秒）
  int max_center_freq_;         // 最大中心频率（用于调试）

  // 每个轴的陷波滤波器
  float center_freq_[3][DYN_NOTCH_COUNT_MAX];  // 中心频率 [axis][notch]
  biquadFilter_t notch_[3][DYN_NOTCH_COUNT_MAX];  // 陷波滤波器 [axis][notch]

  // 样本累积和降采样
  int sample_index_;
  int sample_count_;
  float sample_count_rcp_;
  float sample_accumulator_[3];  // 样本累加器 [axis]
  float sample_avg_[3];          // 降采样后的平均值 [axis]

  // 状态机
  int state_tick_;
  dynNotchStep_e state_step_;
  int state_axis_;

  // SDFT 相关
  sdft_t sdft_[3];                    // SDFT 实例 [axis]
  float sdft_data_[SDFT_BIN_COUNT];   // SDFT 输出数据
  float sdft_sample_rate_hz_;         // SDFT 采样频率（Hz）
  float sdft_resolution_hz_;          // SDFT 分辨率（Hz/bin）
  int sdft_start_bin_;                // SDFT 起始 bin
  int sdft_end_bin_;                  // SDFT 结束 bin
  float sdft_noise_threshold_;        // SDFT 噪声阈值
  float pt1_looptime_s_;              // PT1 循环时间（秒）

  // 峰值检测
  dynNotchPeak_t peaks_[DYN_NOTCH_COUNT_MAX];
};

#endif  // USE_DYN_NOTCH_FILTER || CONFIG_USE_DYN_NOTCH_FILTER

#endif /* GYRO_DNF_H__ */

