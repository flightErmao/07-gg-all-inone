#include "acc_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "acc_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "timestamp.h"
#include "param.h"
#include "../common/inc/init_sync.h"
#include "boardalignment.h"  // For sensor alignment functions and initBoardAlignment
#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

#include "../imu/inc/bmi270_class.h"

#include <cstring>
#include <cmath>

namespace {
constexpr float GRAVITY = 9.80665f;  // m/s^2
}  // namespace

// AccCalibration 实现（Betaflight 风格）
AccCalibration::AccCalibration()
    : calibration_complete_(false),
      calibrating_(false),
      calibration_cycles_remaining_(0),
      calibration_cycles_total_(0) {
  std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
  std::memset(acc_trim_, 0, sizeof(acc_trim_));
}

void AccCalibration::startCalibration(float sample_rate_hz, uint32_t calibration_duration_ms) {
  calibration_complete_ = false;
  calibrating_ = true;

  // 计算校准周期数：(duration_ms * sample_rate_hz) / 1000
  calibration_cycles_total_ = static_cast<uint32_t>((calibration_duration_ms * sample_rate_hz) / 1000.0f);
  if (calibration_cycles_total_ < 1) {
    calibration_cycles_total_ = 1;
  }
  calibration_cycles_remaining_ = calibration_cycles_total_;

  // Betaflight 风格：使用整数累加
  std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
  std::memset(acc_trim_, 0, sizeof(acc_trim_));
}

bool AccCalibration::updateCalibration(const float acc_adc[3]) {
  // Betaflight 风格：校准使用对齐（旋转）后的 ADC 数据
  // acc_adc 应该是已经经过对齐处理的原始 ADC 值
  if (!calibrating_ || calibration_complete_) {
    return false;
  }

  // Betaflight 风格：在第一个周期重置累加值
  if (isOnFirstCalibrationCycle()) {
    for (int axis = 0; axis < 3; axis++) {
      calibration_sum_[axis] = 0;
    }
  }

  // Betaflight 风格：使用整数累加（将 float 转换为 int32_t）
  for (int axis = 0; axis < 3; axis++) {
    calibration_sum_[axis] += static_cast<int32_t>(acc_adc[axis]);
  }

  calibration_cycles_remaining_--;

  // Betaflight 风格：在最后一个周期计算 trim 值
  if (isOnFinalCalibrationCycle()) {
    // 计算平均值（使用四舍五入）：(sum + cycles/2) / cycles
    // X 和 Y 轴：直接计算平均值
    acc_trim_[0] = static_cast<float>(calibration_sum_[0] + static_cast<int32_t>(calibration_cycles_total_ / 2)) / 
                   static_cast<float>(calibration_cycles_total_);
    acc_trim_[1] = static_cast<float>(calibration_sum_[1] + static_cast<int32_t>(calibration_cycles_total_ / 2)) / 
                   static_cast<float>(calibration_cycles_total_);
    
    // Z 轴：平均值减去 1G（Betaflight 风格）
    acc_trim_[2] = static_cast<float>(calibration_sum_[2] + static_cast<int32_t>(calibration_cycles_total_ / 2)) / 
                   static_cast<float>(calibration_cycles_total_) - ACC_1G_ADC;

    calibration_complete_ = true;
    calibrating_ = false;
    return true;
  }

  return false;
}

void AccCalibration::getAccTrim(float acc_trim[3]) const {
  std::memcpy(acc_trim, acc_trim_, sizeof(acc_trim_));
}

void AccCalibration::setAccTrim(const float acc_trim[3]) {
  std::memcpy(acc_trim_, acc_trim, sizeof(acc_trim_));
  calibration_complete_ = true;
  calibrating_ = false;
}

void AccCalibration::applyTrim(const float acc_adc[3], float acc_corrected[3]) const {
  // Betaflight 风格：trim 值应用于对齐（旋转）后的 ADC 数据
  // acc_adc 应该是已经经过对齐处理的原始 ADC 值
  // acc_trim_ 是在对齐后坐标系中记录的 trim 值（X/Y 是平均值，Z 是平均值减去 1G）
  for (int axis = 0; axis < 3; axis++) {
    acc_corrected[axis] = acc_adc[axis] - acc_trim_[axis];
  }
}

// AccBf 单例实现
AccBf& AccBf::instance() {
  static AccBf instance_obj;
  return instance_obj;
}

AccBf::AccBf()
    : sample_rate_hz_(0),
      pt2_filter_enabled_(false),
      acc_align_(ALIGN_DEFAULT),
      use_custom_matrix_(false),
      calibration_started_(false),
      thread_(RT_NULL),
      thread_inited_(false),
      imu_event_(RT_NULL),
      imu_node_(RT_NULL),
      acc_filtered_hub_(nullptr),
      initialized_(false) {
  std::memset(&thread_obj_, 0, sizeof(thread_obj_));
  std::memset(thread_stack_, 0, sizeof(thread_stack_));
  std::memset(acc_adc_, 0, sizeof(acc_adc_));
  std::memset(acc_filtered_, 0, sizeof(acc_filtered_));
  std::memset(acc_trim_, 0, sizeof(acc_trim_));
  std::memset(&rotation_matrix_, 0, sizeof(rotation_matrix_));
}

AccBf::~AccBf() {
  cleanupMcnSubscriptions();
}

rt_err_t AccBf::init() {
  if (thread_inited_) {
    LOG_W("AccBf already initialized");
    return RT_EOK;
  }

  // 初始化 MCN（订阅和发布）
  rt_err_t ret = initMcn();
  if (ret != RT_EOK) {
    return ret;
  }

  // 注意：滤波器初始化已移到initInThreadEntry中，因为需要等待IMU初始化完成

  // 创建线程
  rt_err_t err = rt_thread_init(&thread_obj_, "acc", threadEntry, this, thread_stack_,
                                PROJECT_BF_ACC_THREAD_STACK_SIZE, PROJECT_BF_ACC_THREAD_PRIORITY,
                                PROJECT_BF_ACC_THREAD_TIMESLICE);

  if (err != RT_EOK) {
    LOG_E("AccBf thread init failed: %d", err);
    cleanupMcnSubscriptions();
    return err;
  }

  thread_ = &thread_obj_;
  thread_inited_ = true;

  LOG_I("AccBf initialized");
  return RT_EOK;
}

rt_err_t AccBf::startThread() {
  if (!thread_inited_) {
    LOG_E("AccBf not initialized");
    return -RT_ERROR;
  }

  // 启动线程
  rt_err_t ret = rt_thread_startup(thread_);
  if (ret != RT_EOK) {
    LOG_E("AccBf thread startup failed: %d", ret);
    return ret;
  }

  LOG_I("AccBf thread started");
  return RT_EOK;
}

void AccBf::setAlignment(sensor_align_e align, const sensorAlignment_t* customAlignment) {
  acc_align_ = align;
  
  if (align == ALIGN_CUSTOM && customAlignment != nullptr) {
    // 使用自定义对齐：读取旋转角度，初始化旋转矩阵
    use_custom_matrix_ = true;
    acc_align_rpy_ = *customAlignment;
    buildRotationMatrixFromAngles(&rotation_matrix_, &acc_align_rpy_);
    LOG_I("Acc alignment set to ALIGN_CUSTOM: roll=%d, pitch=%d, yaw=%d (decidegrees)",
          acc_align_rpy_.roll, acc_align_rpy_.pitch, acc_align_rpy_.yaw);
  } else if (align != ALIGN_DEFAULT && align != ALIGN_CUSTOM) {
    // 使用标准对齐：不初始化旋转矩阵，直接使用 alignSensorViaRotation
    use_custom_matrix_ = false;
    // 不构建旋转矩阵，只保存对齐类型
    LOG_I("Acc alignment set to standard: %d (will use alignSensorViaRotation)", align);
  } else {
    // ALIGN_DEFAULT 或无效值，使用默认（无旋转）
    use_custom_matrix_ = false;
    std::memset(&acc_align_rpy_, 0, sizeof(acc_align_rpy_));
    std::memset(&rotation_matrix_, 0, sizeof(rotation_matrix_));
    rotation_matrix_.m[0][0] = 1.0f;
    rotation_matrix_.m[1][1] = 1.0f;
    rotation_matrix_.m[2][2] = 1.0f;
    LOG_I("Acc alignment set to ALIGN_DEFAULT");
  }
}

void AccBf::threadEntry(void* parameter) {
  auto* self = static_cast<AccBf*>(parameter);
  if (!self) {
    return;
  }

  // 在线程调度器启动后，执行依赖其他线程状态的初始化
  self->initInThreadEntry();

  // 通知 Acc 初始化完成
  initSyncNotify(INIT_SYNC_ACC);

  // 进入主循环
  self->threadLoop();
}

void AccBf::initInThreadEntry() {
  // 等待 IMU 模块初始化完成
  rt_err_t ret = initSyncWait(INIT_SYNC_BMI270, 1000);  // 等待最多1秒
  if (ret != RT_EOK) {
    LOG_E("AccBf: Failed to wait for BMI270 initialization: %d", ret);
    return;
  }

  // 从 IMU 模块获取加速度计采样频率
  using namespace bf_bmi270;
  sample_rate_hz_ = static_cast<uint16_t>(BMI270::instance().getAccSampleRateHz());
  LOG_I("AccBf: Got acc sample rate from IMU: %u Hz", sample_rate_hz_);

  // 初始化滤波器（使用从IMU获取的采样频率）
  initFilters();

  // 从参数系统加载板级对齐参数并初始化（Betaflight 风格）
  boardAlignment_t imu_board_alignment = {0, 0, 0};
  if (getParam("imu_board_alignment", &imu_board_alignment, sizeof(imu_board_alignment)) == RT_EOK) {
    initBoardAlignment(&imu_board_alignment);
    LOG_I("Loaded board alignment: roll=%d, pitch=%d, yaw=%d (degrees)", (int)imu_board_alignment.rollDegrees,
          (int)imu_board_alignment.pitchDegrees, (int)imu_board_alignment.yawDegrees);
  } else {
    // 参数不存在，使用默认值（无板级对齐）
    // initBoardAlignment(&imu_board_alignment);
    LOG_I("Board alignment parameter not found, using default (0, 0, 0)");
  }

  // 从参数系统加载校准值（如果存在）
  // Betaflight 风格：trim 值单位是原始 ADC 值（X/Y 是平均值，Z 是平均值减去 1G）
  float acc_trim[3] = {0.0f, 0.0f, 0.0f};
  if (getParam("cali_imu_acc_offset", acc_trim, sizeof(acc_trim)) == RT_EOK) {
    acc_calibration_.setAccTrim(acc_trim);
    LOG_I("Loaded acc trim from params (ADC units): %.3f, %.3f, %.3f", acc_trim[0], acc_trim[1], acc_trim[2]);
  }

  // 加载 Trim 值
  if (getParam("cali_acc_trim_roll", &acc_trim_[0], sizeof(acc_trim_[0])) == RT_EOK &&
      getParam("cali_acc_trim_pitch", &acc_trim_[1], sizeof(acc_trim_[1])) == RT_EOK) {
    LOG_I("Loaded acc trim: roll=%.2f, pitch=%.2f", acc_trim_[0], acc_trim_[1]);
  }

  // 加载对齐参数（从 acc_init.cpp 中调用）
  loadAlignmentFromParams();

  LOG_I("AccBf thread initialization complete");
}

void AccBf::threadLoop() {
  LOG_I("AccBf thread loop started");

  while (true) {
#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG0_HIGH();  // Debug pin: Acc task execution start
#endif

    // 阻塞等待 acc_raw 数据
    if (mcn_poll_sync(imu_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      acc_raw_msg_t acc_data;
      if (mcn_copy(MCN_HUB(acc_raw), imu_node_, &acc_data) == RT_EOK) {
        // 处理加速度计数据
        processAccData(&acc_data);

        // 发布处理后的数据
        publishAccFiltered(&acc_data);
      }
    }

#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG0_LOW();  // Debug pin: Acc task execution end
#endif
  }
}

void AccBf::processAccData(const acc_raw_msg_t* acc_data) {
  if (acc_data == nullptr) {
    return;
  }

  // 处理流程：原始ADC → 对齐 → 校准 → Trim → PT2滤波
  // Betaflight 风格：acc 数据保持原始 ADC 值（未缩放），缩放将在 attitude 更新中进行
  float acc_processed[3];
  std::memcpy(acc_processed, acc_data->accel, sizeof(acc_processed));

  // 步骤1: 传感器对齐
  // 参考 Betaflight: 对齐在初始化时设置，处理时直接应用
  vector3_t vec_in = {acc_processed[0], acc_processed[1], acc_processed[2]};
  vector3_t vec_aligned;
  
  if (acc_align_ == ALIGN_DEFAULT) {
    // ALIGN_DEFAULT，无需对齐
    vec_aligned = vec_in;
  } else if (use_custom_matrix_) {
    // 使用自定义旋转矩阵（ALIGN_CUSTOM）
    matrixVectorMul(&vec_aligned, &rotation_matrix_, &vec_in);
  } else {
    // 使用标准对齐方式（CW0_DEG, CW90_DEG 等）：直接使用 alignSensorViaRotation
    vec_aligned = vec_in;
    alignSensorViaRotation(&vec_aligned, acc_align_);
  }
  
  acc_processed[0] = vec_aligned.x;
  acc_processed[1] = vec_aligned.y;
  acc_processed[2] = vec_aligned.z;

  // 步骤2: 应用校准（trim 校正，Betaflight 风格）
  // Betaflight 风格：校准在对齐（旋转）之后进行
  // - 校准过程：原始 ADC → 对齐（旋转）→ 记录 trim 值（在对齐后的坐标系中，Z 轴减去 1G）
  // - 应用过程：原始 ADC → 对齐（旋转）→ 去除 trim 值（使用对齐后坐标系的 trim 值）
  // 如果正在校准，更新校准状态（使用对齐后的 ADC 数据）
  if (calibration_started_ && acc_calibration_.isCalibrating()) {
    acc_calibration_.updateCalibration(acc_processed);
  }
  
  // 如果校准完成，应用 trim 校正（使用对齐后坐标系的 trim 值）
  if (acc_calibration_.isCalibrationComplete()) {
    acc_calibration_.applyTrim(acc_processed, acc_processed);
  }

  // 存储对齐和校准后的数据（未Trim和滤波）
  std::memcpy(acc_adc_, acc_processed, sizeof(acc_adc_));

  // 步骤3: 应用 Trim（用于姿态估计时的角度修正）
  // Trim 值在姿态估计中使用，这里先不应用，因为 Trim 主要用于角度计算

  // 步骤4: 应用 PT2 滤波
  applyProcessingChain(acc_processed, acc_filtered_);

  initialized_ = true;
}

void AccBf::applyProcessingChain(const float acc_input[3], float acc_output[3]) {
  // 应用 PT2 滤波器
  if (pt2_filter_enabled_) {
    for (int i = 0; i < 3; i++) {
      acc_output[i] = pt2FilterApply(&pt2_filter_[i], acc_input[i]);
    }
  } else {
    std::memcpy(acc_output, acc_input, sizeof(float) * 3);
  }
}

void AccBf::initFilters() {
  // 从参数系统读取 PT2 滤波器配置（默认值 50Hz）
  uint16_t acc_filter_cutoff_hz = 50;  // 默认值 50Hz
  if (getParam("filter_acc_cutoff_hz", &acc_filter_cutoff_hz, sizeof(acc_filter_cutoff_hz)) != RT_EOK) {
    acc_filter_cutoff_hz = 50;  // 如果参数不存在，使用默认值 50Hz
  }

  // 使用从IMU获取的采样频率（已在initInThreadEntry中设置）
  if (sample_rate_hz_ == 0) {
    LOG_W("Acc sample rate not set, using default 800Hz");
    sample_rate_hz_ = 800;  // 默认800Hz
  }

  if (acc_filter_cutoff_hz > 0 && sample_rate_hz_ > 0) {
    float dT = 1.0f / static_cast<float>(sample_rate_hz_);
    const float k = pt2FilterGain(static_cast<float>(acc_filter_cutoff_hz), dT);
    for (int i = 0; i < 3; i++) {
      pt2FilterInit(&pt2_filter_[i], k);
    }
    pt2_filter_enabled_ = true;
    LOG_I("Acc PT2 filter initialized: cutoff=%u Hz, sample_rate=%u Hz", acc_filter_cutoff_hz, sample_rate_hz_);
  }
}

rt_err_t AccBf::startCalibration() {
  if (calibration_started_) {
    LOG_W("Acc calibration already started");
    return RT_EOK;
  }

  // 使用从IMU获取的采样频率
  if (sample_rate_hz_ == 0) {
    LOG_E("Acc sample rate not set, cannot start calibration");
    return -RT_ERROR;
  }

  float sample_rate_hz = static_cast<float>(sample_rate_hz_);
  uint32_t duration_ms = PROJECT_BF_ACC_CALIBRATION_DURATION_MS;

  acc_calibration_.startCalibration(sample_rate_hz, duration_ms);
  calibration_started_ = true;

  LOG_I("Acc calibration started: duration=%u ms, sample_rate=%.1f Hz", duration_ms, sample_rate_hz);
  return RT_EOK;
}

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_ACC_EN
extern "C" {
static int acc_main_init_wrapper(void) {
  // 小延迟确保 IMU 先初始化
  rt_thread_mdelay(10);

  AccBf& acc = AccBf::instance();
  rt_err_t ret = acc.init();
  if (ret != RT_EOK) {
    LOG_E("AccBf init failed: %d", ret);
    return (int)ret;
  }
  LOG_I("AccBf initialized successfully");

  ret = acc.startThread();
  if (ret == RT_EOK) {
    LOG_I("AccBf auto-init success");
  } else {
    LOG_E("AccBf auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(acc_main_init_wrapper);
}
#endif

