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
#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

#include <cstring>
#include <cmath>

namespace {
constexpr float GRAVITY = 9.80665f;  // m/s^2
}  // namespace

// AccCalibration 实现
AccCalibration::AccCalibration()
    : calibration_complete_(false),
      calibrating_(false),
      calibration_cycles_remaining_(0),
      calibration_cycles_total_(0) {
  std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
  std::memset(acc_zero_, 0, sizeof(acc_zero_));
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

  std::memset(calibration_sum_, 0, sizeof(calibration_sum_));
  std::memset(acc_zero_, 0, sizeof(acc_zero_));
}

bool AccCalibration::updateCalibration(const float acc_raw[3]) {
  if (!calibrating_ || calibration_complete_) {
    return false;
  }

  // 累加数据
  for (int axis = 0; axis < 3; axis++) {
    calibration_sum_[axis] += acc_raw[axis];
  }

  calibration_cycles_remaining_--;

  // 检查是否完成
  if (calibration_cycles_remaining_ == 0) {
    // 计算平均值作为零偏值
    for (int axis = 0; axis < 3; axis++) {
      acc_zero_[axis] = calibration_sum_[axis] / calibration_cycles_total_;
    }

    calibration_complete_ = true;
    calibrating_ = false;
    return true;
  }

  return false;
}

void AccCalibration::getAccZero(float acc_zero[3]) const {
  std::memcpy(acc_zero, acc_zero_, sizeof(acc_zero_));
}

void AccCalibration::setAccZero(const float acc_zero[3]) {
  std::memcpy(acc_zero_, acc_zero, sizeof(acc_zero_));
  calibration_complete_ = true;
  calibrating_ = false;
}

void AccCalibration::applyZeroOffset(const float acc_raw[3], float acc_corrected[3]) const {
  for (int axis = 0; axis < 3; axis++) {
    acc_corrected[axis] = acc_raw[axis] - acc_zero_[axis];
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

  // 初始化滤波器（从参数系统读取配置）
  initFilters();

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
  // 从参数系统加载校准值（如果存在）
  float acc_zero[3] = {0.0f, 0.0f, 0.0f};
  if (getParam("acc_zero_x", &acc_zero[0], sizeof(acc_zero[0])) == RT_EOK &&
      getParam("acc_zero_y", &acc_zero[1], sizeof(acc_zero[1])) == RT_EOK &&
      getParam("acc_zero_z", &acc_zero[2], sizeof(acc_zero[2])) == RT_EOK) {
    acc_calibration_.setAccZero(acc_zero);
    LOG_I("Loaded acc zero from params: %.3f, %.3f, %.3f", acc_zero[0], acc_zero[1], acc_zero[2]);
  }

  // 加载 Trim 值
  if (getParam("acc_trim_roll", &acc_trim_[0], sizeof(acc_trim_[0])) == RT_EOK &&
      getParam("acc_trim_pitch", &acc_trim_[1], sizeof(acc_trim_[1])) == RT_EOK) {
    LOG_I("Loaded acc trim: roll=%.2f, pitch=%.2f", acc_trim_[0], acc_trim_[1]);
  }

  LOG_I("AccBf thread initialization complete");
}

void AccBf::threadLoop() {
  LOG_I("AccBf thread loop started");

  while (true) {
#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG4_HIGH();  // Debug pin: Acc task execution start
#endif

    // 阻塞等待 IMU 数据
    if (mcn_poll_sync(imu_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      imu_raw_msg_t imu_data;
      if (mcn_copy(MCN_HUB(imu), imu_node_, &imu_data) == RT_EOK) {
        // 处理加速度计数据
        processAccData(&imu_data);

        // 发布处理后的数据
        publishAccFiltered(&imu_data);
      }
    }

#ifdef PROJECT_BF_ACC_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG4_LOW();  // Debug pin: Acc task execution end
#endif
  }
}

void AccBf::processAccData(const imu_raw_msg_t* imu_data) {
  if (imu_data == nullptr) {
    return;
  }

  // 处理流程：原始ADC → 对齐 → 校准 → Trim → PT2滤波
  float acc_processed[3];
  std::memcpy(acc_processed, imu_data->accel, sizeof(acc_processed));

  // 步骤1: 传感器对齐
  if (use_custom_matrix_) {
    // 使用自定义旋转矩阵
    vector3_t vec_in = {acc_processed[0], acc_processed[1], acc_processed[2]};
    vector3_t vec_out;
    matrixVectorMul(&vec_out, &rotation_matrix_, &vec_in);
    acc_processed[0] = vec_out.x;
    acc_processed[1] = vec_out.y;
    acc_processed[2] = vec_out.z;
  } else {
    // 使用标准对齐方式
    buildAlignmentFromStandardAlignment(&acc_align_rpy_, acc_align_);
    buildRotationMatrixFromAngles(&rotation_matrix_, &acc_align_rpy_);
    vector3_t vec_in = {acc_processed[0], acc_processed[1], acc_processed[2]};
    vector3_t vec_out;
    matrixVectorMul(&vec_out, &rotation_matrix_, &vec_in);
    acc_processed[0] = vec_out.x;
    acc_processed[1] = vec_out.y;
    acc_processed[2] = vec_out.z;
  }

  // 步骤2: 应用校准（零偏校正）
  // 如果正在校准，更新校准状态
  if (calibration_started_ && acc_calibration_.isCalibrating()) {
    acc_calibration_.updateCalibration(acc_processed);
  }
  
  // 如果校准完成，应用零偏校正
  if (acc_calibration_.isCalibrationComplete()) {
    acc_calibration_.applyZeroOffset(acc_processed, acc_processed);
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
  if (getParam("acc_filter_cutoff_hz", &acc_filter_cutoff_hz, sizeof(acc_filter_cutoff_hz)) != RT_EOK) {
    acc_filter_cutoff_hz = 50;  // 如果参数不存在，使用默认值 50Hz
  }

  // 获取采样频率（从 IMU 模块获取，或使用默认值）
  float sample_rate_hz = 3200.0f;  // 默认 3.2kHz
  // TODO: 从 IMU 模块获取实际采样频率

  if (acc_filter_cutoff_hz > 0 && sample_rate_hz > 0) {
    float dT = 1.0f / sample_rate_hz;
    const float k = pt2FilterGain(static_cast<float>(acc_filter_cutoff_hz), dT);
    for (int i = 0; i < 3; i++) {
      pt2FilterInit(&pt2_filter_[i], k);
    }
    pt2_filter_enabled_ = true;
    LOG_I("Acc PT2 filter initialized: cutoff=%u Hz, sample_rate=%.1f Hz", acc_filter_cutoff_hz, sample_rate_hz);
  }
}

rt_err_t AccBf::startCalibration() {
  if (calibration_started_) {
    LOG_W("Acc calibration already started");
    return RT_EOK;
  }

  float sample_rate_hz = 3200.0f;  // TODO: 从实际采样频率获取
  uint32_t duration_ms = PROJECT_BF_ACC_CALIBRATION_DURATION_MS;

  acc_calibration_.startCalibration(sample_rate_hz, duration_ms);
  calibration_started_ = true;

  LOG_I("Acc calibration started: duration=%u ms", duration_ms);
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

