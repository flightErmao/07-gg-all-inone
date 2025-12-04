#include "attitude_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "attitude"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "timestamp.h"
#include "../common/inc/init_sync.h"
#include "../imu/inc/imu_mcn.h"  // For gyro_raw_msg_t
#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
#include "debugPin.h"
#endif
#ifdef PROJECT_BF_ACC_EN
#include "../acc/inc/acc_mcn.h"
#endif
}

#include "../imu/inc/bmi270_class.h"

#include <cstring>
#include <cmath>

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI_F;
constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr float GRAVITY = 9.80665f;  // m/s^2

// 互补滤波器默认系数（参考 Betaflight）
// alpha 越小，越信任陀螺仪（动态响应好）
// alpha 越大，越信任加速度计（静态准确）
constexpr float DEFAULT_COMPLEMENTARY_ALPHA = 0.02f;  // 2% 加速度计，98% 陀螺仪

// Betaflight风格的数值稳定性阈值（仅用于避免极小向量归一化）
// 约0.05mG = 0.00005G，转换为ADC：0.00005G * 2048 ≈ 0.1 ADC
// 平方和阈值：0.1^2 = 0.01 ADC²（与Betaflight一致）
// 目的：避免对接近零的向量归一化，提高数值稳定性
// 正常情况：静止时Z轴约±2048（±1G），平方和约为 2048² ≈ 4,194,304，远大于 0.01
// 异常情况：三轴都接近0（传感器故障、数据异常），平方和可能小于 0.01，此时不应归一化
constexpr float ACC_NUMERICAL_STABILITY_THRESHOLD_SQ = 0.01f;  // 数值稳定性阈值（ADC²）
}  // namespace

// AttitudeBf 单例实现
AttitudeBf& AttitudeBf::instance() {
  static AttitudeBf instance_obj;
  return instance_obj;
}

AttitudeBf::AttitudeBf()
    : quaternion_{1.0f, 0.0f, 0.0f, 0.0f},  // 初始四元数：无旋转 [1, 0, 0, 0]
      attitude_values_{0.0f, 0.0f, 0.0f},
      update_freq_hz_(PROJECT_BF_ATTITUDE_UPDATE_FREQ_HZ),
      update_period_us_(0),
      downsample_counter_(0),
      downsample_factor_(0),
      imu_process_denom_(0),  // 将在initInThreadEntry中从IMU模块获取
      imu_process_counter_(0),
      gyro_event_(RT_NULL),
      gyro_node_(RT_NULL),
#ifdef PROJECT_BF_ACC_EN
      acc_event_(RT_NULL),
      acc_node_(RT_NULL),
#endif
      attitude_hub_(nullptr),
      thread_(RT_NULL),
      thread_inited_(false),
      complementary_alpha_(DEFAULT_COMPLEMENTARY_ALPHA),
      initialized_(false) {
  std::memset(&thread_obj_, 0, sizeof(thread_obj_));
  std::memset(thread_stack_, 0, sizeof(thread_stack_));
}

AttitudeBf::~AttitudeBf() {
  cleanupMcnSubscriptions();
}

rt_err_t AttitudeBf::init() {
  if (thread_inited_) {
    LOG_W("AttitudeBf already initialized");
    return RT_EOK;
  }

  // 初始化 MCN（订阅和发布）
  rt_err_t ret = initMcn();
  if (ret != RT_EOK) {
    LOG_E("MCN initialization failed");
    return ret;
  }

  // 计算更新周期
  update_period_us_ = 1000000U / update_freq_hz_;

  // 计算降采样因子（假设 IMU 采样频率为 3.2kHz）
  // 如果 IMU 频率未知，将在线程入口中从 IMU 模块获取
  const uint32_t imu_freq_hz = 3200U;  // 默认值，实际从 IMU 获取
  downsample_factor_ = imu_freq_hz / update_freq_hz_;
  if (downsample_factor_ < 1) {
    downsample_factor_ = 1;
  }

  LOG_I("AttitudeBf initialized: update_freq=%u Hz, period=%u us, downsample_factor=%u", 
        update_freq_hz_, update_period_us_, downsample_factor_);

  return RT_EOK;
}

rt_err_t AttitudeBf::startThread() {
  if (thread_inited_) {
    LOG_W("AttitudeBf thread already started");
    return RT_EOK;
  }

  // 初始化线程对象
  rt_err_t ret = rt_thread_init(&thread_obj_, "attitude", threadEntry, this, thread_stack_,
                                 PROJECT_BF_ATTITUDE_THREAD_STACK_SIZE,
                                 PROJECT_BF_ATTITUDE_THREAD_PRIORITY,
                                 PROJECT_BF_ATTITUDE_THREAD_TIMESLICE);

  if (ret != RT_EOK) {
    LOG_E("AttitudeBf thread init failed: %d", ret);
    return ret;
  }

  thread_ = &thread_obj_;
  thread_inited_ = true;

  ret = rt_thread_startup(thread_);
  if (ret != RT_EOK) {
    LOG_E("AttitudeBf thread startup failed: %d", ret);
    thread_inited_ = false;
    return ret;
  }

  LOG_I("AttitudeBf thread started");
  return RT_EOK;
}

void AttitudeBf::threadEntry(void* parameter) {
  auto* self = static_cast<AttitudeBf*>(parameter);
  if (!self) {
    return;
  }

  // 在线程调度器启动后，等待 Gyro Filter 初始化完成
  rt_err_t ret = initSyncWait(INIT_SYNC_GYRO_FILTER, 2000);
  if (ret != RT_EOK) {
    LOG_W("Gyro Filter not ready, continuing anyway (ret=%d)", ret);
  }

#ifdef PROJECT_BF_ACC_EN
  // 等待 Acc 模块初始化完成
  ret = initSyncWait(INIT_SYNC_ACC, 2000);
  if (ret != RT_EOK) {
    LOG_W("Acc module not ready, continuing anyway (ret=%d)", ret);
  }
#endif

  // 在线程中执行初始化（依赖其他模块）
  self->initInThreadEntry();

  // 通知 Attitude 初始化完成
  initSyncNotify(INIT_SYNC_ATTITUDE);

  // 进入主循环
  self->threadLoop();
}

void AttitudeBf::initInThreadEntry() {
  // 从 IMU 模块获取IMU处理分频因子
  using namespace bf_bmi270;
  imu_process_denom_ = BMI270::instance().getImuProcessDenom();
  LOG_I("AttitudeBf: Got imu_process_denom from IMU: %u", imu_process_denom_);

  LOG_I("AttitudeBf thread initialization complete");
}

void AttitudeBf::threadLoop() {
  LOG_I("AttitudeBf thread loop started");

  uint32_t last_update_time_us = timestamp_micros();
  imu_process_counter_ = 0;

#ifdef PROJECT_BF_ACC_EN
  if (acc_node_ == RT_NULL) {
    LOG_E("AttitudeBf: acc_node_ is NULL, cannot continue");
    return;
  }
#endif

  while (true) {
#ifdef PROJECT_BF_ACC_EN
    // 同步阻塞等待 acc 滤波数据（800Hz）
    if (mcn_poll_sync(acc_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      acc_filtered_msg_t acc_data;
      if (mcn_copy(MCN_HUB(acc), acc_node_, &acc_data) == RT_EOK) {
        // 使用imu_process_denom_进行分频：每imu_process_denom_次acc更新，只处理1次
        // 如果imu_process_denom_为0，说明还未初始化，使用默认值1（不分频）
        uint8_t denom = (imu_process_denom_ > 0) ? imu_process_denom_ : 1;
        imu_process_counter_++;

        // 只有当计数器值大于(denom - 1)时才执行，否则立即下一次循环
        if (imu_process_counter_ <= (denom - 1)) {
          continue;
        }

        // 执行姿态估计处理
        imu_process_counter_ = 0;

#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
        DEBUG_PIN_DEBUG1_HIGH();  // Debug pin: Attitude task execution start
#endif

        // 计算时间差
        uint32_t current_time_us = timestamp_micros();
        float dt = (current_time_us - last_update_time_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.1f) {
          // 时间异常，使用默认值
          dt = 1.0f / update_freq_hz_;
        }
        last_update_time_us = current_time_us;

        // 获取处理后的加速度计数据
        float acc_filtered[3] = {0.0f, 0.0f, 0.0f};
        std::memcpy(acc_filtered, acc_data.acc_filtered, sizeof(acc_filtered));

        // 非阻塞poll gyro滤波后的数据（使用专门为attitude准备的滤波数据）
        float gyro_attitude[3] = {0.0f, 0.0f, 0.0f};
        if (gyro_node_ != RT_NULL) {
          gyro_filtered_msg_t gyro_data;
          if (mcn_poll(gyro_node_) == RT_TRUE) {
            if (mcn_copy(MCN_HUB(gyro), gyro_node_, &gyro_data) == RT_EOK) {
              std::memcpy(gyro_attitude, gyro_data.gyro_filtered_for_attitude, sizeof(gyro_attitude));
            }
          }
        }

        // 更新姿态估计
        updateAttitude(acc_filtered, gyro_attitude, dt);

        // 发布姿态数据（使用acc_data的seq，发布attitude_values_数据）
        publishAttitude(acc_data.seq);

#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
        DEBUG_PIN_DEBUG1_LOW();  // Debug pin: Attitude task execution end
#endif
      }
    }
#else
    LOG_E("AttitudeBf: PROJECT_BF_ACC_EN not enabled, cannot run");
    rt_thread_mdelay(100);
#endif
  }
}

void AttitudeBf::quaternionNormalize(float q[4]) {
  float norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm > 0.0001f) {
    float inv_norm = 1.0f / norm;
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
  } else {
    // 归一化失败，重置为单位四元数
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
  }
}

void AttitudeBf::eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg, float q[4]) {
  // 转换为弧度
  float roll_rad = roll_deg * DEG_TO_RAD;
  float pitch_rad = pitch_deg * DEG_TO_RAD;
  float yaw_rad = yaw_deg * DEG_TO_RAD;

  // 计算半角
  float cr = std::cos(roll_rad * 0.5f);
  float sr = std::sin(roll_rad * 0.5f);
  float cp = std::cos(pitch_rad * 0.5f);
  float sp = std::sin(pitch_rad * 0.5f);
  float cy = std::cos(yaw_rad * 0.5f);
  float sy = std::sin(yaw_rad * 0.5f);

  // 四元数计算（ZYX 顺序，与 Betaflight 一致）
  q[0] = cr * cp * cy + sr * sp * sy;  // w (标量部分)
  q[1] = sr * cp * cy - cr * sp * sy;  // x (向量部分 i)
  q[2] = cr * sp * cy + sr * cp * sy;  // y (向量部分 j)
  q[3] = cr * cp * sy - sr * sp * cy;  // z (向量部分 k)
}

void AttitudeBf::quaternionToEuler(const float q[4], float* roll_deg, float* pitch_deg, float* yaw_deg) {
  // 从四元数提取欧拉角（ZYX 顺序，与 Betaflight 一致）
  // 参考 Betaflight 的 quaternionToEulerAngles 函数

  float q0 = q[0];  // w
  float q1 = q[1];  // x
  float q2 = q[2];  // y
  float q3 = q[3];  // z

  // Roll (X 轴旋转)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  *roll_deg = std::atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

  // Pitch (Y 轴旋转)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (std::abs(sinp) >= 1.0f) {
    // 使用 90 度（避免万向锁）
    *pitch_deg = (sinp > 0.0f ? 90.0f : -90.0f);
  } else {
    *pitch_deg = std::asin(sinp) * RAD_TO_DEG;
  }

  // Yaw (Z 轴旋转)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  *yaw_deg = std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

void AttitudeBf::quaternionIntegrate(const float q[4], const float gyro[3], float dt, float q_new[4]) {
  // 四元数积分：使用陀螺仪数据更新四元数
  // 参考 Betaflight 的 quaternionIntegrate 函数

  // 陀螺仪数据转换为弧度/秒
  float gx_rad = gyro[0] * DEG_TO_RAD;
  float gy_rad = gyro[1] * DEG_TO_RAD;
  float gz_rad = gyro[2] * DEG_TO_RAD;

  // 四元数导数
  float q0 = q[0];
  float q1 = q[1];
  float q2 = q[2];
  float q3 = q[3];

  // 四元数导数计算
  float q0_dot = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
  float q1_dot = 0.5f * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
  float q2_dot = 0.5f * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
  float q3_dot = 0.5f * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

  // 积分更新
  q_new[0] = q0 + q0_dot * dt;
  q_new[1] = q1 + q1_dot * dt;
  q_new[2] = q2 + q2_dot * dt;
  q_new[3] = q3 + q3_dot * dt;

  // 归一化
  quaternionNormalize(q_new);
}

// 从四元数提取重力向量（在机体坐标系中）
// 重力在NED坐标系中为 [0, 0, 1]（向下），在机体坐标系中的表示为旋转后的向量
void AttitudeBf::quaternionToGravityVector(const float q[4], float gravity_vec[3]) {
  // 重力向量在世界坐标系（NED）中为 [0, 0, 1]
  // 通过四元数旋转到机体坐标系
  // 重力向量在机体坐标系中的表示为：R^T * [0, 0, 1]
  // 使用旋转矩阵第三列（Z轴方向）
  
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  
  // 旋转矩阵第三列（Z轴在机体坐标系中的方向）
  gravity_vec[0] = 2.0f * (q1 * q3 - q0 * q2);  // X分量
  gravity_vec[1] = 2.0f * (q0 * q1 + q2 * q3);  // Y分量
  gravity_vec[2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);  // Z分量
}

// Betaflight风格：使用误差向量修正姿态（类似Mahony互补滤波器）
void AttitudeBf::updateAttitudeWithErrorVector(const float accel_norm[3], const float gyro[3], float dt) {
  // 1. 从当前四元数计算重力向量在机体坐标系中的表示
  float estimated_gravity[3];
  quaternionToGravityVector(quaternion_, estimated_gravity);
  
  // 2. 计算误差向量：测量的重力方向与估计的重力方向的叉积
  // 误差向量表示姿态估计的偏差方向
  float error[3];
  error[0] = accel_norm[1] * estimated_gravity[2] - accel_norm[2] * estimated_gravity[1];
  error[1] = accel_norm[2] * estimated_gravity[0] - accel_norm[0] * estimated_gravity[2];
  error[2] = accel_norm[0] * estimated_gravity[1] - accel_norm[1] * estimated_gravity[0];
  
  // 3. 使用互补滤波器系数对误差进行加权（用于积分反馈）
  float gyro_corrected[3];
  const float kp = complementary_alpha_;  // 比例增益（通常很小，如0.02）
  gyro_corrected[0] = gyro[0] + kp * error[0];
  gyro_corrected[1] = gyro[1] + kp * error[1];
  gyro_corrected[2] = gyro[2] + kp * error[2];
  
  // 4. 使用修正后的陀螺仪数据积分更新四元数
  quaternionIntegrate(quaternion_, gyro_corrected, dt, quaternion_);
  
  // 5. 归一化四元数（防止数值漂移）
  quaternionNormalize(quaternion_);
  
  // 6. 从四元数转换为欧拉角（用于输出）
  quaternionToEuler(quaternion_, &attitude_values_[0], &attitude_values_[1], &attitude_values_[2]);
  
  // 7. 限制 yaw 角度范围
  if (attitude_values_[2] > 180.0f) {
    attitude_values_[2] -= 360.0f;
  } else if (attitude_values_[2] < -180.0f) {
    attitude_values_[2] += 360.0f;
  }
  
  initialized_ = true;
}

void AttitudeBf::updateAttitude(const float accel[3], const float gyro[3], float dt) {
  // 使用四元数进行姿态估计，避免万向锁
  // 采用Betaflight风格的误差向量方法（类似Mahony互补滤波器），更高效且数学严谨
  // 输入：accel[3] 是原始ADC值（16G量程时，2048 ADC = 1G），gyro[3] 是角速度（deg/s）
  
  // Betaflight风格：只归一化加速度计（方向信息），不需要物理单位转换
  // 因为我们只关心重力方向，而不是重力大小
  // 计算加速度计向量的模长平方（ADC值的平方和）
  float accel_magnitude_sq = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
  
  // 检查加速度计数据有效性（避免自由落体或异常情况）
  // 使用数值稳定性阈值（与Betaflight一致）：0.01 ADC²
  // 这个阈值很小（约0.05mG），主要用于：
  // 1. 数值稳定性：避免对极小向量归一化时放大噪声
  // 2. 数据有效性：过滤传感器故障或异常数据（三轴都接近0的情况）
  // 
  // 正常情况：静止时Z轴约±2048（±1G），X/Y轴接近0，
  // 平方和约为 2048² ≈ 4,194,304，远大于 0.01
  // 
  // 异常情况：如果三轴都接近0（传感器故障、数据异常），
  // 平方和可能小于 0.01，此时不应归一化
  if (accel_magnitude_sq > ACC_NUMERICAL_STABILITY_THRESHOLD_SQ) {
    // 归一化加速度计向量（转换为单位向量，只保留方向信息）
    float inv_magnitude = 1.0f / std::sqrt(accel_magnitude_sq);
    float accel_norm[3];
    accel_norm[0] = accel[0] * inv_magnitude;
    accel_norm[1] = accel[1] * inv_magnitude;
    accel_norm[2] = accel[2] * inv_magnitude;
    
    // 使用误差向量方法更新姿态（Betaflight风格）
    updateAttitudeWithErrorVector(accel_norm, gyro, dt);
  } else {
    // 加速度计数据无效（太小，可能是自由落体、传感器故障或异常），
    // 只使用陀螺仪积分（不进行加速度计修正）
    float q_new[4];
    quaternionIntegrate(quaternion_, gyro, dt, q_new);
    std::memcpy(quaternion_, q_new, sizeof(quaternion_));
    quaternionNormalize(quaternion_);
    
    // 更新欧拉角
    quaternionToEuler(quaternion_, &attitude_values_[0], &attitude_values_[1], &attitude_values_[2]);
    
    if (attitude_values_[2] > 180.0f) {
      attitude_values_[2] -= 360.0f;
    } else if (attitude_values_[2] < -180.0f) {
      attitude_values_[2] += 360.0f;
    }
  }
}

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_ATTITUDE_EN
extern "C" {
static int attitude_main_init_wrapper(void) {
  // 小延迟确保 Gyro Filter 先初始化
  rt_thread_mdelay(10);

  AttitudeBf& attitude = AttitudeBf::instance();
  rt_err_t ret = attitude.init();
  if (ret != RT_EOK) {
    LOG_E("AttitudeBf init failed: %d", ret);
    return (int)ret;
  }
  LOG_I("AttitudeBf initialized successfully");

  ret = attitude.startThread();
  if (ret == RT_EOK) {
    LOG_I("AttitudeBf auto-init success");
  } else {
    LOG_E("AttitudeBf auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(attitude_main_init_wrapper);
}
#endif

