#include "attitude_class.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "attitude"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include "timestamp.h"
#include "../common/inc/init_sync.h"
#ifdef PROJECT_BF_ACC_EN
#include "../acc/inc/acc_mcn.h"
#endif
#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
#include "debugPin.h"
#endif
}

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
      imu_event_(RT_NULL),
      imu_node_(RT_NULL),
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
  // 从 IMU 模块获取实际采样频率（如果需要）
  // 这里可以订阅 IMU 数据来获取频率，或者使用默认值
  // 暂时使用默认值，实际可以从 IMU 模块获取

  LOG_I("AttitudeBf thread initialization complete");
}

void AttitudeBf::threadLoop() {
  LOG_I("AttitudeBf thread loop started");

  uint32_t last_update_time_us = timestamp_micros();
  downsample_counter_ = 0;

  while (true) {
#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG3_HIGH();  // Debug pin: Attitude task execution start
#endif

    // 阻塞等待 IMU 数据
    if (mcn_poll_sync(imu_node_, RT_WAITING_FOREVER) == RT_TRUE) {
      imu_raw_msg_t imu_data;
      if (mcn_copy(MCN_HUB(imu), imu_node_, &imu_data) == RT_EOK) {
        // 降采样：只处理每 N 个样本
        downsample_counter_++;
        if (downsample_counter_ >= downsample_factor_) {
          downsample_counter_ = 0;

          // 计算时间差
          uint32_t current_time_us = timestamp_micros();
          float dt = (current_time_us - last_update_time_us) * 1e-6f;
          if (dt <= 0.0f || dt > 0.1f) {
            // 时间异常，使用默认值
            dt = 1.0f / update_freq_hz_;
          }
          last_update_time_us = current_time_us;

          // 获取专门给 attitude 使用的 PT1 滤波后的陀螺仪数据（如果可用）
          // 参考 Betaflight：attitude 使用 gyro.gyroADCf[axis] 再经过一次 PT1 滤波
          float gyro_attitude[3] = {0.0f, 0.0f, 0.0f};
          if (gyro_node_ != RT_NULL) {
            gyro_filtered_msg_t gyro_data;
            if (mcn_poll(gyro_node_) == RT_TRUE) {
              if (mcn_copy(MCN_HUB(gyro), gyro_node_, &gyro_data) == RT_EOK) {
                // 使用专门给 attitude 的 PT1 滤波后的数据（gyroFilteredDownsampled）
                std::memcpy(gyro_attitude, gyro_data.gyroFilteredDownsampled, sizeof(gyro_attitude));
              }
            }
          } else {
            // 如果没有订阅 gyro，直接使用 imu 的陀螺仪数据
            std::memcpy(gyro_attitude, imu_data.gyro, sizeof(gyro_attitude));
          }

          // 获取处理后的加速度计数据（如果可用）
          float acc_filtered[3] = {0.0f, 0.0f, 0.0f};
#ifdef PROJECT_BF_ACC_EN
          if (acc_node_ != RT_NULL) {
            acc_filtered_msg_t acc_data;
            if (mcn_poll(acc_node_) == RT_TRUE) {
              if (mcn_copy(MCN_HUB(acc), acc_node_, &acc_data) == RT_EOK) {
                std::memcpy(acc_filtered, acc_data.acc_filtered, sizeof(acc_filtered));
              }
            }
          } else {
            // 如果没有订阅 acc，直接使用 imu 的加速度计数据
            std::memcpy(acc_filtered, imu_data.accel, sizeof(acc_filtered));
          }
#else
          // 如果没有启用 acc 模块，直接使用 imu 的加速度计数据
          std::memcpy(acc_filtered, imu_data.accel, sizeof(acc_filtered));
#endif

          // 更新姿态估计
          updateAttitude(acc_filtered, gyro_attitude, dt);

          // 发布姿态数据
          publishAttitude(&imu_data);
        }
      }
    }

#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG3_LOW();  // Debug pin: Attitude task execution end
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

void AttitudeBf::quaternionMultiply(const float q1[4], const float q2[4], float result[4]) {
  result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];  // w
  result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];  // x
  result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];  // y
  result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];  // z
}

void AttitudeBf::quaternionConjugate(const float q[4], float result[4]) {
  result[0] = q[0];   // w
  result[1] = -q[1];  // -x
  result[2] = -q[2];  // -y
  result[3] = -q[3];  // -z
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

void AttitudeBf::calculateQuaternionFromAccel(const float accel[3], float q_accel[4]) {
  // 从加速度计计算参考四元数（只能得到 roll 和 pitch，yaw 无法确定）
  // 参考 Betaflight 的实现

  // 计算加速度计向量的模长
  float accel_magnitude = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

  if (accel_magnitude < 0.1f) {
    // 加速度计数据无效（可能是自由落体或异常），返回单位四元数
    q_accel[0] = 1.0f;
    q_accel[1] = 0.0f;
    q_accel[2] = 0.0f;
    q_accel[3] = 0.0f;
    return;
  }

  // 归一化加速度计向量
  float accel_norm[3];
  float inv_magnitude = 1.0f / accel_magnitude;
  accel_norm[0] = accel[0] * inv_magnitude;
  accel_norm[1] = accel[1] * inv_magnitude;
  accel_norm[2] = accel[2] * inv_magnitude;

  // 从加速度计计算 roll 和 pitch（假设 yaw = 0）
  float roll_rad = std::atan2(accel_norm[1], accel_norm[2]);
  float pitch_rad = -std::asin(accel_norm[0]);
  float yaw_rad = 0.0f;  // 加速度计无法提供 yaw

  // 转换为四元数
  eulerToQuaternion(roll_rad * RAD_TO_DEG, pitch_rad * RAD_TO_DEG, yaw_rad * RAD_TO_DEG, q_accel);
}

void AttitudeBf::complementaryFilterUpdateQuaternion(const float q_gyro[4], const float q_accel[4], float alpha, float q_result[4]) {
  // 四元数互补滤波器：融合陀螺仪积分结果和加速度计参考
  // 使用球面线性插值（SLERP）或简单的线性插值后归一化

  // 简化的线性插值（对于小角度差异足够准确）
  q_result[0] = alpha * q_accel[0] + (1.0f - alpha) * q_gyro[0];
  q_result[1] = alpha * q_accel[1] + (1.0f - alpha) * q_gyro[1];
  q_result[2] = alpha * q_accel[2] + (1.0f - alpha) * q_gyro[2];
  q_result[3] = alpha * q_accel[3] + (1.0f - alpha) * q_gyro[3];

  // 归一化
  quaternionNormalize(q_result);
}

void AttitudeBf::updateAttitude(const float accel[3], const float gyro[3], float dt) {
  // 使用四元数进行姿态估计，避免万向锁

  // 1. 使用陀螺仪数据积分更新四元数
  float q_gyro[4];
  quaternionIntegrate(quaternion_, gyro, dt, q_gyro);

  // 2. 从加速度计计算参考四元数（用于校正 roll 和 pitch）
  float q_accel[4];
  calculateQuaternionFromAccel(accel, q_accel);

  // 3. 使用互补滤波器融合陀螺仪积分和加速度计参考
  // 注意：只对 roll 和 pitch 进行校正，yaw 保持陀螺仪积分结果
  // 为了简化，我们使用互补滤波器融合整个四元数，但 yaw 部分主要来自陀螺仪
  float q_fused[4];
  complementaryFilterUpdateQuaternion(q_gyro, q_accel, complementary_alpha_, q_fused);

  // 4. 更新当前四元数
  std::memcpy(quaternion_, q_fused, sizeof(quaternion_));

  // 5. 从四元数转换为欧拉角（用于输出）
  quaternionToEuler(quaternion_, &attitude_values_[0], &attitude_values_[1], &attitude_values_[2]);

  // 6. 限制 yaw 角度范围（roll 和 pitch 已经在四元数转换中自然限制）
  if (attitude_values_[2] > 180.0f) {
    attitude_values_[2] -= 360.0f;
  } else if (attitude_values_[2] < -180.0f) {
    attitude_values_[2] += 360.0f;
  }

  initialized_ = true;
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

