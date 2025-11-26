#ifndef ATTITUDE_CLASS_H__
#define ATTITUDE_CLASS_H__

#include <rtthread.h>
#include <uMCN.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" {
#include <rtconfig.h>  // For PROJECT_BF_ATTITUDE_THREAD_STACK_SIZE
#include "imu_mcn.h"
#include "gyro_mcn.h"
#include "attitude_mcn.h"
#include "timestamp.h"
#include "../common/inc/init_sync.h"
}

#ifdef PROJECT_BF_ATTITUDE_DEBUG_PIN_EN
#include "debugPin.h"
#endif

// 简化的互补滤波器（参考 Betaflight 的姿态估计）
// 使用加速度计计算角度，陀螺仪积分，然后互补融合
class AttitudeBf {
 public:
  // Singleton pattern
  static AttitudeBf& instance();

  AttitudeBf();
  ~AttitudeBf();

  rt_err_t init();
  rt_err_t startThread();

  // 获取当前姿态角度（度）
  void getAttitude(float attitude[3]) const {
    std::memcpy(attitude, attitude_values_, sizeof(attitude_values_));
  }

  // 获取当前姿态角度（十分之一度，centidegrees）
  void getAttitudeRaw(int16_t attitude_raw[3]) const {
    for (int i = 0; i < 3; i++) {
      attitude_raw[i] = static_cast<int16_t>(attitude_values_[i] * 10.0f);
    }
  }

  // MCN 相关方法
  rt_err_t initMcn();
  void cleanupMcnSubscriptions();
  void publishAttitude(const imu_raw_msg_t* imu_data);

 private:
  AttitudeBf(const AttitudeBf&) = delete;
  AttitudeBf& operator=(const AttitudeBf&) = delete;

  // 线程入口函数
  static void threadEntry(void* parameter);
  void threadLoop();

  // 在线程入口中执行的初始化（依赖其他模块）
  void initInThreadEntry();

  // 姿态估计核心函数
  void updateAttitude(const float accel[3], const float gyro[3], float dt);

  // 从加速度计计算角度（roll 和 pitch）
  void calculateAngleFromAccel(const float accel[3], float* roll, float* pitch);

  // 四元数相关函数
  // 四元数归一化
  void quaternionNormalize(float q[4]);
  
  // 从欧拉角（度）转换为四元数
  void eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg, float q[4]);
  
  // 从四元数转换为欧拉角（度）
  void quaternionToEuler(const float q[4], float* roll_deg, float* pitch_deg, float* yaw_deg);
  
  // 四元数乘法
  void quaternionMultiply(const float q1[4], const float q2[4], float result[4]);
  
  // 四元数共轭
  void quaternionConjugate(const float q[4], float result[4]);
  
  // 使用陀螺仪数据更新四元数（四元数积分）
  void quaternionIntegrate(const float q[4], const float gyro[3], float dt, float q_new[4]);
  
  // 从加速度计计算参考四元数
  void calculateQuaternionFromAccel(const float accel[3], float q_accel[4]);
  
  // 互补滤波器更新（四元数版本）
  void complementaryFilterUpdateQuaternion(const float q_gyro[4], const float q_accel[4], float alpha, float q_result[4]);

  // 当前姿态四元数 [q0, q1, q2, q3]（q0 是标量部分）
  float quaternion_[4];
  
  // 当前姿态角度值 [roll, pitch, yaw]（单位：度，从四元数计算得出）
  float attitude_values_[3];

  // 姿态更新频率（Hz）
  uint16_t update_freq_hz_;

  // 姿态更新周期（微秒）
  uint32_t update_period_us_;

  // 降采样计数器（从 IMU 采样频率降到姿态更新频率）
  uint32_t downsample_counter_;
  uint32_t downsample_factor_;

  // MCN 订阅和发布
  rt_sem_t imu_event_;
  McnNode_t imu_node_;
  rt_sem_t gyro_event_;
  McnNode_t gyro_node_;
#ifdef PROJECT_BF_ACC_EN
  rt_sem_t acc_event_;
  McnNode_t acc_node_;
#endif
  McnHub_t attitude_hub_;

  // 线程相关
  rt_thread_t thread_;
  struct rt_thread thread_obj_;
  rt_uint8_t thread_stack_[PROJECT_BF_ATTITUDE_THREAD_STACK_SIZE];
  bool thread_inited_;

  // 互补滤波器系数（alpha: 加速度计权重，1-alpha: 陀螺仪权重）
  // 较小的 alpha 值更信任陀螺仪（动态响应好），较大的 alpha 值更信任加速度计（静态准确）
  float complementary_alpha_;

  // 初始化完成标志
  bool initialized_;
};

#endif /* ATTITUDE_CLASS_H__ */

