/**
 * @file pid_mlog.h
 * 
 * Mlog PID 数据记录类
 * 用于记录角速度环和角度环的期望和当前快照数据
 */

#pragma once

#include <cstdint>
#include <cstring>

extern "C" {
#include <rtthread.h>
#include "mlog.h"
}

namespace bf_mlog {

/**
 * @brief PID mlog 数据结构体
 * 用于快速修改需要记录到 log 中的数据
 */
struct pid_mlog_data_t {
  uint32_t seq;                       // 序列号
  uint32_t timestamp;                 // 时间戳（微秒）
  
  // 角速度环数据（Rate loop）
  float rate_setpoint[3];             // 角速度期望值 [roll, pitch, yaw] (deg/s)
  float rate_actual[3];               // 角速度当前值 [roll, pitch, yaw] (deg/s)
  
  // 角度环数据（Angle loop）
  float angle_setpoint[2];            // 角度期望值 [roll, pitch] (degrees)
  float angle_actual[2];              // 角度当前值 [roll, pitch] (degrees)
  
  // 滤波后的油门值（来自RC平滑滤波器）
  float smoothed_throttle;            // 滤波后的油门值 (1000-2000)
} __packed;

/**
 * @brief Mlog PID 数据记录类
 * 参考 gyro_mlog 的实现方式
 */
class MlogPid {
public:
    MlogPid();
    ~MlogPid();
    
    /**
     * @brief 获取单例实例
     */
    static MlogPid* getInstance();
    
    /**
     * @brief 初始化 mlog PID 功能
     */
    void init();
    
    /**
     * @brief 推送 PID 数据到 mlog
     * @param data PID mlog 数据结构体
     */
    void pushPidData(const pid_mlog_data_t* data);
    
    /**
     * @brief 检查是否启用推送
     */
    bool isEnabled() const { return enabled_ && param_enabled_; }
    
    /**
     * @brief 设置参数使能状态（通过参数控制是否记录）
     * @param enabled 是否使能（从参数系统读取）
     */
    void setParamEnabled(bool enabled) { param_enabled_ = enabled; }

private:
    static void startCallback();
    
    int bus_id_;           // mlog 总线 ID
    bool enabled_;         // mlog 系统是否启用（通过回调控制）
    bool param_enabled_;   // 参数是否启用（通过参数系统控制）
    static MlogPid* instance_;  // 单例实例（用于回调）
};

}  // namespace bf_mlog

