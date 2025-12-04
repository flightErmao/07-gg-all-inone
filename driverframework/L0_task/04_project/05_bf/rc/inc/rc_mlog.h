/**
 * @file rc_mlog.h
 * 
 * Mlog RC 数据记录类
 * 用于记录 RC 控制数据，包括 rawSetpoint、throttle、aux 模式和上锁状态
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
 * @brief RC mlog 数据结构体
 * 用于快速修改需要记录到 log 中的数据
 */
struct rc_mlog_data_t {
  uint32_t seq;                       // 序列号
  uint32_t timestamp;                 // 时间戳（微秒）
  float raw_channels[6];              // 前 6 个通道的原始数据 [ch0-ch5] (1000-2000)
  float rawSetpoint[3];                // Raw setpoint rates [roll, pitch, yaw] (deg/s)
  float rcCommandThrottle;            // RC command throttle (1000-2000)
  uint8_t armed;                      // 上锁状态 (0=DISARMED, 1=ARMED)
  uint8_t flight_mode;                // 飞行模式 (0=角速度模式/Rate, 1=角度模式/Angle, 2=高度模式/Altitude)
} __packed;

/**
 * @brief Mlog RC 数据记录类
 * 参考 gyro_mlog 的实现方式
 */
class MlogRc {
public:
    MlogRc();
    ~MlogRc();
    
    /**
     * @brief 获取单例实例
     */
    static MlogRc* getInstance();
    
    /**
     * @brief 初始化 mlog RC 功能
     */
    void init();
    
    /**
     * @brief 推送 RC 数据到 mlog
     * @param data RC mlog 数据结构体
     */
    void pushRcData(const rc_mlog_data_t* data);
    
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
    static MlogRc* instance_;  // 单例实例（用于回调）
};

}  // namespace bf_mlog

