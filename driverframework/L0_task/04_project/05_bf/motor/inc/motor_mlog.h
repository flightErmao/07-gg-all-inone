/**
 * @file motor_mlog.h
 * 
 * Mlog Motor 数据记录类
 * 用于记录最终下发给 dshot 的电机数据
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
 * @brief Motor mlog 数据结构体
 * 用于快速修改需要记录到 log 中的数据
 */
struct motor_mlog_data_t {
  uint32_t seq;                       // 序列号
  uint32_t timestamp;                 // 时间戳（微秒）
  uint16_t motor_values[4];            // 最终下发给 dshot 的 4 个通道数据 (16bit uint16)
} __packed;

/**
 * @brief Mlog Motor 数据记录类
 * 参考 gyro_mlog 的实现方式
 */
class MlogMotor {
public:
    MlogMotor();
    ~MlogMotor();
    
    /**
     * @brief 获取单例实例
     */
    static MlogMotor* getInstance();
    
    /**
     * @brief 初始化 mlog Motor 功能
     */
    void init();
    
    /**
     * @brief 推送 Motor 数据到 mlog
     * @param data Motor mlog 数据结构体
     */
    void pushMotorData(const motor_mlog_data_t* data);
    
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
    static MlogMotor* instance_;  // 单例实例（用于回调）
};

}  // namespace bf_mlog

