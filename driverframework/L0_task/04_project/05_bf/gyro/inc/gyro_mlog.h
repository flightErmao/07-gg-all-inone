/**
 * @file gyro_mlog.h
 * 
 * Mlog 陀螺仪数据记录类
 * 用于记录 IMU 滤波前后的陀螺仪数据
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
 * @brief 陀螺仪 mlog 数据结构体
 * 用于快速修改需要记录到 log 中的数据
 */
struct gyro_mlog_data_t {
  uint32_t seq;                       // 序列号
  uint32_t timestamp;                 // 时间戳（微秒）
  float gyro_adc[3];                  // 对齐、校准但未滤波的数据（对应 gyro_adc_）
  float gyro_filtered[3];             // 滤波后的陀螺仪数据（对应 gyro_adcf_）
} __packed;

/**
 * @brief Mlog 陀螺仪数据记录类
 * 参考 aMlogStabilze.c 的实现方式
 */
class MlogGyro {
public:
    MlogGyro();
    ~MlogGyro();
    
    /**
     * @brief 获取单例实例
     */
    static MlogGyro* getInstance();
    
    /**
     * @brief 初始化 mlog gyro 功能
     */
    void init();
    
    /**
     * @brief 推送陀螺仪数据到 mlog
     * @param data 陀螺仪 mlog 数据结构体
     */
    void pushGyroData(const gyro_mlog_data_t* data);
    
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
    static MlogGyro* instance_;  // 单例实例（用于回调）
};

}  // namespace bf_mlog

