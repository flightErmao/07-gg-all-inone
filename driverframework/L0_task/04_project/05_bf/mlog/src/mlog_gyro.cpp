/**
 * @file mlog_gyro.cpp
 * 
 * Mlog 陀螺仪数据记录实现
 * 参考 aMlogStabilze.c 的实现方式
 */

#include "mlog_gyro.hpp"
#include <rtthread.h>

extern "C" {
#include "mlog.h"
}

namespace bf_mlog {

// 陀螺仪数据结构（对应 aMlogStabilze.c 中的数据结构）
struct mlogGyroData_t {
    uint32_t timestamp;
    uint32_t seq;
    float gyro_raw[3];        // 滤波前的陀螺仪数据（对应 gyro_adc_）
    float gyro_filtered[3];   // 滤波后的陀螺仪数据（对应 gyro_adcf_）
} __packed;

// Mlog 元素定义（参考 aMlogStabilze.c:81-85）
static mlog_elem_t GyroData_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(seq, MLOG_UINT32),
    MLOG_ELEMENT_VEC(gyro_raw, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filtered, MLOG_FLOAT, 3),
};

// Mlog 总线定义（参考 aMlogStabilze.c:86）
MLOG_BUS_DEFINE(GyroData, GyroData_Elems);

// 单例实例（用于回调）
MlogGyro* MlogGyro::instance_ = nullptr;

MlogGyro::MlogGyro()
    : bus_id_(-1),
      enabled_(false),
      param_enabled_(false)
{
    instance_ = this;
}

MlogGyro::~MlogGyro()
{
    instance_ = nullptr;
}

void MlogGyro::init()
{
    // 获取 mlog 总线 ID（参考 aMlogStabilze.c:123）
    bus_id_ = mlog_get_bus_id("GyroData");
    if (bus_id_ < 0) {
        rt_kprintf("[MlogGyro] Failed to get mlog bus ID for GyroData\n");
    } else {
        rt_kprintf("[MlogGyro] GyroData mlog bus ID: %d\n", bus_id_);
    }
    
    // 注册 mlog 开始回调（参考 aMlogStabilze.c:175）
    mlog_register_callback(MLOG_CB_START, startCallback);
}

void MlogGyro::startCallback()
{
    // 当 mlog 开始时启用推送（参考 aMlogStabilze.c:187）
    if (instance_ != nullptr) {
        instance_->enabled_ = true;
    }
}

void MlogGyro::pushGyroData(uint32_t seq, uint32_t timestamp, const float gyro_raw[3], const float gyro_filtered[3])
{
    // 检查是否启用推送（参考 aMlogStabilze.c:210）
    if (bus_id_ < 0 || !enabled_) {
        return;
    }
    
    // 填充数据结构
    mlogGyroData_t gyro_data = {0};
    gyro_data.timestamp = timestamp;
    gyro_data.seq = seq;
    std::memcpy(gyro_data.gyro_raw, gyro_raw, sizeof(gyro_data.gyro_raw));
    std::memcpy(gyro_data.gyro_filtered, gyro_filtered, sizeof(gyro_data.gyro_filtered));
    
    // 推送消息到 mlog（参考 aMlogStabilze.c:211）
    mlog_push_msg(reinterpret_cast<const uint8_t*>(&gyro_data), bus_id_, sizeof(mlogGyroData_t));
}

}  // namespace bf_mlog

