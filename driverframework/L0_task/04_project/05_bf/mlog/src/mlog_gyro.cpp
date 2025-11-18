/**
 * @file mlog_gyro.cpp
 * 
 * Mlog 陀螺仪数据记录实现
 * 参考 aMlogStabilze.c 的实现方式
 */

#include "mlog_gyro.hpp"
#include <rtthread.h>
#include <rtconfig.h>

extern "C" {
#include "mlog.h"
#include "debugPin.h"
}

// Debug Pin 配置宏默认值（如果未在 rtconfig.h 中定义）
#ifndef CONFIG_PROJECT_BF_MLOG_GYRO_DEBUG_PIN_INDEX
#define CONFIG_PROJECT_BF_MLOG_GYRO_DEBUG_PIN_INDEX 1
#endif

namespace bf_mlog {

// Mlog 元素定义（批量数据结构：包含 10 帧数据）
// 注意：这里定义的是批量数据结构，一次消息包含多帧数据
static mlog_elem_t GyroData_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(frame_count, MLOG_UINT32),  // 当前批次中的帧数
    // 定义 10 帧数据的数组，每帧包含：timestamp, seq, gyro_raw[3], gyro_filtered[3]
    // 由于 mlog 不支持嵌套结构，我们需要将 10 帧数据展开为数组
    // timestamp 数组（10 个 uint32）
    MLOG_ELEMENT_VEC(frames_timestamp, MLOG_UINT32, 10),
    // seq 数组（10 个 uint32）
    MLOG_ELEMENT_VEC(frames_seq, MLOG_UINT32, 10),
    // gyro_raw 数组（10 帧 × 3 轴 = 30 个 float）
    MLOG_ELEMENT_VEC(frames_gyro_raw, MLOG_FLOAT, 30),
    // gyro_filtered 数组（10 帧 × 3 轴 = 30 个 float）
    MLOG_ELEMENT_VEC(frames_gyro_filtered, MLOG_FLOAT, 30),
};

// Mlog 总线定义（参考 aMlogStabilze.c:86）
// 批量推送：一次消息包含 10 帧数据
MLOG_BUS_DEFINE(GyroData, GyroData_Elems);

// 单例实例（用于回调）
MlogGyro* MlogGyro::instance_ = nullptr;

MlogGyro::MlogGyro()
    : bus_id_(-1),
      enabled_(false),
      param_enabled_(false),
      batch_count_(0)
{
    instance_ = this;
    std::memset(batch_buffer_, 0, sizeof(batch_buffer_));
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
    // 但是需要检查参数使能状态，只有参数使能时才真正启用推送
    if (instance_ != nullptr) {
        // 如果参数已使能，则设置 enabled_ 为 true
        // 否则保持 enabled_ 为 false，这样 pushGyroData 会被跳过
        if (instance_->param_enabled_) {
            instance_->enabled_ = true;
        } else {
            instance_->enabled_ = false;
        }
    }
}

void MlogGyro::pushGyroData(uint32_t seq, uint32_t timestamp, const float gyro_raw[3], const float gyro_filtered[3])
{
    // 检查是否启用推送（参考 aMlogStabilze.c:210）
    // 必须同时满足：bus_id_ 有效、mlog 系统已启用（enabled_）、参数已使能（param_enabled_）
    if (bus_id_ < 0 || !isEnabled()) {
        return;
    }
    
    // 将数据存储到批量缓冲区
    if (batch_count_ < BATCH_SIZE) {
        mlogGyroDataFrame_t& gyro_data = batch_buffer_[batch_count_];
        gyro_data.timestamp = timestamp;
        gyro_data.seq = seq;
        std::memcpy(gyro_data.gyro_raw, gyro_raw, sizeof(gyro_data.gyro_raw));
        std::memcpy(gyro_data.gyro_filtered, gyro_filtered, sizeof(gyro_data.gyro_filtered));
        batch_count_++;
    }
    
    // 当缓冲区满时，批量推送数据
    if (batch_count_ >= BATCH_SIZE) {
        flushBatch();
    }
}

void MlogGyro::flushBatch()
{
    if (batch_count_ == 0 || bus_id_ < 0) {
        return;
    }
    
    // Debug Pin: 翻转，表示开始批量推送数据
#ifdef PROJECT_BF_MLOG_GYRO_DEBUG_PIN_EN
    DEBUG_PIN_DEBUG1_TOGGLE();
#endif
    
    // 将批量缓冲区数据打包成 mlog 消息格式
    // 注意：mlog 不支持嵌套结构，需要按照元素定义顺序打包数据
    // 元素顺序：frame_count, frames_timestamp[10], frames_seq[10], frames_gyro_raw[30], frames_gyro_filtered[30]
    
    // 计算打包后的数据大小
    // frame_count (4) + timestamp[10] (40) + seq[10] (40) + gyro_raw[30] (120) + gyro_filtered[30] (120) = 324 字节
    constexpr size_t packed_size = sizeof(uint32_t) +              // frame_count
                                    (sizeof(uint32_t) * 10) +      // frames_timestamp[10]
                                    (sizeof(uint32_t) * 10) +      // frames_seq[10]
                                    (sizeof(float) * 30) +         // frames_gyro_raw[30]
                                    (sizeof(float) * 30);          // frames_gyro_filtered[30]
    
    // 分配临时缓冲区（栈上）
    uint8_t packed_data[packed_size];
    size_t offset = 0;
    
    // 1. 写入 frame_count
    uint32_t frame_count = static_cast<uint32_t>(batch_count_);
    std::memcpy(packed_data + offset, &frame_count, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 2. 写入 frames_timestamp[10]
    for (size_t i = 0; i < batch_count_; i++) {
        std::memcpy(packed_data + offset, &batch_buffer_[i].timestamp, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    // 如果不足 10 帧，填充 0
    for (size_t i = batch_count_; i < BATCH_SIZE; i++) {
        uint32_t zero = 0;
        std::memcpy(packed_data + offset, &zero, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    
    // 3. 写入 frames_seq[10]
    for (size_t i = 0; i < batch_count_; i++) {
        std::memcpy(packed_data + offset, &batch_buffer_[i].seq, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    // 如果不足 10 帧，填充 0
    for (size_t i = batch_count_; i < BATCH_SIZE; i++) {
        uint32_t zero = 0;
        std::memcpy(packed_data + offset, &zero, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    
    // 4. 写入 frames_gyro_raw[30] (10帧 × 3轴)
    for (size_t i = 0; i < batch_count_; i++) {
        std::memcpy(packed_data + offset, batch_buffer_[i].gyro_raw, sizeof(float) * 3);
        offset += sizeof(float) * 3;
    }
    // 如果不足 10 帧，填充 0
    for (size_t i = batch_count_; i < BATCH_SIZE; i++) {
        float zero[3] = {0.0f, 0.0f, 0.0f};
        std::memcpy(packed_data + offset, zero, sizeof(float) * 3);
        offset += sizeof(float) * 3;
    }
    
    // 5. 写入 frames_gyro_filtered[30] (10帧 × 3轴)
    for (size_t i = 0; i < batch_count_; i++) {
        std::memcpy(packed_data + offset, batch_buffer_[i].gyro_filtered, sizeof(float) * 3);
        offset += sizeof(float) * 3;
    }
    // 如果不足 10 帧，填充 0
    for (size_t i = batch_count_; i < BATCH_SIZE; i++) {
        float zero[3] = {0.0f, 0.0f, 0.0f};
        std::memcpy(packed_data + offset, zero, sizeof(float) * 3);
        offset += sizeof(float) * 3;
    }
    
    // 一次性推送包含 10 帧数据的消息
    mlog_push_msg(packed_data, bus_id_, packed_size);
    
    // 清空缓冲区
    batch_count_ = 0;
}

}  // namespace bf_mlog

