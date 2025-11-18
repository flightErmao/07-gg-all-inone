/**
 * @file dyn_notch_filter_stub.c
 * 
 * 动态 notch 滤波器 stub 实现
 * 由于 ref 文件夹中的原始实现依赖 Betaflight 框架，
 * 这里提供一个简单的 stub 实现以支持编译和链接
 */

#include <stdint.h>
#include <stdbool.h>

// 时间类型（微秒）
typedef uint32_t timeUs_t;

// 动态 notch 配置结构体（最小化定义）
typedef struct dynNotchConfig_s {
    uint8_t dyn_notch_count;
    uint16_t dyn_notch_q;
    uint16_t dyn_notch_min_hz;
    uint16_t dyn_notch_max_hz;
} dynNotchConfig_t;

// 全局状态：动态 notch 滤波器未激活
static bool dyn_notch_active = false;

void dynNotchInit(const dynNotchConfig_t *config, timeUs_t targetLooptimeUs)
{
    (void)config;
    (void)targetLooptimeUs;
    // Stub: 暂时禁用动态 notch 滤波器
    dyn_notch_active = false;
}

void dynNotchPush(int axis, float sample)
{
    (void)axis;
    (void)sample;
    // Stub: 不做任何处理
}

void dynNotchUpdate(void)
{
    // Stub: 不做任何处理
}

float dynNotchFilter(int axis, float value)
{
    (void)axis;
    // Stub: 直接返回原始值
    return value;
}

bool isDynNotchActive(void)
{
    return dyn_notch_active;
}

int getMaxFFT(void)
{
    // Stub: 返回 0
    return 0;
}

void resetMaxFFT(void)
{
    // Stub: 不做任何处理
}

