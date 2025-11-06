#include <stdint.h>
#include <string.h>

#include "rtdef.h"
#include "uparam.h"

// 如果 mixer 功能可用，则声明 reloadMixer 函数
#ifdef PKG_USING_MIXER
extern void reloadMixer(void);
#endif

typedef struct {
    void *param;
    const void *default_value;
} param_default_t;

static void params_default(void *address, uint8_t size);

// flight/angle_pid - 角度环 PID 参数
/* angle_pid_roll - Roll轴角度环PID [P, I, D] */
static float angle_pid_roll[3];
/* angle_pid_pitch - Pitch轴角度环PID [P, I, D] */
static float angle_pid_pitch[3];
/* angle_pid_yaw - Yaw轴角度环PID [P, I, D] */
static float angle_pid_yaw[3];

// flight/rate_pid - 角速度环 PID 参数
/* rate_pid_roll - Roll轴角速度环PID [P, I, D] */
static float rate_pid_roll[3];
/* rate_pid_pitch - Pitch轴角速度环PID [P, I, D] */
static float rate_pid_pitch[3];
/* rate_pid_yaw - Yaw轴角速度环PID [P, I, D] */
static float rate_pid_yaw[3];

const param_list params[] = {
    // flight/angle_pid - 角度环 PID
    {(void *)&angle_pid_roll, sizeof(angle_pid_roll), "angle_pid_roll", "vf", params_default},
    {(void *)&angle_pid_pitch, sizeof(angle_pid_pitch), "angle_pid_pitch", "vf", params_default},
    {(void *)&angle_pid_yaw, sizeof(angle_pid_yaw), "angle_pid_yaw", "vf", params_default},

    // flight/rate_pid - 角速度环 PID
    {(void *)&rate_pid_roll, sizeof(rate_pid_roll), "rate_pid_roll", "vf", params_default},
    {(void *)&rate_pid_pitch, sizeof(rate_pid_pitch), "rate_pid_pitch", "vf", params_default},
    {(void *)&rate_pid_yaw, sizeof(rate_pid_yaw), "rate_pid_yaw", "vf", params_default},
};

// 默认参数值定义
// 角度环 PID 默认值 [P, I, D]
const float angle_pid_roll_default[3] = {6.0f, 3.0f, 0.0f};    // Roll轴角度环
const float angle_pid_pitch_default[3] = {6.0f, 3.0f, 0.0f};   // Pitch轴角度环
const float angle_pid_yaw_default[3] = {6.0f, 3.0f, 0.0f};     // Yaw轴角度环

// 角速度环 PID 默认值 [P, I, D]
const float rate_pid_roll_default[3] = {42.0f, 65.0f, 29.0f};   // Roll轴角速度环
const float rate_pid_pitch_default[3] = {45.0f, 62.0f, 31.0f};  // Pitch轴角速度环
const float rate_pid_yaw_default[3] = {30.0f, 50.0f, 0.0f};     // Yaw轴角速度环

// 默认参数映射表
const param_default_t defaultParams_[] = {
    // flight/angle_pid
    {&angle_pid_roll, angle_pid_roll_default},
    {&angle_pid_pitch, angle_pid_pitch_default},
    {&angle_pid_yaw, angle_pid_yaw_default},

    // flight/rate_pid
    {&rate_pid_roll, rate_pid_roll_default},
    {&rate_pid_pitch, rate_pid_pitch_default},
    {&rate_pid_yaw, rate_pid_yaw_default},
};

static void params_default(void *address, uint8_t size) {
    // 遍历参数列表并设置默认值
    for (size_t i = 0; i < sizeof(defaultParams_) / sizeof(param_default_t); i++) {
        if (address == defaultParams_[i].param) {
            memcpy(address, defaultParams_[i].default_value, size);
            break;
        }
    }
}

int uparam_data_init(void) {
    uparam_add_list(params, sizeof(params) / sizeof(param_list));
    return 0;  // 返回 0 表示成功
}

#ifdef PKG_USING_UPARAM
INIT_DEVICE_EXPORT(uparam_data_init);
#endif

/**
 * @brief 根据参数名称获取参数值
 *
 * 该函数用于在参数列表中查找指定名称的参数，并将其值拷贝到用户提供的缓冲区中。
 *
 * @param name 参数名称的字符串指针
 * @param value 用户提供的缓冲区指针，用于存储找到的参数值
 * @param value_size 用户提供的缓冲区大小（以字节为单位）
 * @return int 返回操作结果：
 *         - RT_EOK（0）：成功找到参数并拷贝值
 *         - -RT_ERROR：未找到参数或缓冲区大小不足
 *
 * @note 用户需要确保提供的缓冲区大小（value_size）足够存储参数值，否则函数会返回错误。
 *       参数列表（params）是一个全局数组，包含所有可用的参数及其元数据。
 */
rt_err_t getParam(char *name, void *value, size_t value_size) {
    // 遍历 params 数组
    for (size_t i = 0; i < sizeof(params) / sizeof(param_list); i++) {
        // 检查名字是否匹配
        if (strcmp(params[i].name, name) == 0) {
            // 检查传入指针的大小是否满足参数的大小
            if (value_size < params[i].size) {
                return -RT_ERROR;  // 返回错误，表示传入指针大小不足
            }
            // 拷贝数据到传入的指针中
            memcpy(value, params[i].address, params[i].size);
            return RT_EOK;  // 返回成功
        }
    }
    return -RT_ERROR;  // 返回错误，表示未找到参数
}

/**
 * @brief 根据参数名称设置参数值
 *
 * 该函数用于在参数列表中查找指定名称的参数，并将用户提供的值拷贝到对应的内存地址中。
 * 设置完成后调用 uparam_flush 函数将参数写入到存储中。
 *
 * @param name 参数名称的字符串指针
 * @param value 用户提供的值指针
 * @param value_size 用户提供的值大小（以字节为单位）
 * @return int 返回操作结果：
 *         - RT_EOK（0）：成功设置参数并刷新存储
 *         - -RT_ERROR：未找到参数、大小不匹配或刷新失败
 *
 * @note 用户需要确保提供的值大小（value_size）与参数的大小一致，否则函数会返回错误。
 */
rt_err_t setParam(char *name, void *value, size_t value_size) {
    // 遍历 params 数组
    for (size_t i = 0; i < sizeof(params) / sizeof(param_list); i++) {
        // 检查名字是否匹配
        if (strcmp(params[i].name, name) == 0) {
            // 检查传入值的大小是否匹配参数的大小
            if (value_size != params[i].size) {
                return -RT_ERROR;  // 返回错误，表示大小不匹配
            }
            // 拷贝值到对应的内存地址
            memcpy(params[i].address, value, params[i].size);

            // Apply motor_reverse to mixer in real time
#ifdef PKG_USING_MIXER
            if (strcmp(params[i].name, "motor_reverse") == 0) {
                reloadMixer();
            }
#endif

            // 调用 uparam_flush 刷新存储
            uint16_t flush_result = uparam_flush();
            if (flush_result > 0) {
                return RT_EOK;  // 返回成功
            } else {
                return -RT_ERROR;  // 返回错误，表示刷新失败
            }
        }
    }
    return -RT_ERROR;  // 返回错误，表示未找到参数
}

/**
 * @brief 获取参数总数量
 *
 * @return size_t 参数总数量
 */
size_t getParamCount(void) { return sizeof(params) / sizeof(param_list); }

/**
 * @brief 根据索引获取参数信息
 *
 * @param index 参数索引 (0-based)
 * @return const param_list* 参数信息指针，如果索引无效则返回NULL
 */
const param_list *getParamByIndex(size_t index) {
    size_t param_count = sizeof(params) / sizeof(param_list);
    if (index >= param_count) {
        return NULL;  // 索引超出范围
    }
    return &params[index];
}
