#ifndef INIT_SYNC_H__
#define INIT_SYNC_H__

#include <rtthread.h>
#include <rtdef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化同步模块标识符
 * 
 * 用于标识不同的初始化阶段，确保模块按正确顺序初始化
 */
typedef enum {
  INIT_SYNC_BMI270 = 0,      // BMI270 IMU 初始化完成
  INIT_SYNC_GYRO_FILTER,     // Gyro Filter 初始化完成
  INIT_SYNC_RC,              // RC 模块初始化完成
  INIT_SYNC_PID,             // PID 模块初始化完成
  INIT_SYNC_MOTOR,           // Motor 模块初始化完成
  INIT_SYNC_COUNT            // 模块数量（用于数组大小）
} init_sync_id_t;

/**
 * @brief 初始化同步模块
 * 
 * 创建所有初始化同步信号量
 * 应在系统启动时调用一次
 * 
 * @return rt_err_t RT_EOK 成功，其他值表示失败
 */
rt_err_t initSyncInit(void);

/**
 * @brief 通知某个模块初始化完成
 * 
 * 当模块初始化完成后调用此函数，释放对应的信号量
 * 
 * @param id 模块标识符
 * @return rt_err_t RT_EOK 成功，其他值表示失败
 */
rt_err_t initSyncNotify(init_sync_id_t id);

/**
 * @brief 等待某个模块初始化完成
 * 
 * 阻塞等待直到指定模块初始化完成
 * 
 * @param id 模块标识符
 * @param timeout_ms 超时时间（毫秒），RT_WAITING_FOREVER 表示永久等待
 * @return rt_err_t RT_EOK 成功，-RT_ETIMEOUT 超时，其他值表示失败
 */
rt_err_t initSyncWait(init_sync_id_t id, rt_int32_t timeout_ms);

/**
 * @brief 检查某个模块是否已初始化完成（非阻塞）
 * 
 * @param id 模块标识符
 * @return rt_bool_t RT_TRUE 已初始化，RT_FALSE 未初始化
 */
rt_bool_t initSyncIsReady(init_sync_id_t id);

/**
 * @brief 获取模块名称（用于日志）
 * 
 * @param id 模块标识符
 * @return const char* 模块名称字符串
 */
const char* initSyncGetName(init_sync_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* INIT_SYNC_H__ */

