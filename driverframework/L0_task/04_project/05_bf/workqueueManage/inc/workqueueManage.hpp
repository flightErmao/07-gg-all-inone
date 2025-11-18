/**
 * @file workqueueManage.hpp
 * 
 * RT-Thread 工作队列管理器（C++ 版本）
 * 支持通过名称查找和管理多个工作队列实例
 */

#pragma once

#include <rtthread.h>
#include <ipc/workqueue.h>
#include <cstdint>
#include <cstring>

#ifdef __cplusplus

namespace bf_workqueue {

/**
 * @brief 工作队列管理器类
 * 支持通过名称查找和管理多个工作队列
 */
class WorkqueueManager {
public:
    /**
     * @brief 获取单例实例
     */
    static WorkqueueManager& instance();
    
    /**
     * @brief 初始化工作队列管理器
     * 如果 PROJECT_BF_WORKQUEUE_MANAGE_EN 使能，会自动调用
     */
    rt_err_t init();
    
    /**
     * @brief 创建或获取工作队列（通过名称）
     * @param name 工作队列名称
     * @param stack_size 栈大小（字节），如果为 0 使用默认值
     * @param priority 优先级，如果为 0 使用默认值
     * @return 工作队列指针，失败返回 RT_NULL
     */
    struct rt_workqueue* getOrCreate(const char* name, 
                                      uint32_t stack_size = 0, 
                                      uint8_t priority = 0);
    
    /**
     * @brief 通过名称查找工作队列
     * @param name 工作队列名称
     * @return 工作队列指针，未找到返回 RT_NULL
     */
    struct rt_workqueue* find(const char* name);
    
    /**
     * @brief 添加工作到工作队列
     * @param wq_name 工作队列名称
     * @param work 工作项指针
     * @return RT_EOK 成功，其他值失败
     */
    rt_err_t addWork(const char* wq_name, struct rt_work* work);
    
    /**
     * @brief 添加工作到工作队列（使用工作队列指针）
     * @param wq 工作队列指针
     * @param work 工作项指针
     * @return RT_EOK 成功，其他值失败
     */
    rt_err_t addWork(struct rt_workqueue* wq, struct rt_work* work);

private:
    WorkqueueManager();
    ~WorkqueueManager();
    
    // 禁用拷贝构造和赋值
    WorkqueueManager(const WorkqueueManager&) = delete;
    WorkqueueManager& operator=(const WorkqueueManager&) = delete;
    
    static WorkqueueManager* instance_;
    bool initialized_;
    
    // 默认参数
    static constexpr uint32_t DEFAULT_STACK_SIZE = 1536;
    static constexpr uint8_t DEFAULT_PRIORITY = 16;
    static constexpr size_t MAX_QUEUES = 8;  // 最大支持的工作队列数量
    
    // 工作队列条目
    struct WorkqueueEntry {
        const char* name;
        struct rt_workqueue* wq;
        bool in_use;
    };
    
    WorkqueueEntry queues_[MAX_QUEUES];
    
    /**
     * @brief 查找空闲的队列条目
     */
    WorkqueueEntry* findFreeEntry();
    
    /**
     * @brief 查找已使用的队列条目（通过名称）
     */
    WorkqueueEntry* findEntryByName(const char* name);
};

} // namespace bf_workqueue

// C 接口（兼容原有代码）
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化工作队列管理器（C 接口）
 * 用于向后兼容
 */
rt_err_t wq_workqueue_manage_init(void);

/**
 * @brief 添加工作到默认工作队列（C 接口）
 * @param work 工作项指针
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t wq_add_work(struct rt_work* work);

/**
 * @brief 获取默认工作队列（C 接口）
 * @return 工作队列指针，失败返回 RT_NULL
 */
struct rt_workqueue* wq_workqueue_get(void);

/**
 * @brief 通过名称获取工作队列（C 接口）
 * @param name 工作队列名称
 * @return 工作队列指针，失败返回 RT_NULL
 */
struct rt_workqueue* wq_workqueue_get_by_name(const char* name);

#ifdef __cplusplus
}
#endif

#endif /* __cplusplus */

