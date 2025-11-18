/**
 * @file workqueueManage.cpp
 * 
 * RT-Thread 工作队列管理器实现（C++ 版本）
 */

#include "workqueueManage.hpp"

#include <rtthread.h>
#include <rtconfig.h>
#include <ipc/workqueue.h>

// Kconfig 宏定义（如果没有定义，提供默认值）
// 注意：Kconfig 生成的宏名称没有 CONFIG_ 前缀，需要映射
#ifndef CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME
#ifdef PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME
#define CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME
#else
#define CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME "wq_rate_ctrl"
#endif
#endif

#ifndef CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE
#ifdef PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE
#define CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE
#else
#define CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE 1536
#endif
#endif

#ifndef CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY
#ifdef PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY
#define CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY
#else
#define CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY 16
#endif
#endif

// 向后兼容的宏定义
#define CONFIG_PROJECT_BF_WORKQUEUE_NAME CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME
#define CONFIG_PROJECT_BF_WORKQUEUE_STACK_SIZE CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE
#define CONFIG_PROJECT_BF_WORKQUEUE_PRIORITY CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY
#define WQ_RATE_CTRL_NAME CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME

namespace bf_workqueue {

WorkqueueManager* WorkqueueManager::instance_ = nullptr;

WorkqueueManager::WorkqueueManager()
    : initialized_(false)
{
    // 初始化队列条目
    for (size_t i = 0; i < MAX_QUEUES; i++) {
        queues_[i].name = nullptr;
        queues_[i].wq = nullptr;
        queues_[i].in_use = false;
    }
}

WorkqueueManager::~WorkqueueManager()
{
    // 清理所有工作队列（如果需要）
    // RT-Thread 的工作队列会在系统退出时自动清理
}

WorkqueueManager& WorkqueueManager::instance()
{
    if (instance_ == nullptr) {
        // 使用静态存储，确保单例生命周期
        static WorkqueueManager manager;
        instance_ = &manager;
    }
    return *instance_;
}

rt_err_t WorkqueueManager::init()
{
    if (initialized_) {
        return RT_EOK;
    }
    
    initialized_ = true;
    // 不再自动创建工作队列，工作队列由 INIT_ENV_EXPORT 自动创建
    
    return RT_EOK;
}

WorkqueueManager::WorkqueueEntry* WorkqueueManager::findFreeEntry()
{
    for (size_t i = 0; i < MAX_QUEUES; i++) {
        if (!queues_[i].in_use) {
            return &queues_[i];
        }
    }
    return nullptr;
}

WorkqueueManager::WorkqueueEntry* WorkqueueManager::findEntryByName(const char* name)
{
    if (name == nullptr) {
        return nullptr;
    }
    
    for (size_t i = 0; i < MAX_QUEUES; i++) {
        if (queues_[i].in_use && queues_[i].name != nullptr) {
            if (rt_strcmp(queues_[i].name, name) == 0) {
                return &queues_[i];
            }
        }
    }
    return nullptr;
}

rt_err_t WorkqueueManager::registerWorkqueue(const char* name, struct rt_workqueue* wq)
{
    if (name == nullptr || wq == nullptr) {
        return -RT_EINVAL;
    }
    
    // 检查是否已存在同名的工作队列
    WorkqueueEntry* entry = findEntryByName(name);
    if (entry != nullptr) {
        // 已存在同名工作队列
        return -RT_EBUSY;
    }
    
    // 查找空闲条目
    entry = findFreeEntry();
    if (entry == nullptr) {
        // 没有空闲的条目
        return -RT_ENOSPC;
    }
    
    // 注册工作队列
    entry->name = name;  // 注意：这里假设 name 是静态字符串常量
    entry->wq = wq;
    entry->in_use = true;
    
    return RT_EOK;
}

struct rt_workqueue* WorkqueueManager::getOrCreate(const char* name, 
                                                     uint32_t stack_size, 
                                                     uint8_t priority)
{
    if (name == nullptr) {
        return nullptr;
    }
    
    // 先查找是否已存在
    WorkqueueEntry* entry = findEntryByName(name);
    if (entry != nullptr && entry->wq != nullptr) {
        return entry->wq;
    }
    
    // 如果不存在，创建新的工作队列（仅用于向后兼容，不建议使用）
    entry = findFreeEntry();
    if (entry == nullptr) {
        // 没有空闲的条目
        return nullptr;
    }
    
    // 使用默认值（如果未指定）
    if (stack_size == 0) {
        stack_size = DEFAULT_STACK_SIZE;
    }
    if (priority == 0) {
        priority = DEFAULT_PRIORITY;
    }
    
    // 创建工作队列
    struct rt_workqueue* wq = rt_workqueue_create(name, stack_size, priority);
    if (wq == nullptr) {
        return nullptr;
    }
    
    // 保存到条目中
    entry->name = name;  // 注意：这里假设 name 是静态字符串常量
    entry->wq = wq;
    entry->in_use = true;
    
    return wq;
}

struct rt_workqueue* WorkqueueManager::find(const char* name)
{
    WorkqueueEntry* entry = findEntryByName(name);
    if (entry != nullptr) {
        return entry->wq;
    }
    return nullptr;
}

rt_err_t WorkqueueManager::addWork(const char* wq_name, struct rt_work* work)
{
    if (wq_name == nullptr || work == nullptr) {
        return -RT_EINVAL;
    }
    
    struct rt_workqueue* wq = find(wq_name);
    if (wq == nullptr) {
        // 工作队列未找到，只能通过 find 查找，不能自动创建
        return -RT_ENOENT;
    }
    
    return rt_workqueue_dowork(wq, work);
}

rt_err_t WorkqueueManager::addWork(struct rt_workqueue* wq, struct rt_work* work)
{
    if (wq == nullptr || work == nullptr) {
        return -RT_EINVAL;
    }
    
    return rt_workqueue_dowork(wq, work);
}

} // namespace bf_workqueue

// C 接口实现
extern "C" {

rt_err_t wq_workqueue_manage_init(void)
{
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    return mgr.init();
}

rt_err_t wq_add_work(struct rt_work* work)
{
    if (work == nullptr) {
        return -RT_EINVAL;
    }
    
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    mgr.init();  // 确保已初始化
    
    return mgr.addWork(WQ_RATE_CTRL_NAME, work);
}

struct rt_workqueue* wq_workqueue_get(void)
{
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    mgr.init();  // 确保已初始化
    
    return mgr.find(WQ_RATE_CTRL_NAME);
}

struct rt_workqueue* wq_workqueue_get_by_name(const char* name)
{
    if (name == nullptr) {
        return nullptr;
    }
    
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    mgr.init();  // 确保已初始化
    
    return mgr.find(name);
}

} // extern "C"

#ifdef PROJECT_BF_WORKQUEUE_MANAGE_EN

// 工作队列管理器自动初始化（INIT_ENV_EXPORT）
static int workqueue_manager_auto_init(void)
{
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    rt_err_t ret = mgr.init();
    if (ret != RT_EOK) {
        return -1;
    }
    return 0;
}
INIT_ENV_EXPORT(workqueue_manager_auto_init);

// 速率控制工作队列自动创建（INIT_ENV_EXPORT）
#ifdef PROJECT_BF_WORKQUEUE_RATE_CTRL_EN
static int workqueue_rate_ctrl_init(void)
{
    const char* name = CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_NAME;
    uint32_t stack_size = CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_STACK_SIZE;
    uint8_t priority = CONFIG_PROJECT_BF_WORKQUEUE_RATE_CTRL_PRIORITY;
    
    // 创建工作队列
    struct rt_workqueue* wq = rt_workqueue_create(name, stack_size, priority);
    if (wq == nullptr) {
        rt_kprintf("[WQ] Failed to create workqueue '%s'\n", name);
        return -1;
    }
    
    // 注册到管理器
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    mgr.init();  // 确保管理器已初始化
    rt_err_t ret = mgr.registerWorkqueue(name, wq);
    if (ret != RT_EOK) {
        rt_kprintf("[WQ] Failed to register workqueue '%s': %d\n", name, ret);
        return -1;
    }
    
    rt_kprintf("[WQ] Created workqueue '%s' (stack=%u, priority=%u)\n", name, stack_size, priority);
    return 0;
}
INIT_ENV_EXPORT(workqueue_rate_ctrl_init);
#endif /* PROJECT_BF_WORKQUEUE_RATE_CTRL_EN */

// 可以在这里添加更多工作队列的自动初始化函数
// 例如：
// #ifdef PROJECT_BF_WORKQUEUE_SENSOR_EN
// static int workqueue_sensor_init(void) { ... }
// INIT_ENV_EXPORT(workqueue_sensor_init);
// #endif

#endif /* PROJECT_BF_WORKQUEUE_MANAGE_EN */

