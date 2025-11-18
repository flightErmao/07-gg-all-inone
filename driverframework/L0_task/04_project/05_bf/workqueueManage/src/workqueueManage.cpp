/**
 * @file workqueueManage.cpp
 * 
 * RT-Thread 工作队列管理器实现（C++ 版本）
 */

#include "workqueueManage.hpp"

#include <rtthread.h>
#include <rtconfig.h>
#include <ipc/workqueue.h>

#ifndef CONFIG_PROJECT_BF_WORKQUEUE_NAME
#define CONFIG_PROJECT_BF_WORKQUEUE_NAME "wq_rate_ctrl"
#endif

#ifndef CONFIG_PROJECT_BF_WORKQUEUE_STACK_SIZE
#define CONFIG_PROJECT_BF_WORKQUEUE_STACK_SIZE 1536
#endif

#ifndef CONFIG_PROJECT_BF_WORKQUEUE_PRIORITY
#define CONFIG_PROJECT_BF_WORKQUEUE_PRIORITY 16
#endif

// 从 Kconfig 获取工作队列名称（用于向后兼容的 C 接口）
#define WQ_RATE_CTRL_NAME CONFIG_PROJECT_BF_WORKQUEUE_NAME

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
    
    // 初始化默认工作队列（向后兼容）
    // 使用 Kconfig 中定义的名称为和参数
    getOrCreate(CONFIG_PROJECT_BF_WORKQUEUE_NAME, 
                CONFIG_PROJECT_BF_WORKQUEUE_STACK_SIZE,
                CONFIG_PROJECT_BF_WORKQUEUE_PRIORITY);
    
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
    
    // 如果不存在，创建新的工作队列
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
        // 如果未找到，尝试创建
        wq = getOrCreate(wq_name, CONFIG_PROJECT_BF_WORKQUEUE_STACK_SIZE, 
                         CONFIG_PROJECT_BF_WORKQUEUE_PRIORITY);
        if (wq == nullptr) {
            return -RT_ERROR;
        }
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

// 静态初始化器：当 PROJECT_BF_WORKQUEUE_MANAGE_EN 使能时自动初始化
// 使用 RT-Thread 的初始化机制
static int workqueue_manager_auto_init(void)
{
    bf_workqueue::WorkqueueManager& mgr = bf_workqueue::WorkqueueManager::instance();
    rt_err_t ret = mgr.init();
    if (ret != RT_EOK) {
        return -1;
    }
    return 0;
}
INIT_COMPONENT_EXPORT(workqueue_manager_auto_init);

#endif /* PROJECT_BF_WORKQUEUE_MANAGE_EN */

