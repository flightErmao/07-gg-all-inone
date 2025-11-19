/**
 * @file testThread_gyro.hpp
 * 
 * 测试线程：订阅 imu_raw MCN 节点并记录到 mlog_gyro
 */

#pragma once

extern "C" {
#include <rtdef.h>
#include <rtthread.h>
#include "uMCN.h"
}

namespace bf_testthread {

/**
 * @brief 测试线程类
 * 订阅 imu_raw MCN 节点，阻塞等待数据，然后调用 mlog_gyro 记录
 */
class TestThreadGyro {
public:
    /**
     * @brief 获取单例实例
     */
    static TestThreadGyro& instance();
    
    /**
     * @brief 初始化测试线程
     * @return RT_EOK 成功，其他值失败
     */
    rt_err_t init();
    
    /**
     * @brief 线程入口函数（静态）
     */
    static void threadEntry(void* parameter);

private:
    TestThreadGyro();
    ~TestThreadGyro();
    
    TestThreadGyro(const TestThreadGyro&) = delete;
    TestThreadGyro& operator=(const TestThreadGyro&) = delete;
    
    /**
     * @brief 线程主循环
     */
    void threadLoop();
    
    // 线程相关
    rt_thread_t thread_;
    struct rt_thread thread_obj_;
    // 栈大小：使用 Kconfig 配置，如果没有定义则使用默认值 2048
    // 注意：数组大小必须是编译时常量，所以这里使用一个足够大的默认值
    // 实际使用的栈大小由 rt_thread_init 的参数决定
    rt_uint8_t thread_stack_[4096];  // 最大栈大小，实际使用大小由 Kconfig 配置
    bool thread_inited_;
    
    // MCN 订阅相关
    rt_sem_t imu_event_;      // MCN 事件信号量（用于 mcn_poll_sync）
    McnNode_t imu_node_;      // MCN 订阅节点
};

}  // namespace bf_testthread

