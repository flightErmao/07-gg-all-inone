/**
 * @file testThread_gyro.cpp
 * 
 * 测试线程实现：订阅 imu_raw MCN 节点并记录到 mlog_gyro
 */

#include "testThread_gyro.hpp"
#include "../mlog/inc/mlog_gyro.hpp"
#include "../imu270_pub/inc/imu_raw_msg.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "test_thread"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
#include <stdlib.h>  // for rand()
#include "timestamp.h"
#include "debugPin.h"
#include "uMCN.h"
}

namespace bf_testthread {

// 线程配置（从 Kconfig 获取，如果没有定义则使用默认值）
#ifndef CONFIG_PROJECT_BF_TEST_THREAD_STACK_SIZE
#define CONFIG_PROJECT_BF_TEST_THREAD_STACK_SIZE 2048
#endif

#ifndef CONFIG_PROJECT_BF_TEST_THREAD_PRIORITY
#define CONFIG_PROJECT_BF_TEST_THREAD_PRIORITY 10
#endif

#ifndef CONFIG_PROJECT_BF_TEST_THREAD_TIMESLICE
#define CONFIG_PROJECT_BF_TEST_THREAD_TIMESLICE 5
#endif

TestThreadGyro& TestThreadGyro::instance() {
    static TestThreadGyro instance_obj;
    return instance_obj;
}

TestThreadGyro::TestThreadGyro()
    : thread_(RT_NULL),
      thread_inited_(false),
      imu_event_(RT_NULL),
      imu_node_(RT_NULL) {
    rt_memset(&thread_obj_, 0, sizeof(thread_obj_));
    rt_memset(thread_stack_, 0, sizeof(thread_stack_));
}

TestThreadGyro::~TestThreadGyro() {
    // 清理工作
    if (thread_inited_ && thread_ != RT_NULL) {
        rt_thread_delete(thread_);
        thread_ = RT_NULL;
        thread_inited_ = false;
    }
    
    if (imu_node_ != RT_NULL) {
        mcn_unsubscribe(MCN_HUB(imu_raw), imu_node_);
        imu_node_ = RT_NULL;
    }
    
    if (imu_event_ != RT_NULL) {
        rt_sem_delete(imu_event_);
        imu_event_ = RT_NULL;
    }
}

rt_err_t TestThreadGyro::init() {
    if (thread_inited_) {
        LOG_W("TestThreadGyro already initialized");
        return RT_EOK;
    }
    
    // 创建 MCN 事件信号量（用于 mcn_poll_sync）
    if (imu_event_ == RT_NULL) {
        imu_event_ = rt_sem_create("imu_evt", 0, RT_IPC_FLAG_FIFO);
        if (imu_event_ == RT_NULL) {
            LOG_E("create imu event semaphore failed");
            return -RT_ERROR;
        }
    }
    
    // 订阅 imu_raw MCN 节点（传入 event 用于同步等待）
    imu_node_ = mcn_subscribe(MCN_HUB(imu_raw), imu_event_, RT_NULL);
    if (imu_node_ == RT_NULL) {
        LOG_E("subscribe imu_raw topic failed");
        if (imu_event_ != RT_NULL) {
            rt_sem_delete(imu_event_);
            imu_event_ = RT_NULL;
        }
        return -RT_ERROR;
    }
    LOG_I("Subscribed to imu_raw MCN topic");
    
    // 初始化 mlog_gyro（使用单例）
    bf_mlog::MlogGyro* mlog_gyro = bf_mlog::MlogGyro::getInstance();
    mlog_gyro->init();
    mlog_gyro->setParamEnabled(true);  // 使能记录
    LOG_I("MlogGyro initialized and enabled");
    
    // 创建静态线程
    rt_err_t err = rt_thread_init(
        &thread_obj_,
        "test_gyro",
        TestThreadGyro::threadEntry,
        this,
        thread_stack_,
        CONFIG_PROJECT_BF_TEST_THREAD_STACK_SIZE,
        CONFIG_PROJECT_BF_TEST_THREAD_PRIORITY,
        CONFIG_PROJECT_BF_TEST_THREAD_TIMESLICE
    );
    
    if (err != RT_EOK) {
        LOG_E("TestThreadGyro thread init failed: %d", err);
        if (imu_node_ != RT_NULL) {
            mcn_unsubscribe(MCN_HUB(imu_raw), imu_node_);
            imu_node_ = RT_NULL;
        }
        if (imu_event_ != RT_NULL) {
            rt_sem_delete(imu_event_);
            imu_event_ = RT_NULL;
        }
        return err;
    }
    
    thread_ = &thread_obj_;
    thread_inited_ = true;
    
    // 启动线程
    rt_thread_startup(thread_);
    LOG_I("TestThreadGyro thread started");
    
    return RT_EOK;
}

void TestThreadGyro::threadEntry(void* parameter) {
    if (parameter == RT_NULL) {
        return;
    }
    
    TestThreadGyro* instance = static_cast<TestThreadGyro*>(parameter);
    instance->threadLoop();
}

void TestThreadGyro::threadLoop() {
    imu_raw_msg_t imu_data;
    
    LOG_I("TestThreadGyro thread loop started");
    
    while (true) {
        // 阻塞等待 MCN 发布
        if (mcn_poll_sync(imu_node_, RT_WAITING_FOREVER) == RT_TRUE) {
            // 复制数据
            if (mcn_copy(MCN_HUB(imu_raw), imu_node_, &imu_data) == RT_EOK) {
                // 生成假数据用于 mlog 记录
                // 使用随机数生成假数据（范围：-2000 到 +2000 dps）
                float gyro_raw[3];
                float gyro_filtered[3];
                
                for (int i = 0; i < 3; i++) {
                    gyro_raw[i] = ((float)rand() / RAND_MAX) * 4000.0f - 2000.0f;
                    gyro_filtered[i] = ((float)rand() / RAND_MAX) * 4000.0f - 2000.0f;
                }
                
                uint32_t timestamp = timestamp_micros();
                
                // 调用 mlog_gyro 记录数据（使用 debugpin 监测）
#ifdef PROJECT_BF_TEST_THREAD_DEBUG_PIN_EN
                DEBUG_PIN_DEBUG0_HIGH();
#endif
                bf_mlog::MlogGyro::getInstance()->pushGyroData(
                    imu_data.seq,
                    timestamp,
                    gyro_raw,
                    gyro_filtered
                );
#ifdef PROJECT_BF_TEST_THREAD_DEBUG_PIN_EN
                DEBUG_PIN_DEBUG0_LOW();
#endif
            }
        }
    }
}

}  // namespace bf_testthread

// RT-Thread 自动初始化包装函数
#ifdef PROJECT_BF_TEST_THREAD_EN
extern "C" {
static int test_thread_gyro_init_wrapper(void) {
    bf_testthread::TestThreadGyro& instance = bf_testthread::TestThreadGyro::instance();
    rt_err_t ret = instance.init();
    if (ret == RT_EOK) {
        LOG_I("TestThreadGyro auto-init success");
    } else {
        LOG_E("TestThreadGyro auto-init failed: %d", ret);
    }
    return (int)ret;
}
INIT_APP_EXPORT(test_thread_gyro_init_wrapper);
}
#endif

