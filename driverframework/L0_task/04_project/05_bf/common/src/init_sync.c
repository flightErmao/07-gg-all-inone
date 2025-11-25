#include "init_sync.h"

#include <rtthread.h>
#include <rtconfig.h>
#define LOG_TAG "init_sync"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

// 初始化同步事件组
static rt_event_t init_sync_event_ = RT_NULL;
static rt_bool_t init_sync_inited_ = RT_FALSE;

// 模块名称数组（用于日志）
static const char* init_sync_names_[] = {
    "BMI270",
    "GyroFilter",
    "RC",
    "PID",
    "Motor",
};

rt_err_t initSyncInit(void) {
  if (init_sync_inited_) {
    LOG_W("InitSync already initialized");
    return RT_EOK;
  }

  // 创建初始化同步事件组
  init_sync_event_ = rt_event_create("init_sync", RT_IPC_FLAG_FIFO);
  if (init_sync_event_ == RT_NULL) {
    LOG_E("Failed to create init sync event");
    return -RT_ENOMEM;
  }

  init_sync_inited_ = RT_TRUE;
  LOG_I("InitSync initialized successfully");
  return RT_EOK;
}

rt_err_t initSyncNotify(init_sync_id_t id) {
  if (!init_sync_inited_) {
    LOG_E("InitSync not initialized when %s tries to notify", init_sync_names_[id]);
    return -RT_ERROR;
  }

  if (id >= INIT_SYNC_COUNT) {
    LOG_E("Invalid init sync id: %d", id);
    return -RT_EINVAL;
  }

  if (init_sync_event_ == RT_NULL) {
    LOG_E("Init sync event is NULL when %s tries to notify", init_sync_names_[id]);
    return -RT_ERROR;
  }

  // 设置对应的事件位（使用 1 << id 作为事件标志）
  rt_uint32_t event_flag = 1UL << id;
  rt_err_t ret = rt_event_send(init_sync_event_, event_flag);
  if (ret == RT_EOK) {
    LOG_I("%s initialization completed (event_flag=0x%08X)", init_sync_names_[id], event_flag);
  } else {
    LOG_E("Failed to send init sync event for %s: %d", init_sync_names_[id], ret);
  }

  return ret;
}

rt_err_t initSyncWait(init_sync_id_t id, rt_int32_t timeout_ms) {
  if (!init_sync_inited_) {
    LOG_E("InitSync not initialized when waiting for %s", init_sync_names_[id]);
    return -RT_ERROR;
  }

  if (id >= INIT_SYNC_COUNT) {
    LOG_E("Invalid init sync id: %d", id);
    return -RT_EINVAL;
  }

  if (init_sync_event_ == RT_NULL) {
    LOG_E("Init sync event is NULL when waiting for %s", init_sync_names_[id]);
    return -RT_ERROR;
  }

  // 检查是否已经就绪（非阻塞检查，不清除事件位）
  rt_uint32_t event_flag = 1UL << id;
  rt_uint32_t recved = 0;
  rt_err_t check_ret = rt_event_recv(init_sync_event_, event_flag, RT_EVENT_FLAG_OR, 0, &recved);
  if (check_ret == RT_EOK && (recved & event_flag)) {
    // 已经就绪，立即返回（不清除事件位，以便其他等待者也能收到）
    LOG_I("%s is already ready (event_flag=0x%08X, recved=0x%08X)", init_sync_names_[id], event_flag, recved);
    return RT_EOK;
  }

  // 等待初始化完成（使用 RT_EVENT_FLAG_CLEAR 清除事件位，确保只接收一次）
  LOG_I("Waiting for %s initialization... (event_flag=0x%08X)", init_sync_names_[id], event_flag);
  rt_err_t ret = rt_event_recv(init_sync_event_, event_flag, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout_ms, &recved);
  
  if (ret == RT_EOK) {
    LOG_I("%s is ready (recved=0x%08X)", init_sync_names_[id], recved);
  } else if (ret == -RT_ETIMEOUT) {
    // 超时后再次检查事件状态（用于调试）
    rt_uint32_t current_events = 0;
    rt_event_control(init_sync_event_, RT_IPC_CMD_GET_STATE, &current_events);
    LOG_W("%s initialization timeout after %d ms (current_events=0x%08X, expected=0x%08X)", 
          init_sync_names_[id], timeout_ms, current_events, event_flag);
  } else {
    LOG_E("Failed to wait for %s initialization: %d", init_sync_names_[id], ret);
  }

  return ret;
}

rt_bool_t initSyncIsReady(init_sync_id_t id) {
  if (!init_sync_inited_ || id >= INIT_SYNC_COUNT || init_sync_event_ == RT_NULL) {
    return RT_FALSE;
  }

  // 检查事件位（非阻塞）
  rt_uint32_t event_flag = 1UL << id;
  rt_uint32_t recved = 0;
  if (rt_event_recv(init_sync_event_, event_flag, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved) == RT_EOK) {
    // 已经就绪，恢复事件状态（重新发送事件）
    rt_event_send(init_sync_event_, event_flag);
    return RT_TRUE;
  }

  return RT_FALSE;
}

const char* initSyncGetName(init_sync_id_t id) {
  if (id >= INIT_SYNC_COUNT) {
    return "Unknown";
  }
  return init_sync_names_[id];
}

// 自动初始化：在系统启动时初始化同步机制
static int init_sync_auto_init(void) {
  rt_err_t ret = initSyncInit();
  if (ret != RT_EOK) {
    LOG_E("InitSync auto-init failed: %d", ret);
    return (int)ret;
  }
  LOG_I("InitSync auto-init completed");
  return 0;
}
INIT_DEVICE_EXPORT(init_sync_auto_init);  // 使用 INIT_DEVICE_EXPORT 确保在其他模块之前初始化

