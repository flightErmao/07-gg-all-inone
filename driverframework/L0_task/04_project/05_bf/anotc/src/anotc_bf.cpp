#include "anotc_bf.h"

extern "C" {
#include <rtthread.h>
#include <rtconfig.h>
#include <rtdevice.h>
#include "deviceManager.h"
#define LOG_TAG "anotc_bf"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>
}

#include <cstring>

// Constants
#define MAX_SENSOR_FUNCS 10
#define MSG_NUM 30
#define POOL_SIZE_BYTE (sizeof(atkp_t) * MSG_NUM)

// Singleton instance
AnotcBf& AnotcBf::instance() {
  static AnotcBf instance_obj;
  return instance_obj;
}

// Constructor
AnotcBf::AnotcBf()
    : mq_rec_thread_(RT_NULL),
      mq_rec_thread_stack_(nullptr),
      mq_rec_thread_inited_(false),
      mq_send_thread_(RT_NULL),
      mq_send_thread_stack_(nullptr),
      mq_send_thread_inited_(false),
      msg_pool_(nullptr),
      sensor_func_count_(0) {
  std::memset(&mq_rec_thread_obj_, 0, sizeof(mq_rec_thread_obj_));
  std::memset(&mq_send_thread_obj_, 0, sizeof(mq_send_thread_obj_));
  std::memset(&device_send_mq_, 0, sizeof(device_send_mq_));
  std::memset(sensor_func_list_, 0, sizeof(sensor_func_list_));
}

// Destructor
AnotcBf::~AnotcBf() {
  if (mq_rec_thread_inited_ && mq_rec_thread_ != RT_NULL) {
    rt_thread_delete(mq_rec_thread_);
  }
  if (mq_rec_thread_stack_ != nullptr) {
    delete[] mq_rec_thread_stack_;
  }

  if (mq_send_thread_inited_ && mq_send_thread_ != RT_NULL) {
    rt_thread_delete(mq_send_thread_);
  }
  if (mq_send_thread_stack_ != nullptr) {
    delete[] mq_send_thread_stack_;
  }

  if (msg_pool_ != nullptr) {
    delete[] msg_pool_;
  }
}

rt_err_t AnotcBf::init() {
  // Initialize message queue
  rt_err_t ret = initMessageQueue();
  if (ret != RT_EOK) {
    LOG_E("Message queue init failed");
    return ret;
  }

  // Initialize device
  ret = initDevice();
  if (ret != RT_EOK) {
    LOG_E("Device init failed");
    return ret;
  }

  // Allocate thread stacks
  size_t rec_stack_size = PROJECT_BF_ANOTC_REC_THREAD_STACK_SIZE;
  mq_rec_thread_stack_ = new rt_uint8_t[rec_stack_size];
  if (mq_rec_thread_stack_ == nullptr) {
    LOG_E("Failed to allocate rec thread stack");
    return -RT_ENOMEM;
  }
  std::memset(mq_rec_thread_stack_, 0, rec_stack_size);

  size_t send_stack_size = PROJECT_BF_ANOTC_SEND_THREAD_STACK_SIZE;
  mq_send_thread_stack_ = new rt_uint8_t[send_stack_size];
  if (mq_send_thread_stack_ == nullptr) {
    LOG_E("Failed to allocate send thread stack");
    delete[] mq_rec_thread_stack_;
    mq_rec_thread_stack_ = nullptr;
    return -RT_ENOMEM;
  }
  std::memset(mq_send_thread_stack_, 0, send_stack_size);

  // Initialize rec thread
  ret = rt_thread_init(&mq_rec_thread_obj_, "anotc_rec", mqRecThreadEntry, this, mq_rec_thread_stack_, rec_stack_size,
                       PROJECT_BF_ANOTC_REC_THREAD_PRIORITY, PROJECT_BF_ANOTC_REC_THREAD_TIMESLICE);
  if (ret != RT_EOK) {
    LOG_E("Rec thread init failed: %d", ret);
    delete[] mq_rec_thread_stack_;
    mq_rec_thread_stack_ = nullptr;
    delete[] mq_send_thread_stack_;
    mq_send_thread_stack_ = nullptr;
    return ret;
  }
  mq_rec_thread_ = &mq_rec_thread_obj_;
  mq_rec_thread_inited_ = true;

  // Initialize send thread
  ret = rt_thread_init(&mq_send_thread_obj_, "anotc_send", mqSendThreadEntry, this, mq_send_thread_stack_, send_stack_size,
                       PROJECT_BF_ANOTC_SEND_THREAD_PRIORITY, PROJECT_BF_ANOTC_SEND_THREAD_TIMESLICE);
  if (ret != RT_EOK) {
    LOG_E("Send thread init failed: %d", ret);
    if (mq_rec_thread_inited_ && mq_rec_thread_ != RT_NULL) {
      rt_thread_delete(mq_rec_thread_);
      mq_rec_thread_inited_ = false;
    }
    delete[] mq_rec_thread_stack_;
    mq_rec_thread_stack_ = nullptr;
    delete[] mq_send_thread_stack_;
    mq_send_thread_stack_ = nullptr;
    return ret;
  }
  mq_send_thread_ = &mq_send_thread_obj_;
  mq_send_thread_inited_ = true;

  // Start threads
  ret = rt_thread_startup(mq_rec_thread_);
  if (ret != RT_EOK) {
    LOG_E("Rec thread startup failed: %d", ret);
    mq_rec_thread_inited_ = false;
    return ret;
  }

  ret = rt_thread_startup(mq_send_thread_);
  if (ret != RT_EOK) {
    LOG_E("Send thread startup failed: %d", ret);
    mq_send_thread_inited_ = false;
    return ret;
  }

  LOG_I("AnotcBf initialized successfully");
  return RT_EOK;
}

rt_err_t AnotcBf::initMessageQueue() {
  msg_pool_ = new rt_uint8_t[POOL_SIZE_BYTE];
  if (msg_pool_ == nullptr) {
    LOG_E("Failed to allocate message pool");
    return -RT_ENOMEM;
  }

  rt_err_t result = rt_mq_init(&device_send_mq_, "anotc_mq", msg_pool_, sizeof(atkp_t), POOL_SIZE_BYTE, RT_IPC_FLAG_PRIO);
  if (result != RT_EOK) {
    LOG_E("Message queue init failed: %d", result);
    delete[] msg_pool_;
    msg_pool_ = nullptr;
    return result;
  }

  LOG_I("Message queue initialized");
  return RT_EOK;
}

rt_err_t AnotcBf::initDevice() {
  char device_name[RT_NAME_MAX];
  
#ifndef PROJECT_BF_ANOTC_DEVICE_DEFAULT
  const char* default_name = "uart2";
#else
  const char* default_name = PROJECT_BF_ANOTC_DEVICE_DEFAULT;
#endif
  
  rt_strncpy(device_name, default_name, RT_NAME_MAX);
  
  rt_err_t ret = uartDevAnotcInit(device_name);
  if (ret != RT_EOK) {
    LOG_E("Device init failed: %d", ret);
    return ret;
  }

  // dev_anotc_telem_ is a global variable set by uartDevAnotcInit
  // dev_anotc_telem_ is a global variable set by uartDevAnotcInit
  LOG_I("Device %s initialized", device_name);
  return RT_EOK;
}

void AnotcBf::stashPacket(atkp_t* p) {
  if (p == nullptr) {
    return;
  }

  rt_err_t result = rt_mq_send(&device_send_mq_, p, sizeof(atkp_t));
  if (result != RT_EOK) {
    LOG_W("Failed to send packet to queue: %d", result);
  }
}

void AnotcBf::addSensorFunc(sensor_data_send_func_t func) {
  if (func == nullptr) {
    return;
  }

  for (uint8_t i = 0; i < sensor_func_count_; i++) {
    if (sensor_func_list_[i] == func) {
      return;  // Already registered
    }
  }

  if (sensor_func_count_ < MAX_SENSOR_FUNCS) {
    sensor_func_list_[sensor_func_count_++] = func;
    LOG_I("Added sensor func, total: %d", sensor_func_count_);
  } else {
    LOG_W("Sensor func list full!");
  }
}

// Thread entry point - Receive and forward packets
void AnotcBf::mqRecThreadEntry(void* parameter) {
  AnotcBf* instance = static_cast<AnotcBf*>(parameter);
  if (instance == nullptr) {
    return;
  }

  LOG_I("Anotc rec thread started");

  atkp_t msg_temp;
  while (true) {
#if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 1))
    if (rt_mq_recv(&instance->device_send_mq_, &msg_temp, sizeof(msg_temp), RT_WAITING_FOREVER) > 0)
#else
    if (rt_mq_recv(&instance->device_send_mq_, &msg_temp, sizeof(msg_temp), RT_WAITING_FOREVER) == RT_EOK)
#endif
    {
      anotcDeviceSendDirect(&msg_temp);
    }
  }
}

// Thread entry point - Periodic sensor data sender
void AnotcBf::mqSendThreadEntry(void* parameter) {
  AnotcBf* instance = static_cast<AnotcBf*>(parameter);
  if (instance == nullptr) {
    return;
  }

  LOG_I("Anotc send thread started");

  uint16_t count_ms = 0;
  while (true) {
    for (uint8_t i = 0; i < instance->sensor_func_count_; i++) {
      if (instance->sensor_func_list_[i] != nullptr) {
        instance->sensor_func_list_[i](count_ms);
      }
    }
    count_ms++;
    rt_thread_mdelay(1);
  }
}

// RT-Thread auto initialization wrapper
#ifdef PROJECT_BF_ANOTC_EN
extern "C" {
static int anotc_bf_init_wrapper(void) {
  AnotcBf& instance = AnotcBf::instance();
  rt_err_t ret = instance.init();
  if (ret == RT_EOK) {
    LOG_I("AnotcBf auto-init success");
  } else {
    LOG_E("AnotcBf auto-init failed: %d", ret);
  }
  return (int)ret;
}
INIT_APP_EXPORT(anotc_bf_init_wrapper);
}
#endif

// C wrapper functions for backward compatibility
extern "C" {
void anotcMqStash(atkp_t* p) {
  AnotcBf& instance = AnotcBf::instance();
  instance.stashPacket(p);
}

void anotcTelemAddSensorFunc(sensor_data_send_func_t func) {
  AnotcBf& instance = AnotcBf::instance();
  instance.addSensorFunc(func);
}
}

