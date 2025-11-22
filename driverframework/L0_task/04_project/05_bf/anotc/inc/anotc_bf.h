#ifndef ANOTC_BF_H__
#define ANOTC_BF_H__

#include <rtthread.h>
#include <cstdint>
#include <cstring>

extern "C" {
#include "protocolAtkpInterface.h"
}

// Forward declarations
typedef void (*sensor_data_send_func_t)(uint16_t count_ms);

// C wrapper function declaration
extern "C" {
void anotcMqStash(atkp_t* p);
void anotcTelemAddSensorFunc(sensor_data_send_func_t func);
}

class AnotcBf {
 public:
  // Singleton pattern
  static AnotcBf& instance();

  AnotcBf();
  ~AnotcBf();

  rt_err_t init();
  
  // Queue packet for sending
  void stashPacket(atkp_t* p);
  
  // Add sensor function to periodic call list
  void addSensorFunc(sensor_data_send_func_t func);

 private:
  AnotcBf(const AnotcBf&) = delete;
  AnotcBf& operator=(const AnotcBf&) = delete;

  // Thread entry points
  static void mqRecThreadEntry(void* parameter);
  static void mqSendThreadEntry(void* parameter);
  
  // Initialize message queue
  rt_err_t initMessageQueue();
  
  // Initialize device
  rt_err_t initDevice();

  // Thread handles
  rt_thread_t mq_rec_thread_;
  struct rt_thread mq_rec_thread_obj_;
  rt_uint8_t* mq_rec_thread_stack_;
  bool mq_rec_thread_inited_;

  rt_thread_t mq_send_thread_;
  struct rt_thread mq_send_thread_obj_;
  rt_uint8_t* mq_send_thread_stack_;
  bool mq_send_thread_inited_;

  // Message queue
  struct rt_messagequeue device_send_mq_;
  rt_uint8_t* msg_pool_;

  // Sensor function list
  static constexpr int MAX_SENSOR_FUNCS = 10;
  sensor_data_send_func_t sensor_func_list_[MAX_SENSOR_FUNCS];
  uint8_t sensor_func_count_;

};

#endif /* ANOTC_BF_H__ */

