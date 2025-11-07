/**
 * @file BMI270.hpp
 *
 * 适配 RT-Thread 的 BMI270 SPI 驱动，移除 PX4 依赖。
 */

#pragma once

#include "Bosch_BMI270_registers.hpp"
#include "spi_interface.hpp"

extern "C" {
#include <rtdevice.h>
#include <rtthread.h>

#include "taskImuPub_mcn.h"
#include "imu.h"
#include "sensorsTypes.h"
#include "timestamp.h"
}

#include <cstddef>
#include <cstdint>

namespace bmi270_px4
{

struct RuntimeConfig {
  const char *spi_bus_name;
  const char *spi_device_name;
  const char *cs_pin_name;
  const char *int_pin_name;
  uint32_t spi_max_hz;
  const char *imu_device_name;
  uint8_t fifo_watermark_samples;  // 默认 6~12 之间即可
};

class BMI270 {
 public:
  static BMI270 &instance();

  rt_err_t init(const RuntimeConfig &cfg);
  void RunImpl();

  int readRaw(uint8_t *buffer, rt_size_t size);
  bool initialized() const { return init_ok_; }

 private:
  BMI270();
  ~BMI270();

  enum class State : uint8_t {
    RESET = 0,
    WAIT_FOR_RESET,
    MICROCODE_LOAD,
    CONFIGURE,
    FIFO_READ,
    ERROR,
  };

  enum EventFlag : uint32_t {
    EVENT_DRDY = 1u << 0,
    EVENT_TIMER = 1u << 1,
  };

  struct SampleBuffer {
    uint8_t raw[14];
    sensorData_t sensor;
    bool valid;
  };

  void workerLoop();
  bool resetSensor();
  bool waitResetAndCheckId();
  bool loadMicrocode();
  bool configureSensor();
  bool configureInterrupt();
  void disableInterrupt();
  bool fifoCycle();
  void publishSample(const int16_t accel[3], const int16_t gyro[3], uint32_t timestamp_us);
  void updateSampleBuffer(const int16_t accel[3], const int16_t gyro[3], uint32_t timestamp_us);

  uint8_t regRead(Bosch_BMI270::Register reg);
  bool regWrite(Bosch_BMI270::Register reg, uint8_t value);
  uint16_t fifoLevel();
  void fifoReset();

  static void workerEntry(void *parameter);
  static void drdyIsr(void *parameter);
  static void timerCallback(void *parameter);

  RuntimeConfig config_;
  SpiInterface spi_;

  rt_thread_t worker_thread_;
  rt_timer_t watchdog_timer_;
  rt_event event_;
  rt_sem sample_sem_;
  rt_mutex sample_mutex_;

  State state_;
  bool init_ok_;
  bool use_interrupt_;
  bool event_inited_;
  bool sample_sem_inited_;
  bool sample_mutex_inited_;
  bool timer_started_;

  rt_base_t int_pin_;

  uint32_t fifo_interval_us_;
  uint32_t reset_timestamp_us_;
  uint8_t failure_count_;

  SampleBuffer latest_;

  // 禁止拷贝
  BMI270(const BMI270 &) = delete;
  BMI270 &operator=(const BMI270 &) = delete;
};

rt_err_t bmi270_px4_init_default();

}  // namespace bmi270_px4


