/**
 * @file accgyro_spi_bmi270.hpp
 *
 * Betaflight 风格 BMI270 驱动的 RT-Thread C++ 封装（不使用 FIFO，仅中断+寄存器读取）。
 */

#pragma once

extern "C" {
#include <rtdef.h>
#include <rtdevice.h>
#include <rtthread.h>
#include "uMCN.h"
}

#include <cstdint>
#include "spi_interface.hpp"
#include "imu_raw_msg.h"

namespace bf_bmi270 {

struct RuntimeConfig {
  const char *spi_bus_name;
  const char *spi_device_name;
  const char *cs_pin_name;
  const char *int_pin_name;
  uint32_t spi_max_hz;
};

class BMI270 {
 public:
  static BMI270 &instance();

  rt_err_t init(const RuntimeConfig &cfg);

  /** 是否已经完成初始化并开始工作 */
  bool initialized() const { return init_ok_; }

  /* 工作线程配置（静态栈） */
  static constexpr rt_uint16_t THREAD_STACK_SIZE = 2048;
  static constexpr rt_uint8_t THREAD_PRIORITY = 12;
  static constexpr rt_uint8_t THREAD_TIMESLICE = 5;

 private:
  BMI270();
  ~BMI270();

  BMI270(const BMI270 &) = delete;
  BMI270 &operator=(const BMI270 &) = delete;

  bool resetSensor();
  bool loadConfigFile();
  bool configureSensor();
  bool configureInterrupt();
  void disableInterrupt();

  void workerLoop();

  bool readAccelGyro(int16_t acc[3], int16_t gyro[3]);
  void publishImu(const int16_t acc[3], const int16_t gyro[3]);

  uint8_t regRead(uint8_t reg);
  void regWrite(uint8_t reg, uint8_t value, unsigned delayMs = 0);

  static void drdyIsr(void *parameter);
  static void workerEntry(void *parameter);

 private:
  RuntimeConfig cfg_;

  SpiInterface spi_;
  bool spi_inited_;
  bool init_ok_;

  rt_base_t int_pin_;

  /* 中断事件 + 工作线程（静态创建） */
  rt_event event_;
  bool event_inited_;

  rt_thread_t worker_thread_;
  struct rt_thread worker_thread_obj_;
  rt_uint8_t worker_stack_[THREAD_STACK_SIZE];
  bool worker_inited_;
};

/** 使用 Kconfig 默认参数做一次全局初始化，并开始发布 imu_raw MCN 数据 */
rt_err_t bf_bmi270_init_default();

}  // namespace bf_bmi270


