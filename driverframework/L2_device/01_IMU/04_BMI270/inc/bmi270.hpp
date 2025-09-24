#pragma once

#include <stdint.h>
#include <stddef.h>

#include "bmi270_regs.hpp"

class Bmi270 {
 public:
  typedef int (*ReadMultiRegFn)(uint8_t reg, uint8_t* buff, uint8_t len);
  typedef int (*WriteMultiRegFn)(uint8_t reg, uint8_t* buff, uint16_t len);
  typedef void (*DelayMsFn)(unsigned int ms);
  typedef void (*GpioCsControlFn)(bool state);

  static Bmi270& instance();

  Bmi270();
  ~Bmi270();

  int setIoFunctions(ReadMultiRegFn read_fn, WriteMultiRegFn write_fn, DelayMsFn delay_fn, GpioCsControlFn cs_fn);

  int init();
  int configure();
  int calibrate();
  int detect();

  int readBurstImu(uint8_t out14[14]);

  DelayMsFn getDelayMs() const { return delay_ms_; }

 private:
  int readReg8(uint8_t reg, uint8_t* val);
  int readReg16(uint8_t reg, uint16_t* out);
  int writeReg8(uint8_t reg, uint8_t val, unsigned delay_ms = 0);
  int writeReg16(uint8_t reg, uint16_t val, unsigned delay_ms = 0);
  int writeMultiDelay(uint8_t reg, const uint8_t* data, uint16_t len, unsigned delay_ms);

  void resetToSpi();
  int basicInit();
  int initConfig();
  int enableCas();
  int enableCrt();

  ReadMultiRegFn read_multi_reg_;
  WriteMultiRegFn write_multi_reg_;
  DelayMsFn delay_ms_;
  GpioCsControlFn gpio_cs_control_;

  int8_t gyro_cas_;
};
