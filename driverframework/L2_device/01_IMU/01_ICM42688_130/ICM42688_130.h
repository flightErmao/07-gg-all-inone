#ifndef __ICM42688_130_H__
#define __ICM42688_130_H__

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "rttypes.h"

typedef struct {
  uint32_t id;
  uint8_t index;
  int16_t raw_accel[3];
  int16_t raw_gyro[3];
  int16_t raw_temp;
  uint64_t timestamp_us;
  uint16_t PACKET_SIZE ;
  uint16_t MAX_PACKET_COUNT;
  uint8_t* fifo_data;
  uint16_t fifo_count ;
  uint16_t packet_count ;
  float accel[3];
  float gyro[3];
  float temperature;
  bool is_need_cali_time;
  bool is_use_chip_fifo_time;
  uint16_t chip_cali_fifo_timestamp;
  bool vaild;
}IMURawData;

rt_err_t drv_42688_init(const char* spi_slave_name,const char* imu_device_name);

#endif

