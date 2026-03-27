#pragma once

#include <stdint.h>
#include <string.h>

namespace drvf {

static constexpr uint16_t kImuMaxPacketSize = 20;
static constexpr uint16_t kImuMaxPacketCount = 32;

struct IMURawData {
  uint32_t id;
  uint8_t index;
  int16_t raw_accel[3];
  int16_t raw_gyro[3];
  int16_t raw_temp;
  uint64_t timestamp_us;
  uint16_t PACKET_SIZE;
  uint16_t MAX_PACKET_COUNT;
  uint16_t fifo_count;
  uint16_t packet_count;
  float accel[3];
  float gyro[3];
  float temperature;
  bool is_need_cali_time;
  bool is_use_chip_fifo_time;
  uint16_t chip_cali_fifo_timestamp;
  bool vaild;
  uint8_t fifo_data[kImuMaxPacketSize * kImuMaxPacketCount];
};

class IMURaw {};

}  // namespace drvf
