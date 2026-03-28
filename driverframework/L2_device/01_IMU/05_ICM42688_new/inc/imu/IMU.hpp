#pragma once

#include <stdint.h>
#include <string.h>

namespace drvf {

static constexpr uint16_t kImuMaxPacketSize = 20;
static constexpr uint16_t kImuMaxPacketCount = 32;

struct IMURawData {
  uint64_t timestamp_us;
  uint16_t packet_size;
  uint16_t fifo_count;
  uint8_t fifo_data[kImuMaxPacketSize * kImuMaxPacketCount];
};

class IMURaw {};

}  // namespace drvf
