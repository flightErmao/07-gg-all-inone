#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "imu/inv_imu_driver.h"
#ifdef __cplusplus
}
#endif
#ifdef ICM45686
#undef ICM45686
#endif

#include "imu/IMU.hpp"
#include "spi_interface.hpp"

namespace drvf {

struct ICM45686HwConfig {
  const char *device_name;
  const char *spi_bus_name;
  const char *spi_slave_name;
  const char *spi_cs_pin;
  uint32_t spi_max_hz;
  uint8_t whoami_reg;
  uint8_t whoami_expected;
};

class ICM45686 {
 public:
  ICM45686(int id, int cs, const ICM45686HwConfig &config);
  ~ICM45686();

  int DebugInit();
  bool ReadRaw(IMURawData &data);
  bool ReadUiSnapshot(uint8_t *buf, uint16_t len);
  bool probe();
  void DumpKeyRegisters(const char *stage);
  uint8_t lastWhoAmI() const;
  int transportReadReg(uint8_t reg, uint8_t *buf, uint32_t len);
  int transportWriteReg(uint8_t reg, const uint8_t *buf, uint32_t len);

 private:
  bool initSpi();
  bool readRegister(uint8_t reg, uint8_t *value);
  bool readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool writeRegisterChecked(uint8_t reg, uint8_t value, uint8_t expected_mask = 0xFFU);
  bool updateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value);
  bool readMreg(uint16_t reg, uint8_t *buf, uint16_t len);
  bool writeMreg(uint16_t reg, const uint8_t *buf, uint16_t len);
  bool writeMregByte(uint16_t reg, uint8_t value);
  bool writeMregByteChecked(uint16_t reg, uint8_t value, uint8_t expected_mask = 0xFFU);
  bool updateMregBits(uint16_t reg, uint8_t mask, uint8_t value);
  bool configureWithReferenceDriver();
  bool readUiSnapshotWithReferenceDriver(uint8_t *buf, uint16_t len);
  bool readFifoWithReferenceDriver(IMURawData &data);
  bool readFifoByteCount(uint16_t *byte_count);
  bool readFifoData(uint8_t *buf, uint16_t len);
  void logKeyRegisters(const char *stage);
  bool configureForPolling();
  bool flushFifo();

  int id_;
  int cs_;
  bool spi_inited_;
  bool configured_;
  uint8_t last_whoami_;
  uint16_t active_spi_mode_;
  SpiInterface spi_;
  inv_imu_device_t ref_device_;
  ICM45686HwConfig config_;
};

}  // namespace drvf
