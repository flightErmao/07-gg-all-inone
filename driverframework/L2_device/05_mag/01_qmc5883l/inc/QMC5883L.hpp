#pragma once

#include <stdint.h>

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10  // FBR[7] soft reset
#define QMC5883L_RESET 11    // FBR[7:0] enable/disable sleep SET/RESET Period
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64 0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ 0b00000000
#define QMC5883L_CONFIG_50HZ 0b00000100
#define QMC5883L_CONFIG_100HZ 0b00001000
#define QMC5883L_CONFIG_200HZ 0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT 0b00000001

class QMC5883L : public Mag {
 public:
  int id_;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;

  QMC5883L(int id);

  bool Init();

  bool ReadWhoAmI(uint8_t* who_am_i);

  bool Read(MagData& data);

  void ExecutePeriodically();

  void SoftReset();

  bool WaitForReady(int max_wait_time_ms);

  bool Deinit();

  // bool WaitForReady(int max_wait_time_ms);
 protected:
  MagData data_;
  Device* port_ = nullptr;
  static constexpr uint8_t QMC5883L_I2C_SLAVE_ADDRESS = 0x0D;
  // dspd::Socket *mag_post_to_ap_socket_;
  static constexpr float coe_8g = 0.0333333f;
};
