#pragma once

#include <stdint.h>

#include <cmath>
//#include <device/GPIO.hpp>
#include <device/SPI.hpp>
#include <device/Scheduler.hpp>

#include "imu/IMU.hpp"

/*
* [HAVANA/YOKOHAMA] Disable aux pads(pin10&pin11) which are typically connected
to OIS controller if applicable.
* 1. If the pin10 and pin11 are floating in customer design, need define below
macro to disable aux pads to avoid current leak.
* 2. If the pin10 and pin11 are used/connected in customer design(e.g. OIS),
need comment below macro to enable aux pads.
* [YOKO_C1] the aux pads were trimmed to enabled/disable aux, but need to
comment below macro to config pads to avoid current leak.
* 3. [ICM42631][triple interface mode] pin 2/3/10/7/11 set to pull-up and
pin9(AUX2) pull-down
* 4. [ICM42631][single/dual interface mode] pin 7/9/9(AUX2) must be set to 0.

*/

// time limiT WA for A/G-044/045, TOTO: remove this and CHOOSE BETTER sotlution
#define ICM4X6XX_ODR_CHANGING_SAMPLE_MISSING_SAMPLE_GAP
#define ICM4X6XX_YOKOHAMA_MAX_FIFO_SIZE (2080)

/* Periodic reset mode "should be" better than Continuous mode */
#define ICM4X6XX_GYRO_STALL_WORKAROUND
/* workaround for gyro ring down issue */
#define ICM4X6XX_GYRO_RING_DOWN_WA
#define ICM4X6XX_DISABLE_DAE
#define ICM4X6XX_XR_DRM
#define ICM4X6XX_REMOVE_FILTER_DELAY
#define BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS 3
#define BIT_APEX_CONFIG6_HIGHG_PEAK_TH_MASK \
  (0x1F << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS)

/* CTS(JitterVerification.java) allowed jitter is +/-2% */
#define CTS_JITTER_PCNT (0.98f)

#define tick2us(tick) ((tick) / (192 / 10))   // 19.2MHz tick frequency
#define tick2ms(tick) (tick2us(tick) / 1000)  // 19.2MHz tick frequency
#define ms2tick(ms) ((ms)*19200)              // 19.2MHz tick frequency
#define NS2TICK(ns) ((ns)*19.2f / 1000)       // 19.2MHz tick frequency
#define TICK2NS(tick) ((tick) / 19.2f)        // 19.2MHz tick frequency
/**
 * ICM4X6XX ODR (Hz) definitions
 */
#define ICM4X6XX_ODR_0 0.0f
#define ICM4X6XX_ODR_1_5625 1.5625f
#define ICM4X6XX_ODR_3_125 3.125f
#define ICM4X6XX_ODR_6_25 6.25f
#define ICM4X6XX_ODR_12_5 12.5f
#define ICM4X6XX_ODR_25 25.0f
#define ICM4X6XX_ODR_50 50.0f
#define ICM4X6XX_ODR_100 100.0f
#define ICM4X6XX_ODR_200 200.0f
#define ICM4X6XX_ODR_500 500.0f
#define ICM4X6XX_ODR_1000 1000.0f
#define ICM4X6XX_ODR_2000 2000.0f
#define ICM4X6XX_ODR_8000 8000.0f

#define INTVL_TOLERANCE_FACTOR (1.02f)
#define LPF_PARAMETER_N 25
#define MAX_SCP_SAMPLES 20

/* icm4x6xx power mode */
typedef enum icm4x6xx_power_mode {
  ICM4X6XX_A_OFF,
  ICM4X6XX_A_OFF_1,
  ICM4X6XX_A_LPM,
  ICM4X6XX_A_LNM
} icm4x6xx_power_mode;

typedef enum icm4x6xx_sensor_odr {
  ODR_NOT_SUPPORTED = 0,
  ODR_32KHZ = 1,
  ODR_16KHZ = 2,
  ODR_8KHZ = 3,
  ODR_4KHZ = 4,
  ODR_2KHZ = 5,
  ODR_1KHZ = 6,
  ODR_200HZ = 7,
  ODR_100HZ = 8,
  ODR_50HZ = 9,
  ODR_25HZ = 10,
  ODR_12_5HZ = 11,
  ODR_6_25HZ = 12,
  ODR_3_125HZ = 13,
  ODR_1_5625HZ = 14,
  ODR_500HZ = 15,
} icm4x6xx_sensor_odr;

#ifdef ICM4X6XX_REMOVE_FILTER_DELAY
#define ICM4X6XX_FILTER_DELAY_3RD_BW2_ODR_25 31.8f    // a/g filter delay:31.8ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW2_ODR_50 15.9f    // a/g filter delay:15.9ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW2_ODR_100 8.0f    // a/g filter delay:8.0ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW2_ODR_200 4.0f    // a/g filter delay:4.0ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW2_ODR_500 1.6f    // a/g filter delay:1.6ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW2_ODR_1000 0.8f   // a/g filter delay:0.8ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW5_ODR_1000 2.7f   // a/g filter delay:0.8ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW8_ODR_1000 4.0f   // a/g filter delay:0.8ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW16_ODR_1000 6.6f  // a/g filter delay:0.8ms
#define ICM4X6XX_FILTER_DELAY_3RD_BW15_ODR_1000 0.2f  // a/g filter delay:0.2ms
#endif /*ICM4X6XX_REMOVE_FILTER_DELAY*/

typedef struct icm4x6xx_filter_delay_map {
  enum icm4x6xx_sensor_odr odr_reg_value;
  float filter_delay;
} icm4x6xx_filter_delay_map;

typedef enum {
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2000MG =
      (0x00 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2031MG =
      (0x01 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2063MG =
      (0x02 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2094MG =
      (0x03 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2125MG =
      (0x04 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2156MG =
      (0x05 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2188MG =
      (0x06 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2219MG =
      (0x07 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2250MG =
      (0x08 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2281MG =
      (0x09 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2313MG =
      (0x0A << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2344MG =
      (0x0B << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2375MG =
      (0x0C << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2406MG =
      (0x0D << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2438MG =
      (0x0E << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2469MG =
      (0x0F << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2500MG =
      (0x10 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2531MG =
      (0x11 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2563MG =
      (0x12 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2594MG =
      (0x13 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2625MG =
      (0x14 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2656MG =
      (0x15 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2688MG =
      (0x16 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2719MG =
      (0x17 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2750MG =
      (0x18 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2781MG =
      (0x19 << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2813MG =
      (0x1A << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2844MG =
      (0x1B << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2875MG =
      (0x1C << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2906MG =
      (0x1D << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2938MG =
      (0x1E << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS),
  ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2969MG =
      (0x1F << BIT_APEX_CONFIG6_HIGHG_PEAK_TH_POS)
} ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_t;

#define BIT_APEX_CONFIG6_HIGHG_TIME_TH_POS 0
#define BIT_APEX_CONFIG6_HIGHG_TIME_TH_MASK 0x07

typedef enum {
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_20MS = 0x0,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_40MS = 0x1,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_60MS = 0x2,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_80MS = 0x3,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_100MS = 0x4,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_120MS = 0x5,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_140MS = 0x6,
  ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_160MS = 0x7
} ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_t;

namespace drvf {

struct RegAddrNew {
  /* USER BANK 0 REGISTER MAP */
  static constexpr uint8_t DEVICE_CONFIG = 0x11;
  static constexpr uint8_t DRIVE_CONFIG = 0x13;
  static constexpr uint8_t INT_CONFIG = 0x14;
  static constexpr uint8_t FIFO_CONFIG = 0x16;
  static constexpr uint8_t TEMP_DATA1 = 0x1D;
  static constexpr uint8_t TEMP_DATA0 = 0x1E;

  static constexpr uint8_t ACCEL_DATA_X1 = 0x1F;
  static constexpr uint8_t ACCEL_DATA_X0 = 0x20;
  static constexpr uint8_t ACCEL_DATA_Y1 = 0x21;
  static constexpr uint8_t ACCEL_DATA_Y0 = 0x22;
  static constexpr uint8_t ACCEL_DATA_Z1 = 0x23;
  static constexpr uint8_t ACCEL_DATA_Z0 = 0x24;

  static constexpr uint8_t GYRO_DATA_X1 = 0x25;
  static constexpr uint8_t GYRO_DATA_X0 = 0x26;
  static constexpr uint8_t GYRO_DATA_Y1 = 0x27;
  static constexpr uint8_t GYRO_DATA_Y0 = 0x28;
  static constexpr uint8_t GYRO_DATA_Z1 = 0x29;
  static constexpr uint8_t GYRO_DATA_Z0 = 0x2A;

  static constexpr uint8_t TMST_FSYNCH = 0x2B;
  static constexpr uint8_t TMST_FSYNCl = 0x2C;

  static constexpr uint8_t INT_STATUS = 0x2D;

  static constexpr uint8_t FIFO_COUNTH = 0x2E;
  static constexpr uint8_t FIFO_COUNTL = 0x2F;

  static constexpr uint8_t FIFO_DATA = 0x30;

  static constexpr uint8_t APEX_DATA0 = 0x31;
  static constexpr uint8_t APEX_DATA1 = 0x32;
  static constexpr uint8_t APEX_DATA2 = 0x33;
  static constexpr uint8_t APEX_DATA3 = 0x34;
  static constexpr uint8_t APEX_DATA4 = 0x35;
  static constexpr uint8_t APEX_DATA5 = 0x36;

  static constexpr uint8_t INT_STATUS2 = 0x37;
  static constexpr uint8_t INT_STATUS3 = 0x38;

  static constexpr uint8_t SIGNAL_PATH_RESET = 0x4B;

  static constexpr uint8_t INTF_CONFIG0 = 0x4C;
  static constexpr uint8_t INTF_CONFIG1 = 0x4D;

  static constexpr uint8_t PWR_MGMT0 = 0x4E;
  static constexpr uint8_t GYRO_CONFIG0 = 0x4F;
  static constexpr uint8_t ACCEL_CONFIG0 = 0x50;
  static constexpr uint8_t GYRO_CONFIG1 = 0x51;
  static constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
  static constexpr uint8_t ACCEL_CONFIG1 = 0x53;

  static constexpr uint8_t TMST_CONFIG = 0x54;

  static constexpr uint8_t APEX_CONFIG0 = 0x56;

  static constexpr uint8_t SMD_CONFIG = 0x57;

  static constexpr uint8_t FIFO_CONFIG1 = 0x5F;

  static constexpr uint8_t FIFO_CONFIG2 = 0x60;
  static constexpr uint8_t FIFO_CONFIG3 = 0x61;

  static constexpr uint8_t FSYNC_CONFIG = 0x62;

  static constexpr uint8_t INT_CONFIG0 = 0x63;
  static constexpr uint8_t INT_CONFIG1 = 0x64;
  static constexpr uint8_t INT_SOURCE0 = 0x65;
  static constexpr uint8_t INT_SOURCE1 = 0x66;
  static constexpr uint8_t INT_SOURCE3 = 0x68;
  static constexpr uint8_t INT_SOURCE4 = 0x69;

  static constexpr uint8_t FIFO_LOST_PKT0 = 0x6C;
  static constexpr uint8_t FIFO_LOST_PKT1 = 0x6D;
  static constexpr uint8_t SELF_TEST_CONFIG = 0x70;
  static constexpr uint8_t WHO_AM_I = 0x75;
  static constexpr uint8_t REG_BANK_SEL = 0x76;

  /* USER BANK 1 REGISTER MAP */
  static constexpr uint8_t SENSOR_CONFIG0 = 0x03;
  static constexpr uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
  static constexpr uint8_t GYRO_CONFIG_STATIC3 = 0x0C;
  static constexpr uint8_t GYRO_CONFIG_STATIC4 = 0x0D;
  static constexpr uint8_t GYRO_CONFIG_STATIC5 = 0x0E;
  static constexpr uint8_t GYRO_CONFIG_STATIC6 = 0x0F;
  static constexpr uint8_t GYRO_CONFIG_STATIC7 = 0x10;
  static constexpr uint8_t GYRO_CONFIG_STATIC8 = 0x11;
  static constexpr uint8_t GYRO_CONFIG_STATIC9 = 0x12;
  static constexpr uint8_t GYRO_CONFIG_STATIC10 = 0x13;

  static constexpr uint8_t XG_ST_DATA = 0x5F;
  static constexpr uint8_t YG_ST_DATA = 0x60;
  static constexpr uint8_t ZG_ST_DATA = 0x61;
  static constexpr uint8_t TMSTVAL0 = 0x62;
  static constexpr uint8_t TMSTVAL1 = 0x63;
  static constexpr uint8_t TMSTVAL2 = 0x64;
  static constexpr uint8_t INTF_CONFIG4 = 0x7A;
  static constexpr uint8_t INTF_CONFIG5 = 0x7B;
  static constexpr uint8_t INTF_CONFIG6 = 0x7C;

  /* USER BANK 3 REGISTER MAP */
  static constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
  static constexpr uint8_t ACCEL_CONFIG_STATIC3 = 0x04;
  static constexpr uint8_t ACCEL_CONFIG_STATIC4 = 0x05;
  static constexpr uint8_t XA_ST_DATA = 0x3B;
  static constexpr uint8_t YA_ST_DATA = 0x3C;
  static constexpr uint8_t ZA_ST_DATA = 0x3D;

  /* USER BANK 4 REGISTER MAP */
  static constexpr uint8_t APEX_CONFIG1 = 0x40;
  static constexpr uint8_t APEX_CONFIG2 = 0x41;
  static constexpr uint8_t APEX_CONFIG3 = 0x42;
  static constexpr uint8_t APEX_CONFIG4 = 0x43;
  static constexpr uint8_t APEX_CONFIG5 = 0x44;
  static constexpr uint8_t APEX_CONFIG6 = 0x45;
  static constexpr uint8_t APEX_CONFIG7 = 0x46;
  static constexpr uint8_t APEX_CONFIG8 = 0x47;
  static constexpr uint8_t APEX_CONFIG9 = 0x48;

  static constexpr uint8_t ACCEL_WOM_X_THR = 0x4A;
  static constexpr uint8_t ACCEL_WOM_Y_THR = 0x4B;
  static constexpr uint8_t ACCEL_WOM_Z_THR = 0x4C;
  static constexpr uint8_t INT_SOURCE6 = 0x4D;
  static constexpr uint8_t INT_SOURCE7 = 0x4E;
  static constexpr uint8_t INT_SOURCE8 = 0x4F;
  static constexpr uint8_t INT_SOURCE9 = 0x50;
  static constexpr uint8_t INT_SOURCE10 = 0x51;
  static constexpr uint8_t OFFSET_USER0 = 0x77;
  static constexpr uint8_t OFFSET_USER1 = 0x78;
  static constexpr uint8_t OFFSET_USER2 = 0x79;
  static constexpr uint8_t OFFSET_USER3 = 0x7A;
  static constexpr uint8_t OFFSET_USER4 = 0x7B;
  static constexpr uint8_t OFFSET_USER5 = 0x7C;
  static constexpr uint8_t OFFSET_USER6 = 0x7D;
  static constexpr uint8_t OFFSET_USER7 = 0x7E;
  static constexpr uint8_t OFFSET_USER8 = 0x7F;
};

enum class ICM42688Register : uint8_t {
  /* USER BANK 0 REGISTER MAP */
  DEVICE_CONFIG = 0x11,
  DRIVE_CONFIG = 0x13,
  INT_CONFIG = 0x14,
  FIFO_CONFIG = 0x16,
  TEMP_DATA1 = 0x1D,
  TEMP_DATA0 = 0x1E,

  ACCEL_DATA_X1 = 0x1F,
  ACCEL_DATA_X0 = 0x20,
  ACCEL_DATA_Y1 = 0x21,
  ACCEL_DATA_Y0 = 0x22,
  ACCEL_DATA_Z1 = 0x23,
  ACCEL_DATA_Z0 = 0x24,

  GYRO_DATA_X1 = 0x25,
  GYRO_DATA_X0 = 0x26,
  GYRO_DATA_Y1 = 0x27,
  GYRO_DATA_Y0 = 0x28,
  GYRO_DATA_Z1 = 0x29,
  GYRO_DATA_Z0 = 0x2A,

  TMST_FSYNCH = 0x2B,
  TMST_FSYNCl = 0x2C,

  INT_STATUS = 0x2D,

  FIFO_COUNTH = 0x2E,
  FIFO_COUNTL = 0x2F,

  FIFO_DATA = 0x30,

  APEX_DATA0 = 0x31,
  APEX_DATA1 = 0x32,
  APEX_DATA2 = 0x33,
  APEX_DATA3 = 0x34,
  APEX_DATA4 = 0x35,
  APEX_DATA5 = 0x36,

  INT_STATUS2 = 0x37,
  INT_STATUS3 = 0x38,

  SIGNAL_PATH_RESET = 0x4B,

  INTF_CONFIG0 = 0x4C,
  INTF_CONFIG1 = 0x4D,

  PWR_MGMT0 = 0x4E,
  GYRO_CONFIG0 = 0x4F,
  ACCEL_CONFIG0 = 0x50,
  GYRO_CONFIG1 = 0x51,
  GYRO_ACCEL_CONFIG0 = 0x52,
  ACCEL_CONFIG1 = 0x53,

  TMST_CONFIG = 0x54,

  APEX_CONFIG0 = 0x56,

  SMD_CONFIG = 0x57,

  FIFO_CONFIG1 = 0x5F,

  FIFO_CONFIG2 = 0x60,
  FIFO_CONFIG3 = 0x61,

  FSYNC_CONFIG = 0x62,

  INT_CONFIG0 = 0x63,
  INT_CONFIG1 = 0x64,
  INT_SOURCE0 = 0x65,
  INT_SOURCE1 = 0x66,
  INT_SOURCE3 = 0x68,
  INT_SOURCE4 = 0x69,

  FIFO_LOST_PKT0 = 0x6C,
  FIFO_LOST_PKT1 = 0x6D,
  SELF_TEST_CONFIG = 0x70,
  WHO_AM_I = 0x75,
  REG_BANK_SEL = 0x76,

  /* USER BANK 1 REGISTER MAP */
  SENSOR_CONFIG0 = 0x03,
  GYRO_CONFIG_STATIC2 = 0x0B,
  GYRO_CONFIG_STATIC3 = 0x0C,
  GYRO_CONFIG_STATIC4 = 0x0D,
  GYRO_CONFIG_STATIC5 = 0x0E,
  GYRO_CONFIG_STATIC6 = 0x0F,
  GYRO_CONFIG_STATIC7 = 0x10,
  GYRO_CONFIG_STATIC8 = 0x11,
  GYRO_CONFIG_STATIC9 = 0x12,
  GYRO_CONFIG_STATIC10 = 0x13,

  XG_ST_DATA = 0x5F,
  YG_ST_DATA = 0x60,
  ZG_ST_DATA = 0x61,
  TMSTVAL0 = 0x62,
  TMSTVAL1 = 0x63,
  TMSTVAL2 = 0x64,
  INTF_CONFIG4 = 0x7A,
  INTF_CONFIG5 = 0x7B,
  INTF_CONFIG6 = 0x7C,

  /* USER BANK 3 REGISTER MAP */
  ACCEL_CONFIG_STATIC2 = 0x03,
  ACCEL_CONFIG_STATIC3 = 0x04,
  ACCEL_CONFIG_STATIC4 = 0x05,
  XA_ST_DATA = 0x3B,
  YA_ST_DATA = 0x3C,
  ZA_ST_DATA = 0x3D,

  /* USER BANK 4 REGISTER MAP */
  APEX_CONFIG1 = 0x40,
  APEX_CONFIG2 = 0x41,
  APEX_CONFIG3 = 0x42,
  APEX_CONFIG4 = 0x43,
  APEX_CONFIG5 = 0x44,
  APEX_CONFIG6 = 0x45,
  APEX_CONFIG7 = 0x46,
  APEX_CONFIG8 = 0x47,
  APEX_CONFIG9 = 0x48,

  ACCEL_WOM_X_THR = 0x4A,
  ACCEL_WOM_Y_THR = 0x4B,
  ACCEL_WOM_Z_THR = 0x4C,
  INT_SOURCE6 = 0x4D,
  INT_SOURCE7 = 0x4E,
  INT_SOURCE8 = 0x4F,
  INT_SOURCE9 = 0x50,
  INT_SOURCE10 = 0x51,
  OFFSET_USER0 = 0x77,
  OFFSET_USER1 = 0x78,
  OFFSET_USER2 = 0x79,
  OFFSET_USER3 = 0x7A,
  OFFSET_USER4 = 0x7B,
  OFFSET_USER5 = 0x7C,
  OFFSET_USER6 = 0x7D,
  OFFSET_USER7 = 0x7E,
  OFFSET_USER8 = 0x7F,
};

/* fifo work mode */
typedef enum icm4x6xx_fifo_mode {
  BYPASS,
  STREAM,
  SNAPSHOT,
  FIFO_MODE_MASK
} icm4x6xx_fifo_mode;

/* sensor bandwidth */
typedef enum icm4x6xx_bandwidth {
  BW_ODR_DIV_2 = 0,
  BW_ODR_DIV_4 = 1,
  BW_ODR_DIV_5 = 2,
  BW_ODR_DIV_8 = 3,
  BW_ODR_DIV_10 = 4,
  BW_ODR_DIV_16 = 5,
  BW_ODR_DIV_20 = 6,
  BW_ODR_DIV_40 = 7,
  BW_LL_MAX_200_8X_ODR = 15
} icm4x6xx_bandwidth;

typedef enum icm4x6xx_gyro_fsr {
  GYRO_RANGE_2000DPS = 0,
  GYRO_RANGE_1000DPS = 1,
  GYRO_RANGE_500DPS = 2,
  GYRO_RANGE_250DPS = 3,
  GYRO_RANGE_125DPS = 4,
  GYRO_RANGE_62_5DPS = 5,
  GYRO_RANGE_31_25DPS = 6,
  GYRO_RANGE_15_625DPS = 7,
  GYRO_RANGE_MAX = GYRO_RANGE_15_625DPS
} icm4x6xx_gyro_fsr;

/* Filter order */
typedef enum icm4x6xx_filter_order {
  FIRST_ORDER,  // 1st order
  SEC_ORDER,    // 2nd order
  THIRD_ORDER,  // 3rd order
  ORDER_MASK    // N/A
} icm4x6xx_filter_order;

#define REG_CHIP_CONFIG 0x11
#define SOFT_RESET_MASK 0x01
#define REG_DRIVE_CONFIG 0x13
#define SPI_SPEED_MASK 0x07
#define I2C_SPEED_MASK 0x38
#define I2C_SPEED_SHIFT 3
#define REG_INT_CONFIG 0x14
#define INT1_ACTIVE_HIGH_MASK 0x01
#define INT1_PUSH_PULL_MASK 0x02
#define INT1_LATCHED_MODE_MASK 0x04
#define INT2_ACTIVE_HIGH_MASK 0x08
#define INT2_PUSH_PULL_MASK 0x10
#define INT2_LATCHED_MODE_MASK 0x20
#define REG_FIFO_CONFIG 0x16
#define BIT_FIFO_MODE_SHIFT (6)
#define BIT_FIFO_MODE_CTRL_MASK ((0x03) << BIT_FIFO_MODE_SHIFT)
#define BIT_FIFO_MODE_CTRL_BYPASS ((0x00) << BIT_FIFO_MODE_SHIFT)
#define BIT_FIFO_MODE_CTRL_STREAM ((0x01) << BIT_FIFO_MODE_SHIFT)
#define BIT_FIFO_MODE_CTRL_SNAPSHOT ((0x02) << BIT_FIFO_MODE_SHIFT)
#define REG_TEMP_DATA0 0x1d
#define REG_ACCEL_DATA_X0_UI 0x1F
#define REG_GYRO_DATA_X0_UI 0x25
#define REG_TMST_DATA 0x2b
#define REG_FIFO_BYTE_COUNT_L 0x2e
#define REG_INT_STATUS 0x2d
#define RESET_DONE_MASK 0x10
#define DRI_INT_MASK 0x08
#define WM_INT_MASK 0x04
#define OVERFLOW_INT_MASK 0x02
#define REG_FIFO_DATA 0x30
#define REG_APEX_DATA0 0x31
#define REG_APEX_DATA1 0x32
#define REG_APEX_DATA2 0x33
#define REG_APEX_DATA3 0x34
#define BIT_DMP_IDLE 0X04
#define REG_INT_STATUS2 0x37
#define BIT_INT_STATUS_SMD 0x08
#define BIT_INT_STATUS_WOM_Z 0x04
#define BIT_INT_STATUS_WOM_Y 0x02
#define BIT_INT_STATUS_WOM_X 0x01
/* Havana INT_STATUS2 config register */
#define REG_HAVANA_INT_STATUS2 0x59

#define REG_INT_STATUS3 0x38
#define BIT_INT_DMP_POWER_SAVE_DET 0x40
#define BIT_INT_STATUS3_STEP_DET 0x20
#define BIT_INT_STATUS3_STEP_CNT_OVFL 0x10
#define BIT_INT_STATUS3_TILT_DET 0x08
#define BIT_INT_STATUS3_WAKE_DET 0x04
#define BIT_INT_STATUS3_SLEEP_DET 0x02
#ifndef ICM4X6XX_YOKOHAMA_C1
#define BIT_INT_STATUS3_LOW_G_DET 0x04
#define BIT_INT_STATUS3_HIGH_G_DET 0x02
#else
#define BIT_INT_STATUS3_LOW_G_DET 0x02
#endif
#define BIT_INT_STATUS3_TAP_DET 0x01

#define REG_SIGNAL_PATH_RESET_REG 0x4B
#define BIT_DMP_INIT_EN 0x40
#define BIT_DMP_MEM_RESET_EN 0x20
#define BIT_ABORT_AND_RESET 0x08
#define BIT_TMST_STROBE 0x04
#define BIT_FIFO_FLUSH 0x02
#define BIT_TEMP_RST 0x01
#define REG_INTF_CONFIG0 0x4C
#define FIFO_HOLD_LAST_DATA_EN 0x80
#define RECORD_MODE_MASK 0x40
#define FIFO_COUNT_BIG_ENDIAN_MASK 0x20
#define SENSOR_DATA_BIG_ENDIAN_MASK 0x10
#define UI_INTF_MASK 0x03
#define REG_INTF_CONFIG1 0x4D
#define BIT_ACCEL_LP_CLK_SEL 0x08
#define BIT_RTC_MODE_EN 0x04
#define REG_PWR_MGMT_0 0x4E
#define ACCEL_LNM_MASK 0x03
#define GYRO_LNM_MASK 0x0C
#define ACCEL_GYRO_POWERUP 0x0F
#define ICM4X6XX_IDLE_MASK 0x10
#define REG_GYRO_CONFIG0 0x4F
#define GYRO_FSR_MASK 0xE0
#define GYRO_ODR_MASK 0x0F
#define ACCEL_FSR_SHIFT 5
#define REG_ACCEL_CONFIG0 0x50
#define ACCEL_FSR_MASK 0xE0
#define ACCEL_ODR_MASK 0x0f
#define GYRO_FSR_SHIFT 5
#define REG_GYRO_CONFIG1 0x51
#define BIT_GYRO_AVG_FILT_8K_HZ 0x10
#define BIT_GYRO_FILT_ORD_SHIFT 0x02
#define BIT_GYRO_FILT_ORD_MASK 0x0C
#define REG_GYRO_ACCEL_CONFIG0 0x52
#define BIT_GYRO_BW_MASK 0x0f
#define BIT_ACCEL_BW_MASK 0xf0
#define BIT_ACCEL_BW_SHIFT 0x04
#define REG_ACC_CONFIG1 0x53
#define BIT_ACC_AVG_FILT_8K_HZ 0x01
#define BIT_ACC_FILT_ORD_SHIFT 0x03
#define BIT_ACC_FILT_ORD_MASK 0x18
#define REG_TMST_CONFIG_REG 0x54
#define TMST_SREG_ON_EN 0x80
#define BIT_TMST_TO_REGS_EN 0x10
#define BIT_TMST_RESOL 0x08
#define BIT_TMST_DELTA_EN 0x04
#define BIT_TMST_FSYNC_EN 0x02
#define BIT_TMST_EN 0x01

#define REG_APEX_CONFIG0 0x56
#define BIT_DMP_POWER_SAVE_EN 0x80
#define BIT_PEDO_EN 0x20
#ifndef ICM4X6XX_YOKOHAMA_C1
#define BIT_LOWG_EN 0x08
#define BIT_HIGH_EN 0x04
#else
#define BIT_LOWG_EN 0x04
#endif
#define BIT_DMP_ODR_25HZ 0x00
#define BIT_DMP_ODR_50HZ 0x02
#define BIT_DMP_ODR_100HZ 0x03
#ifdef ICM4X6XX_YOKOHAMA_C1
#define BIT_DMP_ODR_500HZ 0x01
#endif
#define BIT_DMP_ODR_MASK 0x03

/* Havana TMST config register */
#define REG_HAVANA_TMST_CONFIG_REG 0x5A

/* Havana WOM THRESH register */
#define REG_HAVANA_ACCEL_WOM_X_THR 0x54
#define REG_HAVANA_ACCEL_WOM_Y_THR 0x55
#define REG_HAVANA_ACCEL_WOM_Z_THR 0x56

#define REG_SMD_CONFIG 0x57
#define BIT_WOM_AND_MODE 0x08
#define BIT_WOM_COMPARE_PRE 0x04
#define SMD_MODE_MASK 0x03
#define REG_FIFO_CONFIG_1 0x5f
#define FIFO_RESUME_PARTIAL_RD_MASK 0x40
#define FIFO_WM_GT_TH_MASK 0x20
#define FIFO_HIRES_EN_MASK 0x10
#define FIFO_TMST_FSYNC_EN_MASK 0x08
#define FIFO_TEMP_EN_MASK 0x04
#define FIFO_GYRO_EN_MASK 0x02
#define FIFO_ACCEL_EN_MASK 0x01
#define REG_FIFO_WM_TH_L 0x60
#define REG_FIFO_WM_TH_H 0x61
#define REG_FSYNC_CONFIG 0x62
#define REG_INT_CONFIG0 0x63
#define INT_FIFO_THS_CLR_SEL_MASK 0x0C
#define INT_FIFO_THS_CLR_SEL_SHIFT 0x02
#define REG_INT_CONFIG1 0x64
#define BIT_INT_ASY_RST_DIS_MASK 0x10
#define REG_INT_SOURCE0 0x65
#define DRI_EN_MASK 0x08
#define WM_INT_EN_MASK 0x04
#define FIFO_FULL_EN_MASK 0x02
#define REG_INT_SOURCE1 0x66
#define WOM_EN_MASK 0x07
#define SMD_EN_MASK 0x08
#define REG_INT_SOURCE3 0x68
#define DRI_INT2_EN_MASK 0x08
#define WM_INT_INT2_EN_MASK 0x04
#define FIFO_FULL_INT2_EN_MASK 0x02
#define REG_INT_SOURCE4 0x69
#define WOM_INT2_EN_MASK 0x07
#define SMD_INT2_EN_MASK 0x08
#define REG_SELF_TEST_CONFIG 0x70
#define BIT_ST_REGULATOR_EN 0x40
#define BIT_ACCEL_Z_ST_EN 0x20
#define BIT_ACCEL_Y_ST_EN 0x10
#define BIT_ACCEL_X_ST_EN 0x08
#define BIT_GYRO_Z_ST_EN 0x04
#define BIT_GYRO_Y_ST_EN 0x02
#define BIT_GYRO_X_ST_EN 0x01
#define REG_SCAN0 0x71
#define BIT_DMP_MEM_ACCESS_EN 0x08
#define BIT_MEM_OTP_ACCESS_EN 0x04
#define BIT_FIFO_MEM_RD_SYS 0x02
#define BIT_FIFO_MEM_WR_SER 0x01
#define REG_MEM_BANK_SEL 0x72
#define REG_MEM_START_ADDR 0x73
#define REG_MEM_R_W 0x74
#define REG_WHO_AM_I 0x75
#define REG_BANK_SEL 0x76

/* Bank 1 */
#define REG_XG_ST_DATA 0x5F
#define REG_YG_ST_DATA 0x60
#define REG_ZG_ST_DATA 0x61

#define REG_TMSTVAL0 0x62
#define REG_TMSTVAL1 0x63
#define REG_TMSTVAL2 0x64
#define REG_INTF_CONFIG5 0x7B
#define BIT_PIN9_FUNC_INT2 0x00
#define BIT_PIN9_FUNC_FSYNC 0x02
#define BIT_PIN9_FUNC_CLKIN 0x04
#define BIT_PIN9_FUNC_MASK 0x06
#define REG_INTF_CONFIG6 0x7C
#define I3C_SDR_EN 0x01
#define I3C_DDR_EN 0x02

/* Bank 2 */
#define REG_XA_ST_DATA 0x3B
#define REG_YA_ST_DATA 0x3C
#define REG_ZA_ST_DATA 0x3D

/* Bank 3 */
#define REG_PU_PD_CONFIG1 0x06
#define BIT_PIN11_PU_EN 0x80
#define BIT_PIN7_PU_EN 0x40
#define BIT_PIN9_AUX2_PD_EN 0x20
#define BIT_PIN9_PD_EN 0x10
#define BIT_PIN10_PU_EN 0x08
#define BIT_PIN3_PU_EN 0x04
#define BIT_PIN2_PU_EN 0x02

#ifdef ICM4X6XX_GYRO_STALL_WORKAROUND
#define REG_AMP_GSXYZ_TRIM0 0x2E
#define BIT_GYRO_SC2V_CONT_MODE 0x02
#define REG_AMP_GX_TRIM2 0x32
#define BIT_GX_SC2V_FET_TRIM_MASK 0x60
#define REG_AMP_GY_TRIM2 0x37
#define BIT_GY_SC2V_FET_TRIM_MASK 0x60
#define REG_AMP_GZ_TRIM2 0x3C
#define BIT_GZ_SC2V_FET_TRIM_MASK 0x60
#endif

/* Bank 4 */
#define WOM_THR_BANK 4
#define REG_FDR_CONFIG 0x09
#define BIT_FDR_MASK 0x7F
#define REG_APEX_CONFIG1 0x40
#define BIT_LOW_ENERGY_AMP_TH_SEL 0xF0
#define REG_APEX_CONFIG2 0x41
#define BIT_PED_AMP_TH_SEL 0xF0
#define BIT_PED_STEP_CNT_TH_SEL 0x0F
#define REG_APEX_CONFIG3 0x42
#define BIT_PED_STEP_DET_TH_SEL 0xE0
#define BIT_PED_SB_TIMER_TH_SELPED_SB_TIMER_TH_SEL 0x1C
#define BIT_PED_HI_EN_TH_SEL 0x03
#define REG_APEX_CONFIG4 0x43
#define REG_APEX_CONFIG5 0x44
#define REG_APEX_CONFIG6 0x45
#define REG_WOM_X_THR 0x4A
#define REG_WOM_Y_THR 0x4B
#define REG_WOM_Z_THR 0x4C
#define REG_INT_SOURCE6 0x4D
#define BIT_DMP_POWER_SAVE_INT1_EN 0x40
#define BIT_STEP_DET_INT1_EN 0x20
#define BIT_STEP_CNT_OVFL_INT1_EN 0x10
#define BIT_TILT_DET_INT1_EN 0x8
#define BIT_WAKE_DET_INT1_EN 0x4
#define BIT_SLEEP_DET_INT1_EN 0x2
#ifndef ICM4X6XX_YOKOHAMA_C1
#define BIT_LOWG_DET_INT1_EN 0x4
#define BIT_HIGHG_DET_INT1_EN 0x2
#else
#define BIT_LOWG_DET_INT1_EN 0x2
#endif
#define BIT_TAP_DET_INT1_EN 0x1
#define REG_INT_SOURCE7 0x4E
#define BIT_DMP_POWER_SAVE_INT2_EN 0x40
#define BIT_STEP_DET_INT2_EN 0x20
#define BIT_STEP_CNT_OVFL_INT2_EN 0x10
#define BIT_TILT_DET_INT2_EN 0x8
#define BIT_WAKE_DET_INT2_EN 0x4  // Not mentioned in Excel Reg Map
#ifndef ICM4X6XX_YOKOHAMA_C1
#define BIT_LOWG_DET_INT2_EN 0x4  // From Excel Reg Map
#define BIT_HIGHG_DET_INT2_EN 0x2
#else
#define BIT_LOWG_DET_INT2_EN 0x2
#endif
#define BIT_TAP_DET_INT2_EN 0x1
#define REG_INT_SOURCE8 0x4F
#define BIT_DRI_IBI_EN 0x08
#define BIT_WM_IBI_EN 0x04
#define BIT_FIFO_FULL_IBI_EN 0x02
#define REG_INT_SOURCE9 0x50
#define BIT_I3C_PROTOCOL_ERROR_IBI_EN 0x80
#define BIT_SMD_IBI_EN 0x10
#define BIT_WOM_Z_IBI_EN 0x08
#define BIT_WOM_Y_IBI_EN 0x04
#define BIT_WOM_X_IBI_EN 0x02
#define REG_INT_SOURCE10 0x51
#define BIT_STEP_DET_IBI_EN 0x20
#define BIT_STEP_CNT_OVFL_IBI_EN 0x10
#define BIT_TILT_DET_IBI_EN 0x8
#define BIT_WAKE_DET_IBI_EN 0x4
#define BIT_SLEEP_DET_IBI_EN 0x2
#ifndef ICM4X6XX_YOKOHAMA_C1
#define BIT_LOWG_DET_IBI_EN 0x4
#define BIT_HIGHG_DET_IBI_EN 0x2
#else
#define BIT_LOWG_DET_IBI_EN 0x2
#endif
#define BIT_TAP_DET_IBI_EN 0x1

typedef enum icm4x6xx_ui_intf {
  AUTO_INTF,
  I2C_INTF = 2,
  SPI_INTF = 3,
} icm4x6xx_ui_intf;

enum class ICM42688AccelScaleNew : int {
  PM2G = 2,
  PM4G = 4,
  PM8G = 8,
  PM16G = 16
};

enum class ICM42688GyroScaleNew : int {
  PM250DPS = 250,
  PM500DPS = 500,
  PM1000DPS = 1000,
  PM2000DPS = 2000
};

typedef enum icm4x6xx_accel_fsr {
  ACC_RANGE_16G = 0,
  ACC_RANGE_8G = 1,
  ACC_RANGE_4G = 2,
  ACC_RANGE_2G = 3,
  ACC_RANGE_1G = 4,
  ACC_RANGE_MAX = ACC_RANGE_1G
} icm4x6xx_accel_fsr;

typedef enum {
  ICM4X6XX_INT_ACTIVE_LOW,
  ICM4X6XX_INT_ACTIVE_HIGH
} icm4x6xx_int_polarity;

typedef union icm4x6xx_fifo_header_t {
  unsigned char head_byte;
  struct {
    unsigned char g_odr_change_bit : 1;
    unsigned char a_odr_change_bit : 1;
    unsigned char timestamp_bit : 2;
    unsigned char twentybits_bit : 1;
    unsigned char gyro_bit : 1;
    unsigned char accel_bit : 1;
    unsigned char msg_bit : 1;
  } bits;
} icm4x6xx_fifo_header_t;

/* fifo format*/
typedef enum icm4x6xx_fifo_format {
  ICM4X6XX_FORMAT_EMPTY,
  ICM4X6XX_FORMAT_ACCEL_8_BYTES,
  ICM4X6XX_FORMAT_GYRO_8_BYTES,
  ICM4X6XX_FORMAT_16_BYTES,
  ICM4X6XX_FORMAT_20_BYTES,
  ICM4X6XX_FORMAT_SPECIAL,
  ICM4X6XX_FORMAT_UNKNOWN
} icm4x6xx_fifo_format;

#define FIFO_HEADER_EMPTY_BIT (0x80)
#define FIFO_HEADER_A_BIT (0x40)
#define FIFO_HEADER_G_BIT (0x20)
#define FIFO_HEADER_20_BIT (0x10)

class ICM42688 : public IMURaw {
 public:
  ICM42688(int id,int cs);

  ~ICM42688();
  

  bool Init(bool clkin_enable);
  int DebugInit();
  int DebugInit(bool clkin_enable);

  // choose different config parameters group
  int DebugInit(ConfigGroup group_Id);
  bool Deinit();
  bool Read(IMURawData &data);
  bool ReadRaw(IMURawData &data);

  bool ReadWhoAmI(uint8_t *who_am_i);
  int ReadAccel(int16_t accel_raw_data[3]);
  int ReadGyro(int16_t gyro_raw_data[3]);
  int ReadTemp(int16_t &temp_raw_data);
  bool WriteByte(uint8_t reg, uint8_t val);
  bool ReadBlock(uint8_t first_reg, uint8_t buf[], int len);
  void ExecutePeriodically();

 public:
  static constexpr uint8_t WHO_AM_I_ID = 0x47;

 private:
  bool Init();

 protected:
  Device *port_ = nullptr;
  bool WriteMask(uint32_t reg_addr, uint8_t reg_value, uint8_t mask);

  bool icm4x6xx_disable_aux_pins();
  
  bool icm4x6xx_enable_gyro_periodic_reset();

  bool icm4x6xx_enable_int_async_reset(bool enable);

  bool icm4x6xx_enable_record_mode(bool enable);

  bool icm4x6xx_config_highg_parameter();

  bool icm4x6xx_set_reg_bank(uint8_t bank_num);

  bool icm4x6xx_en_high_shock_int(bool enable);

  bool icm4x6xx_en_fifo_full_int(bool enable);

  bool icm4x6xx_set_fifo_mode(icm4x6xx_fifo_mode fifo_mode);

  bool icm4x6xx_set_gyro_bandwidth(icm4x6xx_bandwidth bw);

  bool icm4x6xx_set_accel_bandwidth(icm4x6xx_bandwidth bw);

  bool icm4x6xx_set_gyro_filter_order(icm4x6xx_filter_order order);

  bool icm4x6xx_set_accel_filter_order(icm4x6xx_filter_order order);

  bool icm4x6xx_set_gyro_fsr(icm4x6xx_gyro_fsr fsr);

  bool icm4x6xx_set_accel_fsr(icm4x6xx_accel_fsr fsr);

  bool icm4x6xx_config_fsync(uint8_t data);

  bool icm4x6xx_en_fifo(bool en_accel, bool en_gyro);

  bool icm4x6xx_config_ui_intf(icm4x6xx_ui_intf intf);

  bool icm4x6xx_en_int_push_pull(bool enable);

  bool icm4x6xx_en_int_latched_mode(bool enable);

  bool icm4x6xx_enable_nflt_gyro(bool enable);

  bool icm4x6xx_enable_nflt_acc(bool enable);

  bool icm4x6xx_config_int_polarity(icm4x6xx_int_polarity polarity);

  bool icm4x6xx_en_big_endian_mode(bool enable);

  bool icm4x6xx_en_fifo_hold_last_data(bool enable);

  bool icm4x6xx_odr_to_reg_val(float odr, icm4x6xx_sensor_odr *odr_reg);

  bool icm4x6xx_read_fifo_count(uint16_t *count);

  /**
   * @brief Set Accel ODR
   *
   * @param[in] odr    the Accel ODR will be set.
   *
   * @return 0 if success,
   *         non-zero value if failed.
   */
  bool icm4x6xx_set_accel_odr(float odr);

  /**
   * @brief Set Gyro and Accel ODR, in this setting
   *        Gyro and Accel always share same odr
   *
   * @param[in] odr    the Accel/Gyro ODR will be set.
   *
   * @return 0 if success,
   *         non-zero value if failed.
   */
  bool icm4x6xx_set_gyro_odr(float odr);

  bool icm4x6xx_en_gyro(bool enable);

  bool icm4x6xx_set_accel_mode(icm4x6xx_power_mode mode);

  bool icm4x6xx_en_dri(bool enable);

  bool icm4x6xx_read_int_status(uint8_t *status);

  bool icm4x6xx_read_int_status2(uint8_t *status);

  bool icm4x6xx_get_packet_size(uint8_t *size);

  bool icm4x6xx_read_fifo_buf(uint8_t *buf, uint32_t len);

  void GetFIFODataFormat();

  bool icm4x6xx_enable_tmst_val(bool enable);

  bool icm4x6xx_read_tmst_val(uint32_t *tmst_reg);

  bool icm4x6xx_is_fifo_format_match(icm4x6xx_fifo_format format);

  bool icm4x6xx_enable_rtc_mode(bool enable);

  bool icm4x6xx_enable_tmst(bool enable);

  bool icm4x6xx_enable_delta_tmst(bool enable);

  /* 
  https://github.com/ArduPilot/ardupilot/issues/25025#issuecomment-1773910280
  The problem is caused due to the default setting of the sensors configuration.
  The default setting used in the the sensor is to to fine-tune the device for lower noise density at lower measurement
  ranges, while still providing the full sensor range. At around 100 dps, you can observe the sensor switching from
  low-range (low noise) to high-range (higher noise) mode, causing the gyro data to be sticky for a short duration of
  the time. When you change the setting to disable the switching automatically, the device will always be in high-range
  mode, with a slight detoriation in noise density at FSR < 100 dps. The resolution to the issue is to set bit [7:6] =
  01 at the address of 77 at bank 0 register. The default bit [7:6] = 10.
  */
  bool icm4x6xx_disable_afsr();

  uint32_t icm4x6xx_cal_valid_fifo_len(const uint8_t *buf, uint32_t buf_len,
                                       uint16_t *cnt);

  uint32_t get_offset(uint16_t early, uint16_t later);

  bool icm4x6xx_accel_gyro_powerup(uint8_t mode);

  static constexpr float ACCEL_SCALER =
      (float)ICM42688AccelScaleNew::PM16G / (1 << 15) * 9.80665;
  static constexpr float GYRO_SCALER = (float)ICM42688GyroScaleNew::PM2000DPS /
                                       (1 << 15) * (3.141592653f) / 180.0f;
  static constexpr float TEMPERATURE_SCALER = 132.48f;
  static constexpr float TEMPERATURE_OFFSET = 25.0f;
  static constexpr uint32_t BAUDRATE_SPI = 10000000;

  int id_;
  int cs_;
  uint64_t err_cnt_ = 0;
  int notify_id_;
  IMURawData internal_raw_data_;
  icm4x6xx_gyro_fsr gyro_fsr_;
  icm4x6xx_accel_fsr accel_fsr_;
  uint8_t a_res_idx_;
  uint8_t g_res_idx_;
  icm4x6xx_fifo_mode fifo_mode_;
  bool use_hi_res_;
  bool en_a_fifo_;
  bool en_g_fifo_;

  icm4x6xx_bandwidth accel_bandwith_;
  bool fifo_info_record_mode_;
  icm4x6xx_power_mode accel_power_mode_;
  icm4x6xx_fifo_format desire_format_;

  icm4x6xx_fifo_format curr_format_;

  uint64_t last_odr_timestamp_;

  int sync_time_count_;

  double dsp_fifo_timestamp_;
  uint16_t last_fifo_timestamp_16b_;

  uint32_t odr_time_offset_;
  // uint32_t odr_time_offset_before;
  bool has_inited_;
  bool clkin_enable_;
};
}  // namespace drvf
