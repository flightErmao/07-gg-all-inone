#include "ICM42688.hpp"
#include <device/DeviceFactory.hpp>
#include <device/Timer.hpp>
#include <utils/Utils.hpp>

// #include <ctype.h>
// #include <fcntl.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include "DALTLMM.h"

namespace drvf {
#define TPM 0


ICM42688::ICM42688(int id,int cs) : id_(id),cs_(cs){
  use_hi_res_ = false;
  fifo_info_record_mode_ = false;
  fifo_mode_ = STREAM;
  desire_format_ = ICM4X6XX_FORMAT_16_BYTES;

  en_a_fifo_ = true;
  en_g_fifo_ = true;

  last_odr_timestamp_ = 0;
  sync_time_count_ = 0;
  odr_time_offset_ = 2000;
  // odr_time_offset_before = 2000;
  last_fifo_timestamp_16b_ = 0;

}

ICM42688::~ICM42688() {}

bool ICM42688::Init() {

  return true; 
}

bool ICM42688::Init(bool clkin_enable) {

  // io_port_open(160,1);
  // io_port_write(160,1);
  // 3 is mainboard imu of spi instance numble
  port_ = DeviceFactory::ShareInstance()->GetSPIDeviceById(id_,cs_,BAUDRATE_SPI);
  if (port_ == nullptr) {
    logger.Info("[ICM42688.cpp][ICM42688::Init][spi init fail!]");
  }
  has_inited_ = false;
  logger.Info("[%d]ICM42688::Start init(), clkin = %d\n", id_, clkin_enable);
  gyro_fsr_ = GYRO_RANGE_2000DPS;
  accel_fsr_ = ACC_RANGE_16G;
  fifo_mode_ = STREAM;
  accel_bandwith_ = BW_LL_MAX_200_8X_ODR;
  accel_power_mode_ = ICM4X6XX_A_LNM;
  use_hi_res_ = false;
  /////////// Check Who Am I
  uint8_t id = 0;
  if (port_->ReadRegister(RegAddrNew::WHO_AM_I, &id)) {
    logger.Info("[%d]ICM42688::CheckWhoAmI(): error! cannot read WHO_AM_I id!", id_);
    return false;
  }
  if (id != WHO_AM_I_ID) {
    logger.Info("[%d]ICM42688::CheckWhoAmI(): error! wanted ID: 0x%02X, got: 0x%02X", id_, WHO_AM_I_ID, id);
    return false;
  }

  clkin_enable_ = clkin_enable;

  if (!icm4x6xx_config_ui_intf(SPI_INTF)) {
    logger.Info("[%d]ICM42688::icm4x6xx_config_ui_intf() faild", id_);
    return false;
  }

  if (!icm4x6xx_enable_rtc_mode(clkin_enable)) {
    logger.Info("[%d]ICM42688::icm4x6xx_enable_rtc_mode() faild", id_);
    return false;
  }

#if 0
    /* Enable Timestamp field contains the measurement of time
     * since the last occurrence of ODR.*/
   if (!icm4x6xx_enable_delta_tmst(true))
   {
     logger.Info("[%d]ICM42688::icm4x6xx_enable_delta_tmst() faild", id_);
     return false;
   }
#else

  if (clkin_enable) {
    /* Enable timestamp register */
    if (!icm4x6xx_enable_tmst(true)) {
      logger.Info("[%d]ICM42688::icm4x6xx_enable_tmst() faild", id_);
      return false;
    }
  }
#endif

  /*if (!icm4x6xx_en_int_push_pull(false)) {
    return false;
  }

  if (!icm4x6xx_en_int_latched_mode(false)) {
    return false;
  }

  if (!icm4x6xx_config_int_polarity(ICM4X6XX_INT_ACTIVE_HIGH)) {
    return false;
  }*/

  /* Choose big endian mode for fifo count and sensor data
   * default mode for FPGA and chip may be different,
   * so we choose one mode here for both */
  if (!icm4x6xx_en_big_endian_mode(true)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_big_endian_mode() faild", id_);
    return false;
  }

  /* enable fifo hold last data */
  /*if (!icm4x6xx_en_fifo_hold_last_data(false)) {
    return false;
  }*/

  /* do not tag fsync flag for temperature resolution */
  if (!icm4x6xx_config_fsync(0)) {
    logger.Info("[%d]ICM42688::icm4x6xx_config_fsync() faild", id_);
    return false;
  }

  /* Choose Accel FSR */
  if (!icm4x6xx_set_accel_fsr(accel_fsr_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_fsr() faild", id_);
    return false;
  }

  /* Choose Gyro FSR */
  if (!icm4x6xx_set_gyro_fsr(gyro_fsr_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_fsr() faild", id_);
    return false;
  }

  /* Choose Accel filter order */
  if (!icm4x6xx_set_accel_filter_order(THIRD_ORDER))  // USE 3rd order filter
  {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_filter_order() faild", id_);
    return false;
  }

  /* Choose Gyro filter order */
  if (!icm4x6xx_set_gyro_filter_order(THIRD_ORDER))  // USE 3rd order filter
  {
    logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_filter_order() faild", id_);
    return false;
  }

  /* Enable fifo */
  if (!icm4x6xx_set_fifo_mode(fifo_mode_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_fifo_mode() faild", id_);
    return false;
  }
#if 0
  /* Enable fifo overflow interrupt */
  if (!icm4x6xx_en_fifo_full_int(false)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_fifo_full_int() faild", id_);
    return false;
  }

  /* Enable high shock interrupt */
  if (!icm4x6xx_en_high_shock_int(false)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_high_shock_int() faild", id_);
    return false;
  }

  /* Config highg parameter*/
  if (!icm4x6xx_config_highg_parameter()) {
    logger.Info("[%d]ICM42688::icm4x6xx_config_highg_parameter() faild", id_);
    return false;
  }

  // TODO: Enable wm on IBI here, since we didn't need irq reg ready now

  /* Choose fifo count record or byte mode */
  // TODO: do test with both byte and record mode
  /* H_W_B 9450 pls don't use record mode for havana */
  if (!icm4x6xx_enable_record_mode(fifo_info_record_mode_))
  {
    logger.Info("[%d]ICM42688::icm4x6xx_enable_record_mode() faild", id_);
    return false;
  }
  /* The field int_asy_rst_disable must be 0 for Yokohama */
  if (!icm4x6xx_enable_int_async_reset(true)) {
    logger.Info("[%d]ICM42688::icm4x6xx_enable_int_async_reset() faild", id_);
    return false;
  }

  /* Set periodic reset mode for Yokohama */
  if (!icm4x6xx_enable_gyro_periodic_reset()) {
    logger.Info("[%d]ICM42688::icm4x6xx_enable_gyro_periodic_reset() faild", id_);
    return false;
  }

#endif
  /*
  * [HAVANA/YOKOHAMA] Disable aux pads(pin10&pin11) which are typically
  connected to OIS controller if applicable.
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

  if (!icm4x6xx_disable_aux_pins()) {
    logger.Info("[%d]ICM42688::icm4x6xx_disable_aux_pins() faild", id_);
    return false;
  }

  if (clkin_enable) {
    if (!icm4x6xx_set_accel_odr(ICM4X6XX_ODR_8000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_accel_odr() faild", id_);
      return false;
    }

    if (!icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_8000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_odr() faild", id_);
      return false;
    }
  } else {
    if (!icm4x6xx_set_accel_odr(ICM4X6XX_ODR_1000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_accel_odr() faild", id_);
      return false;
    }

    if (!icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_1000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_odr() faild", id_);
      return false;
    }
  }

  if (!icm4x6xx_set_accel_bandwidth(BW_ODR_DIV_5)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_bandwidth() faild", id_);
    return false;
  }

  if (!icm4x6xx_set_gyro_bandwidth(BW_ODR_DIV_5)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_bandwidth() faild", id_);
    return false;
  }

  if (!icm4x6xx_en_gyro(true)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_gyro() faild", id_);
    return false;
  }

  if (!icm4x6xx_set_accel_mode(accel_power_mode_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_mode() faild", id_);
    return false;
  }

  if (!icm4x6xx_en_fifo(en_a_fifo_, en_g_fifo_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_fifo() faild", id_);
    return false;
  }

  Timer::SleepUs(20000);

  // interrput_pin_.SetIRQDelegate(this);

  logger.Info("ICM42688::Start ok!\n");
  return true;
}

int ICM42688::DebugInit() { return DebugInit(true); }

int ICM42688::DebugInit(bool clkin_enable) {

  port_ = DeviceFactory::ShareInstance()->GetSPIDeviceById(id_,cs_,BAUDRATE_SPI);
  if (port_ == nullptr) {
    logger.Info("[ICM42688.cpp][ICM42688::Init][spi init fail!]");
  }
  has_inited_ = false;
  logger.Info("ICM42688::Start DebugInit(), clkin = %d\n", clkin_enable);
  gyro_fsr_ = GYRO_RANGE_2000DPS;
  accel_fsr_ = ACC_RANGE_16G;
  fifo_mode_ = STREAM;
  accel_bandwith_ = BW_LL_MAX_200_8X_ODR;
  accel_power_mode_ = ICM4X6XX_A_LNM;
  use_hi_res_ = false;
  sync_time_count_ = 0; // reset count to make a tiemstamp sync 
  /////////// Check Who Am I
  uint8_t id = 0;
  if (port_->ReadRegister(RegAddrNew::WHO_AM_I, &id)) {
    logger.Info("[%d]ICM42688::CheckWhoAmI(): error! cannot read WHO_AM_I id!", id_);
    return -1;
  }
  if (id != WHO_AM_I_ID) {
    logger.Info("[%d]ICM42688::CheckWhoAmI(): error! wanted ID: 0x%02X, got: 0x%02X\n", id_, WHO_AM_I_ID, id);
    return -2;
  }

  clkin_enable_ = clkin_enable;

  if (!icm4x6xx_config_ui_intf(SPI_INTF)) {
    logger.Info("[%d]ICM42688::icm4x6xx_config_ui_intf() faild", id_);
    return -3;
  }

  if (!icm4x6xx_enable_rtc_mode(clkin_enable)) {
    logger.Info("[%d]ICM42688::icm4x6xx_enable_rtc_mode(): faild", id_);
    return -4;
  }

#if 0
    /* Enable Timestamp field contains the measurement of time
     * since the last occurrence of ODR.*/
   if (!icm4x6xx_enable_delta_tmst(true))
   {
    logger.Info("[%d]ICM42688::icm4x6xx_enable_delta_tmst(): faild", id_);
     return false;
   }
#else

  if (clkin_enable) {
    /* Enable timestamp register */
    if (!icm4x6xx_enable_tmst(true)) {
      logger.Info("[%d]ICM42688::icm4x6xx_enable_tmst(): faild", id_);
      return -5;
    }
  }
#endif

  /*if (!icm4x6xx_en_int_push_pull(false)) {
    return false;
  }

  if (!icm4x6xx_en_int_latched_mode(false)) {
    return false;
  }

  if (!icm4x6xx_config_int_polarity(ICM4X6XX_INT_ACTIVE_HIGH)) {
    return false;
  }*/

  /* Choose big endian mode for fifo count and sensor data
   * default mode for FPGA and chip may be different,
   * so we choose one mode here for both */
  if (!icm4x6xx_en_big_endian_mode(true)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_big_endian_mode(): faild", id_);
    return -6;
  }

  /* enable fifo hold last data */
  /*if (!icm4x6xx_en_fifo_hold_last_data(false)) {
    return false;
  }*/

  /* do not tag fsync flag for temperature resolution */
  if (!icm4x6xx_config_fsync(0)) {
    logger.Info("[%d]ICM42688::icm4x6xx_config_fsync():faild", id_);
    return -7;
  }

  /* Choose Accel FSR */
  if (!icm4x6xx_set_accel_fsr(accel_fsr_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_fsr() faild", id_);
    return -8;
  }

  /* Choose Gyro FSR */
  if (!icm4x6xx_set_gyro_fsr(gyro_fsr_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_fsr() faild", id_);
    return -9;
  }

  /* Choose Accel filter order */
  if (!icm4x6xx_set_accel_filter_order(THIRD_ORDER))  // USE 3rd order filter
  {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_filter_order() faild", id_);
    return -10;
  }

  /* Choose Gyro filter order */
  if (!icm4x6xx_set_gyro_filter_order(THIRD_ORDER))  // USE 3rd order filter
  {
    logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_filter_order() faild", id_);
    return -11;
  }

  /* Enable fifo */
  if (!icm4x6xx_set_fifo_mode(fifo_mode_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_fifo_mode() faild", id_);
    return -12;
  }
#if 0
  /* Enable fifo overflow interrupt */
  if (!icm4x6xx_en_fifo_full_int(false)) {
    return false;
  }

  /* Enable high shock interrupt */
  if (!icm4x6xx_en_high_shock_int(false)) {
    return false;
  }

  /* Config highg parameter*/
  if (!icm4x6xx_config_highg_parameter()) {
    return false;
  }

  // TODO: Enable wm on IBI here, since we didn't need irq reg ready now

  /* Choose fifo count record or byte mode */
  // TODO: do test with both byte and record mode
  /* H_W_B 9450 pls don't use record mode for havana */
  if (!icm4x6xx_enable_record_mode(fifo_info_record_mode_))
  {
    return false;
  }
  /* The field int_asy_rst_disable must be 0 for Yokohama */
  if (!icm4x6xx_enable_int_async_reset(true)) {
    return false;
  }

  /* Set periodic reset mode for Yokohama */
  if (!icm4x6xx_enable_gyro_periodic_reset()) {
    return false;
  }

#endif
  /*
  * [HAVANA/YOKOHAMA] Disable aux pads(pin10&pin11) which are typically
  connected to OIS controller if applicable.
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

  if (clkin_enable) {
    icm4x6xx_enable_nflt_gyro(true);
  }

  if (!icm4x6xx_disable_aux_pins()) {
    logger.Info("[%d]ICM42688::icm4x6xx_disable_aux_pins() faild", id_);
    return -13;
  }

  if (clkin_enable) {
    if (!icm4x6xx_set_accel_odr(ICM4X6XX_ODR_8000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_accel_odr() faild", id_);
      return -14;
    }

    if (!icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_8000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_odr() faild", id_);
      return -15;
    }
  } else {
    if (!icm4x6xx_set_accel_odr(ICM4X6XX_ODR_1000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_accel_odr() faild", id_);
      return -14;
    }

    if (!icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_1000)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_odr() faild", id_);
      return -15;
    }
  }

  if (!icm4x6xx_set_accel_bandwidth(BW_ODR_DIV_5)) {
    logger.Info("[%d]ICM42688::icm4x6xx_set_accel_bandwidth() faild", id_);
    return -16;
  }

  if (clkin_enable) {
    if (!icm4x6xx_set_gyro_bandwidth(BW_ODR_DIV_2)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_bandwidth() faild", id_);
      return -17;
    }
  } else {
    if (!icm4x6xx_set_gyro_bandwidth(BW_ODR_DIV_2)) {
      logger.Info("[%d]ICM42688::icm4x6xx_set_gyro_bandwidth() faild", id_);
      return -17;
    }
  }

  if (!icm4x6xx_en_fifo(en_a_fifo_, en_g_fifo_)) {
    logger.Info("[%d]ICM42688::icm4x6xx_en_fifo() faild", id_);
    return -20;
  }

  if(!icm4x6xx_accel_gyro_powerup(ACCEL_GYRO_POWERUP)){
    logger.Info("[%d]ICM42688::icm4x6xx_accel_gyro_powerup() faild", id_);
    return -18;
  }

  if (!icm4x6xx_disable_afsr()) {
    logger.Info("[%d]ICM42688::icm4x6xx_disable_afsr() faild", id_);
    return -19;
  }

  // interrput_pin_.SetIRQDelegate(this);

  logger.Info("ICM42688::Start ok!\n");
  return 0;
}

bool ICM42688::Deinit() {
  if (port_ == nullptr) {
    return true;
  }

  has_inited_ = false;

  sync_time_count_ = 0; // reset count to make a tiemstamp sync 

  /////////// Check Who Am I
  uint8_t id = 0;
  if (port_->ReadRegister(RegAddrNew::WHO_AM_I, &id)) {
    logger.Info("ICM42688::CheckWhoAmI(): error! cannot read WHO_AM_I id!\n");
    return false;
  }

  if (id != WHO_AM_I_ID) {
    logger.Info(
        "ICM42688::CheckWhoAmI(): error! wanted ID: 0x%02X, got: 0x%02X\n",
        WHO_AM_I_ID, id);
    return false;
  }

  Timer::SleepUs(10000);
  if (port_->WriteRegister(
          RegAddrNew::DEVICE_CONFIG,
          0x01)) {  // after the software reset, it is in sleep mode
    logger.Info("ICM42688::WriteRegister Fail!");
    return false;
  }
  logger.Info("ICM42688::WriteRegister Success!");
  Timer::SleepUs(10000);

  return true;
}

void ICM42688::GetFIFODataFormat() {
  if (use_hi_res_ && (en_a_fifo_ || en_g_fifo_)) {
    desire_format_ = ICM4X6XX_FORMAT_20_BYTES;
  } else if (en_a_fifo_ && en_g_fifo_) {
    desire_format_ = ICM4X6XX_FORMAT_16_BYTES;
  } else if (en_a_fifo_) {
    desire_format_ = ICM4X6XX_FORMAT_ACCEL_8_BYTES;
  } else if (en_g_fifo_) {
    desire_format_ = ICM4X6XX_FORMAT_GYRO_8_BYTES;
  } else {
    desire_format_ = ICM4X6XX_FORMAT_EMPTY;
  }
}

bool ICM42688::icm4x6xx_enable_rtc_mode(bool enable) {
  if (!enable) {
    return true;
  }

  if (!icm4x6xx_set_reg_bank(1)) {
    return false;
  }

  if (!WriteMask(REG_INTF_CONFIG5, enable ? BIT_PIN9_FUNC_CLKIN : 0, BIT_PIN9_FUNC_MASK)) {
    return false;
  }

  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  if (!WriteMask(REG_INTF_CONFIG1, BIT_RTC_MODE_EN, BIT_RTC_MODE_EN)) {
    return false;
  }

  return true;
}

/**
 * @brief Enable or Disable data ready interrupt.
 *
 * @param[in] enable    Identify enable data ready interrupt or not.
 *                      true: enable data ready interrupt
 *                      false: disable data ready interrupt
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_en_dri(bool enable) {
#ifdef ICM4X6XX_USE_INT2
  rc += icm4x6xx_write_mask(instance, REG_INT_SOURCE3,
                            enable ? DRI_INT2_EN_MASK : 0, &xfer_bytes, false,
                            DRI_INT2_EN_MASK);
#else

  return WriteMask(REG_INT_SOURCE0, enable ? DRI_EN_MASK : 0, DRI_EN_MASK);

#endif
}

/**
 * @brief Enable or Disable Gyro.
 *
 * @param[in] enable    Identify enable gyro or not.
 *                      true: enable gyro
 *                      false: disable gyro
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_en_gyro(bool enable) {
  return WriteMask(REG_PWR_MGMT_0, enable ? GYRO_LNM_MASK : 0, GYRO_LNM_MASK);
}

/**
 * @brief Enable or Disable Accel.
 *
 * @param[in] enable    Identify enable accel or not.
 *                      true: enable accel
 *                      false: disable accel
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_accel_mode(icm4x6xx_power_mode mode) {
  return WriteMask(REG_PWR_MGMT_0, mode, ACCEL_LNM_MASK);
}

bool ICM42688::icm4x6xx_accel_gyro_powerup(uint8_t mode){
  bool ret ; 
  ret = WriteMask(REG_PWR_MGMT_0, mode, ACCEL_LNM_MASK|GYRO_LNM_MASK);
  if(ret){
    Timer::SleepUs(20000);
  }
  return ret;
}

bool ICM42688::icm4x6xx_disable_aux_pins() {
  uint8_t reg_value = 0;

  if (!icm4x6xx_set_reg_bank(2)) {
    return false;
  }

  reg_value = 0x01;

  if (port_->WriteRegister(static_cast<uint8_t>(0x70), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (port_->WriteRegister(static_cast<uint8_t>(0x71), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (port_->WriteRegister(static_cast<uint8_t>(0x72), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (port_->WriteRegister(static_cast<uint8_t>(0x73), reg_value)) {
    return false;
  }
  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  return true;
}

bool ICM42688::icm4x6xx_enable_gyro_periodic_reset() {
  if (!icm4x6xx_set_reg_bank(3)) {
    return false;
  }

  if (!WriteMask(REG_AMP_GSXYZ_TRIM0, 0, BIT_GYRO_SC2V_CONT_MODE)) {
    return false;
  }

  if (!WriteMask(REG_AMP_GX_TRIM2, 0, BIT_GX_SC2V_FET_TRIM_MASK)) {
    return false;
  }

  if (!WriteMask(REG_AMP_GY_TRIM2, 0, BIT_GY_SC2V_FET_TRIM_MASK)) {
    return false;
  }

  if (!WriteMask(REG_AMP_GZ_TRIM2, 0, BIT_GZ_SC2V_FET_TRIM_MASK)) {
    return false;
  }

  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  return true;
}

/**
 * @brief enable or disable Asynchronous reset for Interrupt
 *
 * @param[in] instance    point to sensor instance
 * @param[in] enable      flag to identify enable or disable
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_enable_int_async_reset(bool enable) {
  return WriteMask(REG_INT_CONFIG1, !enable ? BIT_INT_ASY_RST_DIS_MASK : 0,
                   BIT_INT_ASY_RST_DIS_MASK);
}

/**
 * @brief set fifo watermark, wm_th should be calculated by current fifo
 * format
 *
 * @param[in] wm_th    FIFO watermark, user should calculate it
 *                      with current fifo packet format
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_enable_record_mode(bool enable) {
  return WriteMask(REG_INTF_CONFIG0, enable ? RECORD_MODE_MASK : 0,
                   RECORD_MODE_MASK);
}

bool ICM42688::icm4x6xx_config_highg_parameter() {
  uint8_t reg_value = 0;

  if (!icm4x6xx_set_reg_bank(4)) {
    return false;
  }

  reg_value = ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2844MG |
              ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_20MS;

  if (port_->WriteRegister(static_cast<uint8_t>(REG_APEX_CONFIG6),
                            reg_value)) {
    return false;
  }

  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  return true;
}

/**
 * @brief Set register bank
 *
 * @param[in] bank_num  Register bank number
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_reg_bank(uint8_t bank_num) {
  if (port_->WriteRegister(static_cast<uint8_t>(REG_BANK_SEL), bank_num)) {
    return false;
  }
  return true;
}

bool ICM42688::icm4x6xx_en_high_shock_int(bool enable) {
  if (!icm4x6xx_set_reg_bank(4)) {
    return false;
  }

  if (!WriteMask(REG_INT_SOURCE6, enable ? BIT_HIGHG_DET_INT1_EN : 0,
                 BIT_HIGHG_DET_INT1_EN)) {
    return false;
  }

  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  return true;
}

/**
 * @brief enable/disable  accel or gyro fifo
 *
 * @param[in] en_accel    Identify enable accel fifo or not.
 *                        true: enable accel fifo, write accel data into fifo
 *                        false: disable accel fifo
 * @param[in] en_gyro     Identify enable gyro fifo or not.
 *                        true: enable gyro fifo, write gyro data into fifo
 *                        false: disable gyro fifo
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_en_fifo(bool en_accel, bool en_gyro) {
  uint8_t bit_mask = 0;
  uint8_t reg = 0;

  bit_mask = FIFO_ACCEL_EN_MASK | FIFO_GYRO_EN_MASK | FIFO_TEMP_EN_MASK |
             FIFO_TMST_FSYNC_EN_MASK | FIFO_HIRES_EN_MASK;

  reg = (en_accel ? FIFO_ACCEL_EN_MASK : 0) |
        (en_gyro ? FIFO_GYRO_EN_MASK : 0) |
        ((en_accel || en_gyro) ? FIFO_TEMP_EN_MASK : 0) |
        //((en_accel || en_gyro) ? FIFO_TMST_FSYNC_EN_MASK : 0) |
        (use_hi_res_ ? ((en_accel || en_gyro) ? FIFO_HIRES_EN_MASK : 0) : 0);

  if (!WriteMask(REG_FIFO_CONFIG_1, reg, bit_mask)) {
    return false;
  }

  return true;
}

/**
 * @brief Enable or Disable fifo overflow interrupt.
 *
 * @param[in] enable    Identify enable fifo overflow interrupt or not.
 *                      true: enable fifo overflow interrupt
 *                      false: disable fifo overflow interrupt
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_en_fifo_full_int(bool enable) {
  return WriteMask(REG_INT_SOURCE0, enable ? FIFO_FULL_EN_MASK : 0,
                   FIFO_FULL_EN_MASK);
}

/**
 * @brief choose fifo working mode
 *
 * @param[in] fifo_mode    fifo working mode
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_fifo_mode(icm4x6xx_fifo_mode fifo_mode) {
  return WriteMask(REG_FIFO_CONFIG, fifo_mode << BIT_FIFO_MODE_SHIFT,
                   BIT_FIFO_MODE_CTRL_MASK);
}
/**
 * @brief set gyro bandwidth
 *
 * @param[in] order    choose gyro bandwidth
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_gyro_bandwidth(icm4x6xx_bandwidth bw) {
  return WriteMask(REG_GYRO_ACCEL_CONFIG0, bw, BIT_GYRO_BW_MASK);
}

/**
 * @brief set accel bandwidth
 *
 * @param[in] order    choose accel bandwidth
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_accel_bandwidth(icm4x6xx_bandwidth bw) {
  return WriteMask(REG_GYRO_ACCEL_CONFIG0, bw << BIT_ACCEL_BW_SHIFT,
                   BIT_ACCEL_BW_MASK);
}

/**
 * @brief set gyro filter order
 *
 * @param[in] order    choose gyro filter order
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_gyro_filter_order(icm4x6xx_filter_order order) {
  return WriteMask(REG_GYRO_CONFIG1, order << BIT_GYRO_FILT_ORD_SHIFT,
                   BIT_GYRO_FILT_ORD_MASK);
}

/**
 * @brief set accel filter order
 *
 * @param[in] order    choose accel filter order
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_accel_filter_order(icm4x6xx_filter_order order) {
  return WriteMask(REG_ACC_CONFIG1, order << BIT_ACC_FILT_ORD_SHIFT,
                   BIT_ACC_FILT_ORD_MASK);
}

/**
 * @brief Config Gyro FSR
 *
 * @param[in] fsr    the FSR of Gyro to be set.
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_gyro_fsr(icm4x6xx_gyro_fsr fsr) {
  return WriteMask(REG_GYRO_CONFIG0, fsr << GYRO_FSR_SHIFT, GYRO_FSR_MASK);
}

bool ICM42688::ReadWhoAmI(uint8_t* who_am_i) {
  return port_->ReadRegister(static_cast<uint8_t>(RegAddrNew::WHO_AM_I),
                             who_am_i);
}

/**
 * @brief Config Accel FSR
 *
 * @param[in] fsr    the FSR of Accel to be set.
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_accel_fsr(icm4x6xx_accel_fsr fsr) {
  return WriteMask(REG_ACCEL_CONFIG0, fsr << ACCEL_FSR_SHIFT, ACCEL_FSR_MASK);
}

/**
 * @brief config fsync
 *
 * @param[in] data, bit [4:6]: FSYNC_UI_SEL
 *                      [2]: FSYNC_AUX1_FLAG_CLEAR_SEL
 *                      [1]: FSYNC_UI_FLAG_CLEAR_SEL
 *                      [0]: FSYNC_POLARITY
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_config_fsync(uint8_t data) {
  return WriteMask(REG_FSYNC_CONFIG, data, 0x70);
}
//////////////////////////////////////////////////////
bool ICM42688::icm4x6xx_en_fifo_hold_last_data(bool enable) {
  return WriteMask(REG_INTF_CONFIG0, enable ? FIFO_HOLD_LAST_DATA_EN : 0,
                   FIFO_HOLD_LAST_DATA_EN);
}

/**
 * @brief config ui interface
 *
 * @param[in] instance, point to sensor instance
 *       [in] intf, the specific interface to be used
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_config_ui_intf(icm4x6xx_ui_intf intf) {
  return WriteMask(REG_INTF_CONFIG0, intf, UI_INTF_MASK);
}

bool ICM42688::icm4x6xx_en_int_push_pull(bool enable) {
  uint8_t push_pull_mask = INT1_PUSH_PULL_MASK;
  return WriteMask(REG_INT_CONFIG, enable ? push_pull_mask : 0,
                   INT1_PUSH_PULL_MASK);
}

bool ICM42688::icm4x6xx_en_int_latched_mode(bool enable) {
  uint8_t latch_mode_mask = INT1_LATCHED_MODE_MASK;

  return WriteMask(REG_INT_CONFIG, enable ? latch_mode_mask : 0,
                   latch_mode_mask);
}

bool ICM42688::icm4x6xx_enable_nflt_gyro(bool enable) {
  uint8_t tmst_config_reg = 0x0B;
    if (!icm4x6xx_set_reg_bank(1)) {
    return false;
  }
  return WriteMask(tmst_config_reg,
                   0x03,
                   0x03);
}

bool ICM42688::icm4x6xx_enable_nflt_acc(bool enable) {
  uint8_t tmst_config_reg = 0x03;
    if (!icm4x6xx_set_reg_bank(2)) {
    return false;
  }
  return WriteMask(tmst_config_reg,
                   0x01,
                   0x01);
}

/**
 * @brief Config int polarity.
 *
 * @param[in] enable    Identify enable int latched mode or not.
 *                   true: enable int active high
 *                   false: enable int active low
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_config_int_polarity(icm4x6xx_int_polarity polarity) {
  uint8_t polarity_mask = INT1_ACTIVE_HIGH_MASK;

  return WriteMask(REG_INT_CONFIG,
                   (ICM4X6XX_INT_ACTIVE_HIGH == polarity) ? polarity_mask : 0,
                   polarity_mask);
}

/**
 * @brief Enable or Disable fifo count and sensor data big endian mode.
 *
 * @param[in] enable    Identify enable big endian or not.
 *                      true: enable fifo count and sensor data big endian
 * mode false: disable fifo count and sensor data big endian mode
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_en_big_endian_mode(bool enable) {
  uint8_t bit_mask = FIFO_COUNT_BIG_ENDIAN_MASK | SENSOR_DATA_BIG_ENDIAN_MASK;
  uint8_t reg = enable ? bit_mask : 0;

  return WriteMask(REG_INTF_CONFIG0, reg, bit_mask);
}

bool ICM42688::WriteMask(uint32_t reg_addr, uint8_t reg_value,
                            uint8_t mask) {
  uint8_t rw_buffer = 0;

  if (port_->ReadRegister(reg_addr, &rw_buffer)) {
    return false;
  }

  /* generate new value */
  rw_buffer = (rw_buffer & (~mask)) | (reg_value & mask);

  /* write new value to this register */
  if (port_->WriteRegister(reg_addr, rw_buffer)) {
    return false;
  }

  // logger.Info("ICM42688::ok!\n");
  return true;
}

/**
 * @brief Time Stamp delta Enable
 *
 * @param[in] enable   enable or disable delta_tmst
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_enable_delta_tmst(bool enable) {
  uint8_t tmst_config_reg = REG_TMST_CONFIG_REG;
  return WriteMask(tmst_config_reg,
                   enable ? (BIT_TMST_DELTA_EN | BIT_TMST_TO_REGS_EN) : 0,
                   BIT_TMST_DELTA_EN | BIT_TMST_TO_REGS_EN);
}

bool ICM42688::icm4x6xx_disable_afsr() {
  return WriteMask(0x4D, 0x40, 0xC0);
}

/**
 * @brief enable timestamp register
 *
 * @param[in] enable   enable or disable timestamp register
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_enable_tmst(bool enable) {
  uint8_t tmst_config_reg = REG_TMST_CONFIG_REG;

  /* Choose suitable TMST register address for
   * different chip */

  // TODO: CHECK THIS for which is needed for fifo format change

  return WriteMask(tmst_config_reg,
                   enable ? BIT_TMST_TO_REGS_EN | BIT_TMST_EN : 0,
                   BIT_TMST_TO_REGS_EN | BIT_TMST_EN);
}

/**
 * @brief enable 20-bit timestamp reading
 *
 * @param[in] enable   enable or disable 20-bit timestamp reading
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_enable_tmst_val(bool enable) {
  return WriteMask(REG_SIGNAL_PATH_RESET_REG, enable ? BIT_TMST_STROBE : 0,
                   BIT_TMST_STROBE);
}

bool ICM42688::icm4x6xx_read_tmst_val(uint32_t* tmst_reg) {
  uint8_t buff[3] = {0};

  if (!icm4x6xx_set_reg_bank(1)) {
    return false;
  }

  if (!ReadBlock(REG_TMSTVAL0, buff, sizeof(buff))) {
    return false;
  }

  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  *tmst_reg = (uint32_t)((buff[2] & 0x0f) << 16 | (buff[1] << 8) | buff[0]);

  return true;
}

/**
 * @brief Get current packet size, according current FIFO format
 *
 * @param[in] instance, point to sensor instance
 * @param[in] size, store currect packet size
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_get_packet_size(uint8_t* size) {
  uint8_t packet_size = 0;
  bool result = true;

  // logger.Info("desire_format_ = %d\n", desire_format_ );  //lmy_asi

  if (desire_format_ == ICM4X6XX_FORMAT_20_BYTES)
    packet_size = 20;
  else if (desire_format_ == ICM4X6XX_FORMAT_16_BYTES)
    packet_size = 16;
  else if (desire_format_ == ICM4X6XX_FORMAT_ACCEL_8_BYTES ||
           desire_format_ == ICM4X6XX_FORMAT_GYRO_8_BYTES)
    packet_size = 8;
  else if (desire_format_ == ICM4X6XX_FORMAT_EMPTY) {
    // ICM4X6XX_INST_PRINTF(HIGH, instance, "fifo disabled");
    packet_size = 0;
  } else {
    // ICM4X6XX_INST_PRINTF(ERROR, instance, "incorrect ff format");
    result = false;
  }

  *size = packet_size;

  return result;
}

/**
 * @brief read fifo count.
 *
 * @param[out] count point to the value of
 *                   fifo count
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_read_fifo_count(uint16_t* count) {
  uint8_t buff[2];
  uint16_t max_count = 0;

  if (port_->ReadRegisters(REG_FIFO_BYTE_COUNT_L, buff, 2)) {
    return false;
  }

  *count = (uint16_t)(buff[0] << 8 | buff[1]);

  /* According DS 6.3 MAXIMUM FIFO STORAGE
   * the largest size FIFO size is 2080 bytes*/
  max_count = ICM4X6XX_YOKOHAMA_MAX_FIFO_SIZE;

  if (*count > max_count) {
    // ICM4X6XX_INST_PRINTF(ERROR, instance,
    //    "FF c %d", *count);
    *count = max_count;
  }

  return true;
}

bool ICM42688::icm4x6xx_is_fifo_format_match(icm4x6xx_fifo_format format) {
  uint8_t fifo_header = 0;
  uint8_t value_20bit =
      FIFO_HEADER_A_BIT | FIFO_HEADER_G_BIT | FIFO_HEADER_20_BIT;
  uint8_t value_16bit = FIFO_HEADER_A_BIT | FIFO_HEADER_G_BIT;
  icm4x6xx_fifo_format cur_format;

  if (!icm4x6xx_read_fifo_buf(&fifo_header, 1)) {
    return false;
  }

  // ICM4X6XX_INST_PRINTF(MED,instance, "ff header %#x", fifo_header);

  if (fifo_header & FIFO_HEADER_EMPTY_BIT)
    cur_format = ICM4X6XX_FORMAT_EMPTY;
  else if ((fifo_header & value_20bit) == value_20bit)
    cur_format = ICM4X6XX_FORMAT_20_BYTES;
  else if ((fifo_header & value_16bit) == value_16bit)
    cur_format = ICM4X6XX_FORMAT_16_BYTES;
  else if (fifo_header & FIFO_HEADER_A_BIT)
    cur_format = ICM4X6XX_FORMAT_ACCEL_8_BYTES;
  else if (fifo_header & FIFO_HEADER_G_BIT)
    cur_format = ICM4X6XX_FORMAT_GYRO_8_BYTES;
  else {
    cur_format = ICM4X6XX_FORMAT_UNKNOWN;
    // CM4X6XX_INST_PRINTF(ERROR, instance, "unknown header 0x%x", fifo_header);
  }

  if (cur_format == format || cur_format == ICM4X6XX_FORMAT_EMPTY)
    return true;
  else
    return false;
}

uint32_t ICM42688::icm4x6xx_cal_valid_fifo_len(const uint8_t* buf,
                                                  uint32_t buf_len,
                                                  uint16_t* cnt) {
  icm4x6xx_fifo_header_t header;
  uint32_t valid_buf_len = 0;

  /* Calculator valid FIFO buf length */
  while (valid_buf_len < buf_len) {
    header.head_byte = buf[valid_buf_len];
    if (header.head_byte == 0xff || header.head_byte == 0x80 ||
        header.head_byte == 0x00) {
      break;
    }
    valid_buf_len++;
    if (header.bits.accel_bit) {
      valid_buf_len += 6;
    }

    if (header.bits.gyro_bit) {
      valid_buf_len += 6;
    }

    valid_buf_len += header.bits.twentybits_bit ? 2 : 1;  // temperature bytes

    if (header.bits.timestamp_bit & 0x02) {
      valid_buf_len += 2;
    }

    if (header.bits.twentybits_bit) {
      valid_buf_len += 3;
    }
    (*cnt)++;
  }

  return valid_buf_len;
}

uint32_t ICM42688::get_offset(uint16_t early, uint16_t later) {
  uint32_t offset = 0;

  if (later < early) {
    offset = 0xffff - early + 1 + later;
  } else {
    offset = later - early;
  }

  return offset;
}


bool ICM42688::Read(IMURawData& data) {
  return false; //never use
}



// static void timer_period_monitor(void) {
//   static uint16_t i = 0;
//   static uint64_t time_latst_test = 0;
//   uint64_t time_gap_ = 0;

//   uint64_t dsp_time = Timer::Now();
//   time_gap_ = dsp_time - time_latst_test;

//   // can't work
//   // logger.Info("call %d,dsp_time %ld,time_latst_test %ld,time_gap_ %ld\n",i,dsp_time,time_latst_test,time_gap_);

//   // can't work
//   // logger.Info("call %d,dsp_time %d",i,dsp_time);
//   // logger.Info("call %d,time_latst_test %d",i,time_latst_test);
//   // logger.Info("call %d,time_gap_ %d",i,time_gap_);

//   // can't work
//   // logger.Info("call %d",i);
//   // logger.Info("dsp_time %d,time_latst_test %d,time_gap_ %d",dsp_time,time_latst_test,time_gap_);

//   // this will work
//   logger.Info("call %d", i);
//   logger.Info("dsp_time %d", dsp_time);
//   logger.Info("time_latst_test %d", time_latst_test);
//   logger.Info("time_gap_ %d", time_gap_);

//   time_latst_test = dsp_time;
//   i++;
// }

bool ICM42688::ReadRaw(IMURawData& data) {
  uint64_t dsp_time = Timer::Now();
  data.is_need_cali_time = false;

  int16_t accel[3] = {0};
  int16_t gyro[3] = {0};

  if (clkin_enable_) {
    bool need_sync_time = (sync_time_count_ >= 100) ? false : true;

    if (need_sync_time) {
      sync_time_count_++;
      if (!icm4x6xx_enable_tmst_val(true)) {
        logger.Info("id[%d]: icm4x6xx_enable_tmst_val fail\n", id_);
        return false;
      }
    }

    data.id = 0x47;  // ffc need it //id_;
    data.index = id_;
   
    /*
    1.get chip time
    2.get chip odr time
    3.get odr offset time = chip time - chip odr time
    2.odr time  = dsptime  - odr offset time
    */
    uint32_t intl_cnt_20b = 0;
    uint16_t fifo_count = 0;
    uint16_t bytes_to_read = 0;
    uint8_t packet_size = 0;
 
    int16_t temperture = 0;
    
    uint32_t fifo_timestamp = 0;

    if (!icm4x6xx_get_packet_size(&packet_size)) {
      logger.Info("id[%d]: icm4x6xx_get_packet_size fail\n", id_);
      return false;
    }
    // logger.Info("icm4x6xx_get_packet_size = %d\n", packet_size);
    if (!icm4x6xx_read_fifo_count(&fifo_count)) {
      logger.Info("id[%d]: icm4x6xx_read_fifo_count fail\n", id_);
      return false;
    }
    // logger.Info("fifo_count = %d\n", fifo_count);

    if (fifo_count == 0) {
      logger.Info("id[%d]: fifo_count = 0\n", id_);
      return false;
    }

    bytes_to_read = fifo_count;

    if (bytes_to_read > (packet_size * MAX_SCP_SAMPLES)) {
      bytes_to_read = (packet_size * MAX_SCP_SAMPLES);
    }

    uint8_t buf[bytes_to_read];
    if (!icm4x6xx_read_fifo_buf(buf, bytes_to_read)) {
      logger.Info("id[%d]: icm4x6xx_read_fifo_buf(buf, bytes_to_read) fail\n", id_);
      return false;
    }

    uint16_t packet_cnt = 0;
    uint32_t valid_buf_len = icm4x6xx_cal_valid_fifo_len(buf, bytes_to_read, &packet_cnt);

    // logger.Info("(valid_buf_len == %d || packet_cnt == %d) ok\n",
    // valid_buf_len,
    //          packet_cnt);

    // ICM4X6XX_INST_PRINTF(LOW, instance, "valid buf_len/packet_cnt %d %d",
    // valid_buf_len, packet_cnt);
    if (valid_buf_len == 0 || packet_cnt == 0) {
      // ICM4X6XX_INST_PRINTF(HIGH, instance, "no valid senor data");
      // goto CLEAN_STATUS;

      logger.Info("id[%d]: (valid_buf_len == 0 || packet_cnt == 0) fail, fifo_count = %d", id_, fifo_count);
      return false;
    }

    if (packet_cnt != valid_buf_len / packet_size) {
      logger.Info("id[%d]: (packet_cnt != valid_buf_len / packet_size) fail", id_);
      return false;
    }

    uint16_t index = packet_cnt - 1;
    uint8_t* p = buf;

    temperture = p[index * packet_size + 0x0D];

    fifo_timestamp = p[index * packet_size + 0x0E] * 256 + p[index * packet_size + 0x0F];

    index = 0;

    accel[0] = p[index * packet_size + 0x01] * 256 + p[index * packet_size + 0x02];
    accel[1] = p[index * packet_size + 0x03] * 256 + p[index * packet_size + 0x04];
    accel[2] = p[index * packet_size + 0x05] * 256 + p[index * packet_size + 0x06];
    gyro[0] = p[index * packet_size + 0x07] * 256 + p[index * packet_size + 0x08];
    gyro[1] = p[index * packet_size + 0x09] * 256 + p[index * packet_size + 0x0A];
    gyro[2] = p[index * packet_size + 0x0B] * 256 + p[index * packet_size + 0x0C];

    /*
      temperture = p[index * packet_size + 0x0D];

      fifo_timestamp =
          p[index * packet_size + 0x0E] * 256 + p[index * packet_size + 0x0F];
      // logger.Info("ICM42688::fifo_timestamp = %d\n", fifo_timestamp);
    }*/

    if (need_sync_time) {
      if (!icm4x6xx_read_tmst_val(&intl_cnt_20b)) {
        logger.Info("id[%d]: icm4x6xx_read_tmst_val(&intl_cnt_20b) fail\n", id_);
        return false;
      }

      // logger.Info("Timer::Now() = %lld\n", Timer::Now());
      // logger.Info("ICM42688::intl_cnt_20b = %d\n", intl_cnt_20b);

      ////////////////////////////////////
      //  uint32_t offset_before = 0;
      // if ((intl_cnt_20b & 0xffff) < fifo_timestamp) {
      //   offset_before = 0xffff - fifo_timestamp + 1 + (intl_cnt_20b &
      //   0xffff);
      // } else {
      //   offset_before = (intl_cnt_20b & 0xffff) - fifo_timestamp;
      // }

      // // 1024/1000: 976-> 1000
      // offset_before = offset_before * 1024 / 1000 ;

      // if (offset_before < odr_time_offset_before) {
      //   odr_time_offset_before = offset_before;
      // }

      ////////////////////////////////////////////
      

      /*
      uint32_t offset = get_offset(fifo_timestamp, (intl_cnt_20b & 0xffff));

      // 1024/1000: 976-> 1000
      offset = offset * 1024 / 1000;

      if (offset < odr_time_offset_) {
        odr_time_offset_ = offset;
      }

      // 2700us ODR delay
      dsp_fifo_timestamp_ = dsp_time - 2700 - (odr_time_offset_);

      last_fifo_timestamp_16b_ = fifo_timestamp;*/

      data.is_need_cali_time  = true;
      data.chip_cali_fifo_timestamp = intl_cnt_20b & 0xffff;

    } else {
      /*uint32_t fifo_timestamp_offset =
          get_offset(last_fifo_timestamp_16b_, fifo_timestamp);

      dsp_fifo_timestamp_ += (double)fifo_timestamp_offset * 1.024;

      last_fifo_timestamp_16b_ = fifo_timestamp;*/
    }

    data.timestamp_us = dsp_time;   //(uint64_t)dsp_fifo_timestamp_;
    // logger.Info("%d,%lld,%lld,%lld,%lld", odr_time_offset_,
    //            data.timestamp_us - last_odr_timestamp_, data.timestamp_us,
    //            before, (int64_t)before - (int64_t)data.timestamp_us);
    // logger.Info("====================ICM42688::data.timestamp_us =
    // %lld\n", data.timestamp_us);

    data.temperature = (float)temperture / 2.07 + TEMPERATURE_OFFSET;

    data.accel[0] = accel[0] * ACCEL_SCALER;
    data.accel[1] = accel[1] * ACCEL_SCALER;
    data.accel[2] = accel[2] * ACCEL_SCALER;

    data.raw_accel[0] = accel[0];
    data.raw_accel[1] = accel[1];
    data.raw_accel[2] = accel[2];

    data.gyro[0] = gyro[0] * GYRO_SCALER;
    data.gyro[1] = gyro[1] * GYRO_SCALER;
    data.gyro[2] = gyro[2] * GYRO_SCALER;

    data.raw_gyro[0] = gyro[0];
    data.raw_gyro[1] = gyro[1];
    data.raw_gyro[2] = gyro[2];
    // last_odr_timestamp_ = data.timestamp_us;

    if (packet_cnt > data.MAX_PACKET_COUNT) {
      packet_cnt = data.MAX_PACKET_COUNT;
    }

    data.fifo_count = packet_size * packet_cnt;
    data.packet_count = packet_cnt;

    // logger.Info("packet_cnt = %d, packet_size = %d , data.fifo_count = %d\n",
    // packet_cnt, packet_size, data.fifo_count);  //lmy_asi

    memcpy(data.fifo_data, p, data.fifo_count);

  } else {
    data.id = 0x47;  // ffc need it //id_;
    data.index = id_;

    uint16_t fifo_count = 0;
    uint16_t bytes_to_read = 0;
    uint8_t packet_size = 0;
    // int16_t accel[3] = {0};
    int16_t temperture = 0;
    // int16_t gyro[3] = {0};

    if (!icm4x6xx_get_packet_size(&packet_size)) {
      logger.Info("id[%d]: icm4x6xx_get_packet_size fail\n", id_);
      return false;
    }

    // logger.Info("icm4x6xx_get_packet_size = %d\n", packet_size);
    if (!icm4x6xx_read_fifo_count(&fifo_count)) {
      logger.Info("id[%d]: icm4x6xx_read_fifo_count fail\n", id_);
      return false;
    }
    // logger.Info("fifo_count = %d\n", fifo_count);

    if (fifo_count == 0) {
      // logger.Info("id[%d]: fifo_count == 0\n", id_);
      return false;
    }

    bytes_to_read = fifo_count;

    if (bytes_to_read > (packet_size * MAX_SCP_SAMPLES)) {
      bytes_to_read = (packet_size * MAX_SCP_SAMPLES);
    }

    uint8_t buf[bytes_to_read];
    if (!icm4x6xx_read_fifo_buf(buf, bytes_to_read)) {
      logger.Info("id[%d]: icm4x6xx_read_fifo_buf(buf, bytes_to_read) fail, fifo_count = %d", id_, fifo_count);
      return false;
    }

    uint16_t packet_cnt = 0;
    uint32_t valid_buf_len =
        icm4x6xx_cal_valid_fifo_len(buf, bytes_to_read, &packet_cnt);

    // logger.Info("(valid_buf_len == %d || packet_cnt == %d) ok\n",
    // valid_buf_len,
    //          packet_cnt);

    // ICM4X6XX_INST_PRINTF(LOW, instance, "valid buf_len/packet_cnt %d %d",
    // valid_buf_len, packet_cnt);
    if (valid_buf_len == 0 || packet_cnt == 0) {
      // ICM4X6XX_INST_PRINTF(HIGH, instance, "no valid senor data");
      // goto CLEAN_STATUS;

      logger.Info("id[%d]: (valid_buf_len == 0 || packet_cnt == 0) fail, fifo_count = %d", id_, fifo_count);
      return false;
    }

    if (packet_cnt != valid_buf_len / packet_size) {
      logger.Info("id[%d]: (packet_cnt != valid_buf_len / packet_size) fail", id_);
      return false;
    }

    uint16_t index = packet_cnt - 1;
    uint8_t* p = buf;

    temperture = p[index * packet_size + 0x0D];

    index = 0;
    accel[0] = p[index * packet_size + 0x01] * 256 + p[index * packet_size + 0x02];
    accel[1] = p[index * packet_size + 0x03] * 256 + p[index * packet_size + 0x04];
    accel[2] = p[index * packet_size + 0x05] * 256 + p[index * packet_size + 0x06];
    gyro[0] = p[index * packet_size + 0x07] * 256 + p[index * packet_size + 0x08];
    gyro[1] = p[index * packet_size + 0x09] * 256 + p[index * packet_size + 0x0A];
    gyro[2] = p[index * packet_size + 0x0B] * 256 + p[index * packet_size + 0x0C];
    /*  temperture = p[index * packet_size + 0x0D];

      fifo_timestamp =
          p[index * packet_size + 0x0E] * 256 + p[index * packet_size + 0x0F];
      // logger.Info("ICM42688::fifo_timestamp = %d\n", fifo_timestamp);
    }*/

    // uint64_t before = dsp_time - 2700 - (odr_time_offset_before);
    data.timestamp_us = dsp_time;
    // logger.Info("%d,%lld,%lld,%lld,%lld", odr_time_offset_,
    //            data.timestamp_us - last_odr_timestamp_, data.timestamp_us,
    //            before, (int64_t)before - (int64_t)data.timestamp_us);
    // logger.Info("====================ICM42688::data.timestamp_us =
    // %lld\n", data.timestamp_us);

    data.accel[0] = accel[0] * ACCEL_SCALER;
    data.accel[1] = accel[1] * ACCEL_SCALER;
    data.accel[2] = accel[2] * ACCEL_SCALER;

    data.raw_accel[0] = accel[0];
    data.raw_accel[1] = accel[1];
    data.raw_accel[2] = accel[2];

    data.gyro[0] = gyro[0] * GYRO_SCALER;
    data.gyro[1] = gyro[1] * GYRO_SCALER;
    data.gyro[2] = gyro[2] * GYRO_SCALER;

    data.raw_gyro[0] = gyro[0];
    data.raw_gyro[1] = gyro[1];
    data.raw_gyro[2] = gyro[2];

    data.temperature = (float)temperture / 2.07 + TEMPERATURE_OFFSET;
    if (packet_cnt > data.MAX_PACKET_COUNT) {
      packet_cnt = data.MAX_PACKET_COUNT;
    }
    data.fifo_count = packet_size * packet_cnt;
    data.packet_count = packet_cnt;

    // logger.Info("packet_cnt = %d, packet_size = %d , data.fifo_count = %d\n",
    // packet_cnt, packet_size, data.fifo_count);  //lmy_asi

    memcpy(data.fifo_data, p, data.fifo_count);
  }

  // this->Notify(data);
  return true;
}

int ICM42688::ReadAccel(int16_t accel_raw_data[3]) {
  uint8_t buffer[6];
  int r = ReadBlock(static_cast<uint8_t>(RegAddrNew::ACCEL_DATA_X1), buffer, 6);
  accel_raw_data[0] = buffer[0] * 256 + buffer[1];
  accel_raw_data[1] = buffer[2] * 256 + buffer[3];
  accel_raw_data[2] = buffer[4] * 256 + buffer[5];
  return r;
}

int ICM42688::ReadGyro(int16_t gyro_raw_data[3]) {
  uint8_t buffer[6];
  int r = ReadBlock(static_cast<uint8_t>(RegAddrNew::GYRO_DATA_X1), buffer, 6);
  gyro_raw_data[0] = buffer[0] * 256 + buffer[1];
  gyro_raw_data[1] = buffer[2] * 256 + buffer[3];
  gyro_raw_data[2] = buffer[4] * 256 + buffer[5];
  return r;
}

int ICM42688::ReadTemp(int16_t& temp_raw_data) {
  uint8_t buffer[2];
  int r = ReadBlock(static_cast<uint8_t>(RegAddrNew::TEMP_DATA1), buffer, 2);
  temp_raw_data = buffer[0] * 256 + buffer[1];
  return r;
}

bool ICM42688::WriteByte(uint8_t reg, uint8_t val) {
  if (port_->WriteRegister(static_cast<uint8_t>(reg), val)) {
    return false;
  }
  Timer::SleepMs(5);
  return true;
}

bool ICM42688::ReadBlock(uint8_t first_reg, uint8_t buf[], int len) {
  if (port_->ReadRegisters(static_cast<uint8_t>(first_reg), buf, len)) {
    return false;
  }

  return true;
}

/**
 * @brief Read fifo data
 *
 * @param[out] buf point to fifo data.
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_read_fifo_buf(uint8_t* buf, uint32_t len) {
  return ReadBlock(REG_FIFO_DATA, buf, len);
}
void ICM42688::ExecutePeriodically() {
  if (!this->ReadRaw(internal_raw_data_)) {
    // logger.Error("id[%d]: ReadRawRead IMU failed., %ld", id_, err_cnt_++);
    return;
  }

  this->Notify(internal_raw_data_);
}

/**
 * @brief Convert ODR to register value
 *
 * @param[in] odr  the desired odr.
 * @param[out] odr_reg  the converted odr register value.
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_odr_to_reg_val(float odr,
                                          icm4x6xx_sensor_odr* odr_reg) {
  if (odr >= (uint16_t)ICM4X6XX_ODR_8000)
    *odr_reg = ODR_8KHZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_2000)
    *odr_reg = ODR_2KHZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_1000)
    *odr_reg = ODR_1KHZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_500)
    *odr_reg = ODR_500HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_200)
    *odr_reg = ODR_200HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_100)
    *odr_reg = ODR_100HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_50)
    *odr_reg = ODR_50HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_25)
    *odr_reg = ODR_25HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_12_5)
    *odr_reg = ODR_12_5HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_6_25)
    *odr_reg = ODR_6_25HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_3_125)
    *odr_reg = ODR_3_125HZ;
  else if (odr >= (uint16_t)ICM4X6XX_ODR_1_5625)
    *odr_reg = ODR_1_5625HZ;
  else
    *odr_reg = ODR_12_5HZ;

  return true;
}

/**
 * @brief Set Accel ODR
 *
 * @param[in] odr    the Accel ODR will be set.
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_accel_odr(float odr) {
  icm4x6xx_sensor_odr odr_reg = ODR_NOT_SUPPORTED;
  if (!icm4x6xx_odr_to_reg_val(odr, &odr_reg)) {
    return false;
  }

  if (!WriteMask(REG_ACCEL_CONFIG0, odr_reg, ACCEL_ODR_MASK)) {
    return false;
  }

  return true;
}

/**
 * @brief Set Gyro and Accel ODR, in this setting
 *        Gyro and Accel always share same odr
 *
 * @param[in] odr    the Accel/Gyro ODR will be set.
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_set_gyro_odr(float odr) {
  icm4x6xx_sensor_odr odr_reg = ODR_NOT_SUPPORTED;
  if (!icm4x6xx_odr_to_reg_val(odr, &odr_reg)) {
    return false;
  }

  if (!WriteMask(REG_GYRO_CONFIG0, odr_reg, GYRO_ODR_MASK)) {
    return false;
  }
  return true;
}

/**
 * @brief read interrupt status reg one.
 *
 * @param[out] status point to the value of
 *                    interrupt status reg one
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_read_int_status(uint8_t* status) {
  if (port_->ReadRegister(REG_INT_STATUS, status)) {
    return false;
  }

  return true;
}

/**
 * @brief read interrupt status reg two.
 *
 * @param[out] status point to the value of
 *                    interrupt status reg two
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool ICM42688::icm4x6xx_read_int_status2(uint8_t* status) {
  if (port_->ReadRegister(REG_INT_STATUS2, status)) {
    return false;
  }

  return true;
}

};  // namespace drvf