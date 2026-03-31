#include "ICM42688.hpp"

extern "C" {
#include "rtconfig.h"
#include "rtthread.h"

volatile uint32_t g_icm42688_probe_last_whoami = 0;
volatile uint32_t g_icm42688_probe_last_attempt = 0;
volatile uint32_t g_icm42688_probe_ok_count = 0;
volatile uint32_t g_icm42688_probe_fail_count = 0;
}

#undef LOG_TAG
#define LOG_TAG "icm42688"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif
#include <ulog.h>

namespace drvf {
#define TPM 0

namespace {

static uint64_t GetTimeUs() { return static_cast<uint64_t>(rt_tick_get_millisecond()) * 1000ULL; }

static void DelayMs(rt_uint32_t ms) { rt_thread_mdelay(ms); }

static void DelayUs(rt_uint32_t us) { rt_thread_mdelay((us + 999U) / 1000U); }

static constexpr int kProbeRetryCount = 5;
static volatile uint32_t g_icm42688_packet_parse_mismatch_count = 0;
// static constexpr rt_uint32_t SENSOR_ICM42688_SPI_MAX_HZ = 1000000;
static constexpr rt_uint16_t kSpiMode = (RT_SPI_MODE_0 | RT_SPI_MSB) & RT_SPI_MODE_MASK;

}  // namespace


ICM42688::ICM42688(int id, int cs, const ICM42688HwConfig &config) : id_(id), cs_(cs), config_(config) {
  spi_inited_ = false;
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

bool ICM42688::initSpi() {
  if (spi_inited_) {
    return true;
  }

  if (!spi_.init(config_.spi_bus_name, config_.spi_slave_name, config_.spi_cs_pin)) {
    return false;
  }

  if (!spi_.configure(kSpiMode, config_.spi_max_hz)) {
    return false;
  }

  spi_inited_ = true;
  return true;
}

bool ICM42688::readRegister(uint8_t reg, uint8_t *value) {
  if (!spi_inited_) {
    return false;
  }

  return spi_.readMultiReg8(reg, value, 1) == RT_EOK;
}

bool ICM42688::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
  if (!spi_inited_ || buf == nullptr || len == 0) {
    return false;
  }

  uint16_t offset = 0;
  while (offset < len) {
    const uint16_t chunk = (len - offset) > 255 ? 255 : (len - offset);
    if (spi_.readMultiReg8(reg, buf + offset, static_cast<uint8_t>(chunk)) != RT_EOK) {
      return false;
    }
    offset += chunk;
  }

  return true;
}

bool ICM42688::writeRegister(uint8_t reg, uint8_t value) {
  if (!spi_inited_) {
    return false;
  }

  return spi_.write_reg(reg, value) == RT_EOK;
}

bool ICM42688::verifyRegisterMaskedValue(uint8_t reg, uint8_t mask, uint8_t expected, const char *tag) {
  uint8_t value = 0;

  if (!readRegister(reg, &value)) {
    LOG_E("id[%d]: verify read failed for %s reg=0x%02X", id_, tag, reg);
    return false;
  }

  if ((value & mask) != (expected & mask)) {
    LOG_E("id[%d]: verify mismatch for %s reg=0x%02X mask=0x%02X expected=0x%02X actual=0x%02X",
          id_, tag, reg, mask, expected & mask, value & mask);
    return false;
  }

  return true;
}

bool ICM42688::verifyRegisterMaskedValueInBank(uint8_t bank, uint8_t reg, uint8_t mask, uint8_t expected, const char *tag) {
  bool ok = false;

  if (!icm4x6xx_set_reg_bank(bank)) {
    LOG_E("id[%d]: switch to bank %u failed for %s", id_, bank, tag);
    return false;
  }

  ok = verifyRegisterMaskedValue(reg, mask, expected, tag);

  if (!icm4x6xx_set_reg_bank(0)) {
    LOG_E("id[%d]: restore bank0 failed after %s", id_, tag);
    return false;
  }

  return ok;
}

bool ICM42688::probe() {
  uint8_t who_am_i = 0;
  if (!spi_.configure(kSpiMode, config_.spi_max_hz)) {
    return false;
  }

  DelayMs(1);

  for (int attempt = 1; attempt <= kProbeRetryCount; ++attempt) {
    uint8_t tx_buf[2] = {static_cast<uint8_t>(RegAddrNew::WHO_AM_I | 0x80u), 0xFFu};
    uint8_t rx_buf[2] = {0};

    g_icm42688_probe_last_attempt = static_cast<uint32_t>(attempt);
    who_am_i = 0;

    if (spi_.transfer(tx_buf, rx_buf, sizeof(tx_buf)) != RT_EOK) {
      g_icm42688_probe_last_whoami = 0xFFFFFFFFu;
      DelayMs(2);
      continue;
    }

    who_am_i = rx_buf[1];
    g_icm42688_probe_last_whoami = who_am_i;

    if (who_am_i == WHO_AM_I_ID) {
      ++g_icm42688_probe_ok_count;
      return true;
    }

    DelayMs(2);
  }

  ++g_icm42688_probe_fail_count;
  return false;
}

int ICM42688::DebugInit(bool clkin_enable) {
  if (!initSpi()) {
    has_inited_ = false;
    return -1;
  }

  auto DelayBeforeStep = []() { DelayMs(1); };
  // Normal bring-up path on this project uses clkin_enable == false.
  // Keep the default path focused on 1 kHz ODR, and hide the CLKIN branch below.
  const float target_odr = clkin_enable ? ICM4X6XX_ODR_8000 : ICM4X6XX_ODR_1000;
  icm4x6xx_sensor_odr target_odr_reg = ODR_NOT_SUPPORTED;

  has_inited_ = false;
  gyro_fsr_ = GYRO_RANGE_2000DPS;
  accel_fsr_ = ACC_RANGE_16G;
  fifo_mode_ = STREAM;
  accel_bandwith_ = BW_LL_MAX_200_8X_ODR;
  accel_power_mode_ = ICM4X6XX_A_LNM;
  use_hi_res_ = false;
  sync_time_count_ = 0;  // reset count to make a timestamp sync
  clkin_enable_ = clkin_enable;

  if (!icm4x6xx_odr_to_reg_val(target_odr, &target_odr_reg) || target_odr_reg == ODR_NOT_SUPPORTED) {
    LOG_E("id[%d]: odr %.1f cannot map to register value", id_, target_odr);
    has_inited_ = false;
    return -21;
  }

  // [1] Read WHO_AM_I.
  //     SPI raw probe frame:
  //       TX: [0xF5, 0xFF]  -> 0xF5 = REG 0x75 | READ bit 0x80
  //       RX: [xx,   0x47]  -> RX[1] must be WHO_AM_I_ID = 0x47
  //     Logic analyzer should see one 2-byte read transaction on register 0x75.
  DelayBeforeStep();
  if (!probe()) {
    has_inited_ = false;
    return -1;
  }

  // [2] Configure UI interface to SPI.
  //     Register: REG_INTF_CONFIG0 (0x4C)
  //     Operation: read-modify-write, mask UI_INTF_MASK (0x03)
  //     Field target: bits[1:0] = 0b11 (SPI_INTF = 0x03)
  //     Expected final write on 0x4C: low 2 bits become 0x03.
  DelayBeforeStep();
  if (!icm4x6xx_config_ui_intf(SPI_INTF)) {
      has_inited_ = false;
      return -2;
  }
  if (!verifyRegisterMaskedValue(REG_INTF_CONFIG0, UI_INTF_MASK, SPI_INTF, "ui_intf")) {
    has_inited_ = false;
    return -22;
  }

  // [3] CLKIN-only path.
  // Hidden in normal review because the common setup keeps clkin_enable == false.
  if (clkin_enable) {
    DelayBeforeStep();
  }
  if (!icm4x6xx_enable_rtc_mode(clkin_enable)) {
      has_inited_ = false;
      return -3;
  }
  if (clkin_enable) {
#if 0
    // CLKIN bring-up details kept here for future enablement:
    //   a) Write REG_BANK_SEL (0x76) = 0x01
    //   b) RMW REG_INTF_CONFIG5 (0x7B), set BIT_PIN9_FUNC_CLKIN
    //   c) Write REG_BANK_SEL (0x76) = 0x00
    //   d) RMW REG_INTF_CONFIG1 (0x4D), set BIT_RTC_MODE_EN
#endif
    if (!verifyRegisterMaskedValueInBank(1, REG_INTF_CONFIG5, BIT_PIN9_FUNC_MASK, BIT_PIN9_FUNC_CLKIN, "clkin_pinmux")) {
      return -23;
    }
    if (!verifyRegisterMaskedValue(REG_INTF_CONFIG1, BIT_RTC_MODE_EN, BIT_RTC_MODE_EN, "rtc_mode")) {
      return -24;
    }
  }

  // [4] CLKIN-only timestamp path, skipped for the normal false case.
  if (clkin_enable) {
    DelayBeforeStep();
    if (!icm4x6xx_enable_tmst(true)) {
      return -4;
    }
    if (!verifyRegisterMaskedValue(REG_TMST_CONFIG_REG,
                                   BIT_TMST_TO_REGS_EN | BIT_TMST_EN,
                                   BIT_TMST_TO_REGS_EN | BIT_TMST_EN,
                                   "tmst_enable")) {
      return -25;
    }
  }

  // [5] Enable big-endian mode for FIFO count and sensor payload.
  //     Register: REG_INTF_CONFIG0 (0x4C)
  //     Operation: read-modify-write
  //     Mask: FIFO_COUNT_BIG_ENDIAN_MASK | SENSOR_DATA_BIG_ENDIAN_MASK = 0x20 | 0x10 = 0x30
  //     Target value: 0x30
  DelayBeforeStep();
  if (!icm4x6xx_en_big_endian_mode(true)) {
    has_inited_ = false;
    return -5;
  }
  if (!verifyRegisterMaskedValue(REG_INTF_CONFIG0,
                                 FIFO_COUNT_BIG_ENDIAN_MASK | SENSOR_DATA_BIG_ENDIAN_MASK,
                                 FIFO_COUNT_BIG_ENDIAN_MASK | SENSOR_DATA_BIG_ENDIAN_MASK,
                                 "big_endian")) {
    return -26;
  }

  // [6] Set accelerometer full-scale range to +-16g.
  //     Register: REG_ACCEL_CONFIG0 (0x50)
  //     Operation: read-modify-write
  //     Mask: ACCEL_FSR_MASK = 0xE0
  //     accel_fsr_ = ACC_RANGE_16G = 0x00, so written field value is 0x00 << 5 = 0x00
  //     Expected final masked bits on 0x50[7:5] = 000b.
  DelayBeforeStep();
  if (!icm4x6xx_set_accel_fsr(accel_fsr_)) {
    has_inited_ = false;
    return -7;
  }
  if (!verifyRegisterMaskedValue(REG_ACCEL_CONFIG0,
                                 ACCEL_FSR_MASK,
                                 accel_fsr_ << ACCEL_FSR_SHIFT,
                                 "accel_fsr")) {
    return -28;
  }

  // [7] Set gyroscope full-scale range to +-2000 dps.
  //     Register: REG_GYRO_CONFIG0 (0x4F)
  //     Operation: read-modify-write
  //     Mask: GYRO_FSR_MASK = 0xE0
  //     gyro_fsr_ = GYRO_RANGE_2000DPS = 0x00, so written field value is 0x00 << 5 = 0x00
  //     Expected final masked bits on 0x4F[7:5] = 000b.
  DelayBeforeStep();
  if (!icm4x6xx_set_gyro_fsr(gyro_fsr_)) {
    has_inited_ = false;
    return -8;
  }
  if (!verifyRegisterMaskedValue(REG_GYRO_CONFIG0,
                                 GYRO_FSR_MASK,
                                 gyro_fsr_ << GYRO_FSR_SHIFT,
                                 "gyro_fsr")) {
    return -29;
  }

  // [8] Set FIFO mode to STREAM.
  //      Register: REG_FIFO_CONFIG (0x16)
  //      Operation: read-modify-write
  //      Mask: BIT_FIFO_MODE_CTRL_MASK = 0xC0
  //      STREAM enum value = 1, shifted by BIT_FIFO_MODE_SHIFT (=6)
  //      Target field value: 0x40
  DelayBeforeStep();
  if (!icm4x6xx_set_fifo_mode(fifo_mode_)) {
    has_inited_ = false;
    return -11;
  }
  if (!verifyRegisterMaskedValue(REG_FIFO_CONFIG,
                                 BIT_FIFO_MODE_CTRL_MASK,
                                 fifo_mode_ << BIT_FIFO_MODE_SHIFT,
                                 "fifo_mode")) {
    return -32;
  }

  // [9] Set accelerometer ODR.
  //      Register: REG_ACCEL_CONFIG0 (0x50)
  //      Operation: read-modify-write
  //      Mask: ACCEL_ODR_MASK = 0x0F
  //      DebugInit(false): ODR_1KHZ = 0x06
  //      DebugInit(true):  ODR_8KHZ = 0x03
  //      Expected masked bits on 0x50[3:0] = 0x06 or 0x03.
  DelayBeforeStep();
  if (!icm4x6xx_set_accel_odr(target_odr)) {
    has_inited_ = false;
    return -14;
  }
  if (!verifyRegisterMaskedValue(REG_ACCEL_CONFIG0,
                                 ACCEL_ODR_MASK,
                                 target_odr_reg,
                                 "accel_odr")) {
    return -38;
  }

  // [10] Set gyroscope ODR.
  //      Register: REG_GYRO_CONFIG0 (0x4F)
  //      Operation: read-modify-write
  //      Mask: GYRO_ODR_MASK = 0x0F
  //      DebugInit(false): ODR_1KHZ = 0x06
  //      DebugInit(true):  ODR_8KHZ = 0x03
  //      Expected masked bits on 0x4F[3:0] = 0x06 or 0x03.
  DelayBeforeStep();
  if (!icm4x6xx_set_gyro_odr(target_odr)) {
    has_inited_ = false;
    return -15;
  }
  if (!verifyRegisterMaskedValue(REG_GYRO_CONFIG0,
                                 GYRO_ODR_MASK,
                                 target_odr_reg,
                                 "gyro_odr")) {
    return -39;
  }

  // [11] Power up accel and gyro to low-noise mode.
  //      Register: REG_PWR_MGMT_0 (0x4E)
  //      Operation: read-modify-write
  //      Mask: ACCEL_LNM_MASK | GYRO_LNM_MASK = 0x03 | 0x0C = 0x0F
  //      ACCEL_GYRO_POWERUP = 0x0F
  //      Expected masked bits on 0x4E[3:0] = 0x0F
  //      Helper additionally waits 20ms after successful write.
  DelayBeforeStep();
  if (!icm4x6xx_accel_gyro_powerup(ACCEL_GYRO_POWERUP)) {
    has_inited_ = false;
    return -19;
  }
  if (!verifyRegisterMaskedValue(REG_PWR_MGMT_0, ACCEL_LNM_MASK | GYRO_LNM_MASK, ACCEL_GYRO_POWERUP, "powerup")) {
    has_inited_ = false;
    return -43;
  }

  // [12] Flush FIFO once after power-up so the first count starts from a clean state.
  DelayBeforeStep();
  if (!WriteMask(REG_SIGNAL_PATH_RESET_REG, BIT_FIFO_FLUSH, BIT_FIFO_FLUSH)) {
    has_inited_ = false;
    return -20;
  }
  DelayMs(2);

  // [13] Enable FIFO accel + gyro + temperature after power-up and flush.
  DelayBeforeStep();
  if (!icm4x6xx_en_fifo(en_a_fifo_, en_g_fifo_)) {
    has_inited_ = false;
    return -18;
  }
  const uint8_t expected_fifo_config1 =
      (en_a_fifo_ ? FIFO_ACCEL_EN_MASK : 0) |
      (en_g_fifo_ ? FIFO_GYRO_EN_MASK : 0) |
      ((en_a_fifo_ || en_g_fifo_) ? FIFO_TEMP_EN_MASK : 0) |
      (use_hi_res_ ? ((en_a_fifo_ || en_g_fifo_) ? FIFO_HIRES_EN_MASK : 0) : 0);
  if (!verifyRegisterMaskedValue(REG_FIFO_CONFIG_1,
                                 FIFO_ACCEL_EN_MASK | FIFO_GYRO_EN_MASK | FIFO_TEMP_EN_MASK |
                                     FIFO_TMST_FSYNC_EN_MASK | FIFO_HIRES_EN_MASK,
                                 expected_fifo_config1,
                                 "fifo_enable")) {
    return -42;
  }

  has_inited_ = true;
  return 0;
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
    DelayUs(20000);
  }
  return ret;
}

bool ICM42688::icm4x6xx_disable_aux_pins() {
  uint8_t reg_value = 0;

  if (!icm4x6xx_set_reg_bank(2)) {
    return false;
  }

  reg_value = 0x01;

  if (!writeRegister(static_cast<uint8_t>(0x70), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (!writeRegister(static_cast<uint8_t>(0x71), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (!writeRegister(static_cast<uint8_t>(0x72), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (!writeRegister(static_cast<uint8_t>(0x73), reg_value)) {
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

  if (!writeRegister(static_cast<uint8_t>(REG_APEX_CONFIG6),
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
  if (!writeRegister(static_cast<uint8_t>(REG_BANK_SEL), bank_num)) {
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

  if (!readRegister(static_cast<uint8_t>(reg_addr), &rw_buffer)) {
    return false;
  }

  /* generate new value */
  rw_buffer = (rw_buffer & (~mask)) | (reg_value & mask);

  /* write new value to this register */
  if (!writeRegister(static_cast<uint8_t>(reg_addr), rw_buffer)) {
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

  if (!readRegisters(REG_FIFO_BYTE_COUNT_L, buff, 2)) {
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
  uint64_t dsp_time = GetTimeUs();
  uint16_t fifo_count = 0;
  uint16_t bytes_to_read = 0;
  uint8_t packet_size = 0;
  uint16_t packet_cnt = 0;
  uint32_t valid_buf_len = 0;

  data.timestamp_us = dsp_time;
  data.packet_size = 20;
  data.fifo_count = 0;
  rt_memset(data.fifo_data, 0, sizeof(data.fifo_data));

  if (!has_inited_ && DebugInit(false) != 0) {
    return false;
  }

  if (!icm4x6xx_get_packet_size(&packet_size)) {
    LOG_E("id[%d]: icm4x6xx_get_packet_size fail\n", id_);
    return false;
  }

  if (!icm4x6xx_read_fifo_count(&fifo_count)) {
    LOG_E("id[%d]: icm4x6xx_read_fifo_count fail\n", id_);
    return false;
  }

  if (fifo_count == 0) {
    return false;
  }

  bytes_to_read = fifo_count;
  if (bytes_to_read > (packet_size * MAX_SCP_SAMPLES)) {
    bytes_to_read = (packet_size * MAX_SCP_SAMPLES);
  }

  uint8_t buf[bytes_to_read];
  if (!icm4x6xx_read_fifo_buf(buf, bytes_to_read)) {
    LOG_E("id[%d]: icm4x6xx_read_fifo_buf(buf, bytes_to_read) fail, fifo_count = %d", id_, fifo_count);
    return false;
  }

  valid_buf_len = icm4x6xx_cal_valid_fifo_len(buf, bytes_to_read, &packet_cnt);
  if (valid_buf_len == 0 || packet_cnt == 0) {
    LOG_E("id[%d]: (valid_buf_len == 0 || packet_cnt == 0) fail, fifo_count = %d", id_, fifo_count);
    return false;
  }

  if (packet_cnt != valid_buf_len / packet_size) {
    ++g_icm42688_packet_parse_mismatch_count;
    LOG_E("id[%d]: parse_mismatch #%lu fifo_bytes=%u packet_bytes=%u parsed_packets=%lu remain_bytes=%lu packet_cnt=%u",
          id_,
          (unsigned long)g_icm42688_packet_parse_mismatch_count,
          fifo_count,
          packet_size,
          (unsigned long)(valid_buf_len / packet_size),
          (unsigned long)(valid_buf_len % packet_size),
          packet_cnt);
    return false;
  }

  data.timestamp_us = dsp_time;
  data.packet_size = packet_size;
  data.fifo_count = (uint16_t)valid_buf_len;
  memcpy(data.fifo_data, buf, data.fifo_count);

  return true;
}

bool ICM42688::WriteByte(uint8_t reg, uint8_t val) {
  if (!writeRegister(static_cast<uint8_t>(reg), val)) {
    return false;
  }
  DelayMs(5);
  return true;
}

bool ICM42688::ReadBlock(uint8_t first_reg, uint8_t buf[], int len) {
  if (!readRegisters(static_cast<uint8_t>(first_reg), buf, len)) {
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
  if (!readRegister(REG_INT_STATUS, status)) {
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
  if (!readRegister(REG_INT_STATUS2, status)) {
    return false;
  }

  return true;
}

};  // namespace drvf
