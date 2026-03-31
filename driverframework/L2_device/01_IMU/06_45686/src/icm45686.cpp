#include "ICM45686.hpp"

extern "C" {
#include "rtconfig.h"
#include "rtthread.h"
#if defined(SOC_FAMILY_STM32)
void rt_hw_us_delay(rt_uint32_t us);
#endif
}

#undef LOG_TAG
#define LOG_TAG "icm45686"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif
#include <ulog.h>

namespace drvf {
namespace {

static ICM45686 *g_ref_owner = nullptr;

static constexpr rt_uint16_t kSpiMode = (RT_SPI_MODE_0 | RT_SPI_MSB) & RT_SPI_MODE_MASK;
static constexpr uint8_t kRegPwrMgmt0 = 0x10;
static constexpr uint8_t kRegAccelDataX1Ui = 0x00;
static constexpr uint8_t kRegFifoCount0 = 0x12;
static constexpr uint8_t kRegFifoData = 0x14;
static constexpr uint8_t kRegAccelConfig0 = 0x1B;
static constexpr uint8_t kRegGyroConfig0 = 0x1C;
static constexpr uint8_t kRegFifoConfig0 = 0x1D;
static constexpr uint8_t kRegFifoConfig10 = 0x1E;
static constexpr uint8_t kRegFifoConfig11 = 0x1F;
static constexpr uint8_t kRegFifoConfig2 = 0x20;
static constexpr uint8_t kRegFifoConfig3 = 0x21;
static constexpr uint8_t kRegFifoConfig4 = 0x22;
static constexpr uint8_t kRegTmstWomConfig = 0x23;
static constexpr uint8_t kRegIntfConfig1Ovrd = 0x2D;
static constexpr uint8_t kRegIregAddr158 = 0x7C;
static constexpr uint8_t kRegIregData = 0x7E;
static constexpr uint8_t kRegMisc2 = 0x7F;
static constexpr uint16_t kRegSmcControl0 = 0xA258;
static constexpr uint16_t kRegSregCtrl = 0xA267;
static constexpr uint8_t kSoftReset = 0x01;
static constexpr uint8_t kIntfConfig1Ovrd4Wire = 0x0CU;
static constexpr uint8_t kAccelConfig16G1600Hz = 0x15U;
static constexpr uint8_t kGyroConfig2000dps1600Hz = 0x15U;
static constexpr uint8_t kFifoConfig0Stream = 0x47U;
static constexpr uint8_t kFifoConfig0Bypass = 0x07U;
static constexpr uint8_t kFifoWatermarkLow = 0x01U;
static constexpr uint8_t kFifoWatermarkHigh = 0x00;
static constexpr uint8_t kFifoConfig2WmEqOrGt = 0x08U;
static constexpr uint8_t kFifoConfig2Flush = 0x88U;
static constexpr uint8_t kFifoConfig3AccelGyroEnable = 0x06U;
static constexpr uint8_t kFifoConfig3AccelGyroFifoIf = 0x07U;
static constexpr uint8_t kFifoConfig4TimestampEnable = 0x02U;
static constexpr uint8_t kTmstWomConfigResolutionMask = 0x20U;
static constexpr uint8_t kTmstWomConfigDeltaMask = 0x40U;
static constexpr uint8_t kTmstWomConfigTimestampMask = kTmstWomConfigResolutionMask | kTmstWomConfigDeltaMask;
static constexpr uint8_t kTmstWomConfig1usResolution = 0x00U;
static constexpr uint8_t kSmcControl0TimestampMask = 0x03U;
static constexpr uint8_t kSmcControl0TimestampEnable = 0x03U;
static constexpr uint8_t kSregCtrlEndianMask = 0x02U;
static constexpr uint8_t kSregCtrlLittleEndian = 0x00U;
static constexpr uint8_t kPwrMgmt0AccelGyroLn = 0x0F;
static constexpr uint16_t kPacketSize16 = 16;
static constexpr int kProbeRetryCount = 5;
static constexpr int kPowerUpDelayMs = 80;

static void DelayMs(rt_uint32_t ms) { rt_thread_mdelay(ms); }
static void DelayUs(rt_uint32_t us) {
#if defined(SOC_FAMILY_STM32)
  rt_hw_us_delay(us);
#else
  rt_thread_mdelay((us + 999U) / 1000U);
#endif
}
static uint64_t GetTimeUs() { return static_cast<uint64_t>(rt_tick_get_millisecond()) * 1000ULL; }

static int RefReadReg(uint8_t reg, uint8_t *buf, uint32_t len) {
  if (g_ref_owner == nullptr || buf == nullptr || len == 0U || len > 255U) {
    return -1;
  }
  return g_ref_owner->transportReadReg(reg, buf, len);
}

static int RefWriteReg(uint8_t reg, const uint8_t *buf, uint32_t len) {
  if (g_ref_owner == nullptr || buf == nullptr || len == 0U) {
    return -1;
  }
  return g_ref_owner->transportWriteReg(reg, buf, len);
}

static void RefSleepUs(uint32_t us) { DelayUs(us); }

static inline void StoreLe16(uint8_t *dst, int16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
}

static inline void StoreLe16U(uint8_t *dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

}  // namespace

ICM45686::ICM45686(int id, int cs, const ICM45686HwConfig &config)
    : id_(id), cs_(cs), spi_inited_(false), configured_(false), last_whoami_(0), active_spi_mode_(kSpiMode), config_(config) {
  rt_memset(&ref_device_, 0, sizeof(ref_device_));
}

ICM45686::~ICM45686() {}

bool ICM45686::initSpi() {
  if (spi_inited_) {
    return true;
  }

  if (!spi_.init(config_.spi_bus_name, config_.spi_slave_name, config_.spi_cs_pin)) {
    LOG_E("id[%d]: spi init failed bus=%s slave=%s cs=%s", id_, config_.spi_bus_name, config_.spi_slave_name, config_.spi_cs_pin);
    return false;
  }

  if (!spi_.configure(kSpiMode, config_.spi_max_hz)) {
    LOG_E("id[%d]: spi configure failed hz=%u", id_, static_cast<unsigned>(config_.spi_max_hz));
    return false;
  }

  spi_inited_ = true;
  active_spi_mode_ = kSpiMode;
  g_ref_owner = this;
  ref_device_.transport.read_reg = RefReadReg;
  ref_device_.transport.write_reg = RefWriteReg;
  ref_device_.transport.sleep_us = RefSleepUs;
  ref_device_.transport.serif_type = UI_SPI4;
  return true;
}

bool ICM45686::readRegister(uint8_t reg, uint8_t *value) {
  if (value == nullptr || !initSpi()) {
    return false;
  }
  return readRegisters(reg, value, 1);
}

int ICM45686::transportReadReg(uint8_t reg, uint8_t *buf, uint32_t len) {
  return readRegisters(reg, buf, static_cast<uint16_t>(len)) ? 0 : -1;
}

int ICM45686::transportWriteReg(uint8_t reg, const uint8_t *buf, uint32_t len) {
  uint32_t offset = 0;

  if (buf == nullptr || len == 0U || !initSpi()) {
    return -1;
  }

  while (offset < len) {
    const uint16_t chunk = (len - offset) > 255U ? 255U : static_cast<uint16_t>(len - offset);
    if (spi_.writeMultiReg8(static_cast<uint8_t>(reg + offset), const_cast<uint8_t *>(buf + offset), chunk) != RT_EOK) {
      return -1;
    }
    offset += chunk;
  }

  return 0;
}

bool ICM45686::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
  if (buf == nullptr || len == 0 || !initSpi()) {
    return false;
  }

  uint16_t offset = 0;
  while (offset < len) {
    const uint16_t chunk = (len - offset) > 255 ? 255 : (len - offset);
    if (spi_.readMultiReg8(static_cast<uint8_t>(reg + offset), buf + offset, static_cast<uint8_t>(chunk)) != RT_EOK) {
      return false;
    }
    offset += chunk;
  }

  return true;
}

bool ICM45686::writeRegister(uint8_t reg, uint8_t value) {
  if (!initSpi()) {
    return false;
  }

  return spi_.write_reg(reg, value) == RT_EOK;
}

bool ICM45686::writeRegisterChecked(uint8_t reg, uint8_t value, uint8_t expected_mask) {
  uint8_t readback = 0;

  if (!writeRegister(reg, value)) {
    return false;
  }

  DelayUs(50U);

  if (!readRegister(reg, &readback)) {
    return false;
  }

  if ((readback & expected_mask) != (value & expected_mask)) {
    LOG_W("id[%d]: reg 0x%02X write 0x%02X verify 0x%02X mask 0x%02X", id_, reg, value, readback, expected_mask);
    return false;
  }

  return true;
}

bool ICM45686::updateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current = 0;

  if (mask == 0U) {
    return true;
  }

  if (!readRegister(reg, &current)) {
    return false;
  }

  current = static_cast<uint8_t>((current & static_cast<uint8_t>(~mask)) | (value & mask));
  return writeRegister(reg, current);
}

bool ICM45686::readMreg(uint16_t reg, uint8_t *buf, uint16_t len) {
  uint16_t index = 0;
  uint8_t addr_buf[2] = {0};

  if (buf == nullptr || len == 0U || !initSpi()) {
    return false;
  }

  addr_buf[0] = static_cast<uint8_t>((reg >> 8) & 0xFFU);
  addr_buf[1] = static_cast<uint8_t>(reg & 0xFFU);
  if (spi_.writeMultiReg8(kRegIregAddr158, addr_buf, sizeof(addr_buf)) != RT_EOK) {
    return false;
  }
  DelayUs(4U);

  for (index = 0; index < len; ++index) {
    DelayUs(4U);
    if (!readRegister(kRegIregData, &buf[index])) {
      return false;
    }
  }

  return true;
}

bool ICM45686::writeMreg(uint16_t reg, const uint8_t *buf, uint16_t len) {
  uint16_t index = 0;
  uint8_t write_buf[3] = {0};

  if (buf == nullptr || len == 0U || !initSpi()) {
    return false;
  }

  write_buf[0] = static_cast<uint8_t>((reg >> 8) & 0xFFU);
  write_buf[1] = static_cast<uint8_t>(reg & 0xFFU);
  write_buf[2] = buf[0];
  if (spi_.writeMultiReg8(kRegIregAddr158, write_buf, sizeof(write_buf)) != RT_EOK) {
    return false;
  }

  for (index = 1; index < len; ++index) {
    DelayUs(4U);
    if (!writeRegister(kRegIregData, buf[index])) {
      return false;
    }
  }

  return true;
}

bool ICM45686::writeMregByte(uint16_t reg, uint8_t value) { return writeMreg(reg, &value, 1U); }

bool ICM45686::writeMregByteChecked(uint16_t reg, uint8_t value, uint8_t expected_mask) {
  uint8_t readback = 0;

  if (!writeMregByte(reg, value)) {
    return false;
  }

  DelayUs(50U);

  if (!readMreg(reg, &readback, 1U)) {
    return false;
  }

  if ((readback & expected_mask) != (value & expected_mask)) {
    LOG_W("id[%d]: mreg 0x%04X write 0x%02X verify 0x%02X mask 0x%02X", id_, reg, value, readback, expected_mask);
    return false;
  }

  return true;
}

bool ICM45686::updateMregBits(uint16_t reg, uint8_t mask, uint8_t value) {
  uint8_t current = 0;

  if (mask == 0U) {
    return true;
  }

  if (!readMreg(reg, &current, 1U)) {
    return false;
  }

  current = static_cast<uint8_t>((current & static_cast<uint8_t>(~mask)) | (value & mask));
  return writeMregByteChecked(reg, current, mask);
}

bool ICM45686::readFifoByteCount(uint16_t *byte_count) {
  uint8_t raw_count[2] = {0};

  if (byte_count == nullptr) {
    return false;
  }

  if (!readRegisters(kRegFifoCount0, raw_count, sizeof(raw_count))) {
    return false;
  }

  /* AN-000364: use the second FIFO count sample. */
  if (!readRegisters(kRegFifoCount0, raw_count, sizeof(raw_count))) {
    return false;
  }

  *byte_count = static_cast<uint16_t>(raw_count[0] | (static_cast<uint16_t>(raw_count[1]) << 8));
  return true;
}

bool ICM45686::readFifoData(uint8_t *buf, uint16_t len) {
  if (buf == nullptr || len == 0U || !initSpi()) {
    return false;
  }

  uint16_t offset = 0;
  while (offset < len) {
    const uint16_t chunk = (len - offset) > 255 ? 255 : (len - offset);
    if (spi_.readMultiReg8(kRegFifoData, buf + offset, static_cast<uint8_t>(chunk)) != RT_EOK) {
      return false;
    }
    offset += chunk;
  }

  return true;
}

bool ICM45686::flushFifo() {
  if (!writeRegister(kRegFifoConfig2, kFifoConfig2Flush)) {
    return false;
  }
  DelayMs(2);
  return writeRegister(kRegFifoConfig2, kFifoConfig2WmEqOrGt);
}

void ICM45686::logKeyRegisters(const char *stage) {
  uint8_t pwr = 0;
  uint8_t fifo_count[2] = {0};
  uint8_t acc0 = 0;
  uint8_t gyr0 = 0;
  uint8_t fifo0 = 0;
  uint8_t fifo1_l = 0;
  uint8_t fifo1_h = 0;
  uint8_t fifo234[3] = {0};
  uint8_t tmst_wom = 0;
  uint8_t smc0 = 0;
  uint8_t sreg = 0;

  if (!readRegister(kRegPwrMgmt0, &pwr) || !readRegisters(kRegFifoCount0, fifo_count, sizeof(fifo_count)) ||
      !readRegister(kRegAccelConfig0, &acc0) || !readRegister(kRegGyroConfig0, &gyr0) || !readRegister(kRegFifoConfig0, &fifo0) ||
      !readRegister(kRegFifoConfig10, &fifo1_l) || !readRegister(kRegFifoConfig11, &fifo1_h) ||
      !readRegisters(kRegFifoConfig2, fifo234, sizeof(fifo234)) || !readRegister(kRegTmstWomConfig, &tmst_wom)) {
    LOG_W("id[%d]: %s readback failed", id_, stage ? stage : "regdump");
    return;
  }

  LOG_I(
      "id[%d]: %s PWR=0x%02X FIFO_CNT=[0x%02X 0x%02X] ACC0=0x%02X GYR0=0x%02X FIFO0=0x%02X FIFO1L=0x%02X FIFO1H=0x%02X",
      id_,
      stage ? stage : "regdump",
      pwr,
      fifo_count[0],
      fifo_count[1],
      acc0,
      gyr0,
      fifo0,
      fifo1_l,
      fifo1_h);

  LOG_I("id[%d]: %s FIFO2=0x%02X FIFO3=0x%02X FIFO4=0x%02X TMST_WOM=0x%02X",
        id_,
        stage ? stage : "regdump",
        fifo234[0],
        fifo234[1],
        fifo234[2],
        tmst_wom);
  if (readMreg(kRegSmcControl0, &smc0, 1U) && readMreg(kRegSregCtrl, &sreg, 1U)) {
    LOG_I("id[%d]: %s SMC0=0x%02X SREG=0x%02X", id_, stage ? stage : "regdump", smc0, sreg);
  }
}

bool ICM45686::configureForPolling() {
  uint8_t who_am_i = 0;

  if (configured_) {
    return true;
  }

  /*
   * [00] SPI Host Mode
   * 目的: 在真正写配置前，先切回探测成功时使用的 SPI mode，避免 reset 后 host 侧仍停留在错误 CPOL/CPHA。
   * 字段: RT-Thread SPI host config
   * 最终值: probe 成功时记录的 mode
   */
  if (!spi_.configure(active_spi_mode_, config_.spi_max_hz)) {
    return false;
  }

  LOG_I("id[%d]: cfg_path=v2_no_intf_ovrd spi_mode=0x%X", id_, active_spi_mode_);

  /*
   * [01] INTF_CONFIG1_OVRD (0x2D)
   * 目的: 原本计划强制 4-wire SPI。
   * 当前板上实测: 写 0x0C 后回读始终为 0x00，会导致初始化过早失败。
   * 处理: 先沿用 probe 成功时的 SPI mode，不在主路径强制改该寄存器。
   */

  /*
   * [02] REG_MISC2 (0x7F)
   * 当前板上临时跳过 soft reset。
   * 原因: 实测 soft reset 之后，后续 DREG 写入无法稳定生效。
   * 处理: 直接从 probe 成功后的默认状态继续配置，先把 FIFO 主链路救通。
   */
  configured_ = false;
  who_am_i = last_whoami_;

  /*
   * [03] INTF_CONFIG1_OVRD (0x2D)
   * 同上，暂不在 reset 后重写该寄存器，避免把初始化卡死在 0x2D。
   */

  /*
   * [04] SREG_CTRL (MREG 0xA267)
   * 目的: 指定 Sensor Register/FIFO 数据采用 little-endian，和当前上位机及解析代码保持一致。
   * 字段: SREG_DATA_ENDIAN_SEL
   * 最终值: 保留其它位，仅将 bit[1] 配置为 0，对应 little-endian
   */
  /* 暂时跳过 MREG 端序配置，先保证基础 FIFO 链路稳定。 */

  /*
   * [05] SMC_CONTROL_0 (MREG 0xA258)
   * 目的: 使能 timestamp 功能，让 16-byte FIFO 包尾部的 2 字节时间戳有效。
   * 字段: TMST_EN
   * 最终值: 保留其它位，仅将 bit[1:0] 配置为 0x03
   */
  /*
   * [05] SMC_CONTROL_0 (MREG 0xA258)
   * 目的: 只打开 TMST_EN，让 FIFO 16-byte 包尾部的 timestamp 有效。
   * 字段: TMST_EN(bit0)
   * 最终值: 保留其它位，仅将 bit0 置 1
   */
  if (!updateMregBits(kRegSmcControl0, 0x01U, 0x01U)) {
    return false;
  }

  /*
   * [06] TMST_WOM_CONFIG (0x23)
   * 目的: 尝试将 FIFO timestamp 固定为“绝对时间戳 + 1 us 分辨率”。
   * 字段: TMST_RESOL(bit5)=0, TMST_DELTA_EN(bit6)=0
   * 当前板上实测: bit5 会稳定读回 1，芯片仍工作在约 16 us/tick。
   * 处理: 这里改成 best-effort，不再因为 1 us 分辨率写不进去而让整条 FIFO 链路初始化失败。
   */
  {
    uint8_t tmst_wom_config = 0;
    uint8_t tmst_wom_readback = 0;

    if (!readRegister(kRegTmstWomConfig, &tmst_wom_config)) {
      return false;
    }

    tmst_wom_config = static_cast<uint8_t>(tmst_wom_config & static_cast<uint8_t>(~kTmstWomConfigTimestampMask));
    tmst_wom_config = static_cast<uint8_t>(tmst_wom_config | kTmstWomConfig1usResolution);
    if (!writeRegister(kRegTmstWomConfig, tmst_wom_config)) {
      return false;
    }
    DelayUs(50U);
    if (!readRegister(kRegTmstWomConfig, &tmst_wom_readback)) {
      return false;
    }
    if ((tmst_wom_readback & kTmstWomConfigTimestampMask) != (tmst_wom_config & kTmstWomConfigTimestampMask)) {
      LOG_W("id[%d]: TMST_WOM_CONFIG keeps 0x%02X, timestamp raw tick stays sensor-default", id_, tmst_wom_readback);
    }
  }

  /*
   * [07] FIFO_CONFIG3 (0x21)
   * 目的: 先关闭 FIFO interface，确保 FIFO 参数修改过程不会在运行中生效。
   * 字段: FIFO_IF_EN/FIFO_ACCEL_EN/FIFO_GYRO_EN
   * 最终值: 0x00
   */
  if (!writeRegisterChecked(kRegFifoConfig3, 0x00, 0x07U)) {
    return false;
  }

  /*
   * [08] FIFO_CONFIG0 (0x1D)
   * 目的: 将 FIFO 置为 bypass，进入安全重配状态。
   * 字段: FIFO_MODE/FIFO_DEPTH
   * 最终值: 0x07
   */
  if (!writeRegisterChecked(kRegFifoConfig0, kFifoConfig0Bypass)) {
    return false;
  }

  /*
   * [09] ACCEL_CONFIG0 (0x1B)
   * 目的: 配置加速度量程和 ODR。
   * 字段: ACCEL_UI_FS_SEL=16G，ACCEL_ODR=1600Hz
   * 最终值: 0x15
   */
  if (!writeRegisterChecked(kRegAccelConfig0, kAccelConfig16G1600Hz)) {
    return false;
  }

  /*
   * [10] GYRO_CONFIG0 (0x1C)
   * 目的: 配置陀螺仪量程和 ODR。
   * 字段: GYRO_UI_FS_SEL=2000dps，GYRO_ODR=1600Hz
   * 最终值: 0x15
   */
  if (!writeRegisterChecked(kRegGyroConfig0, kGyroConfig2000dps1600Hz)) {
    return false;
  }

  /*
   * [11] FIFO_CONFIG4 (0x22)
   * 目的: 打开 FIFO 里的 timestamp/fsync 扩展尾字段，使双 sensor 包扩展成 16-byte。
   * 字段: FIFO_TMST_FSYNC_EN
   * 最终值: 0x02
   */
  if (!writeRegisterChecked(kRegFifoConfig4, kFifoConfig4TimestampEnable, 0x02U)) {
    return false;
  }

  /*
   * [12] FIFO_CONFIG1_0 (0x1E)
   * 目的: 设置 FIFO watermark 低字节。
   * 字段: FIFO_WM_TH[7:0]
   * 最终值: 0x01
   */
  if (!writeRegisterChecked(kRegFifoConfig10, kFifoWatermarkLow) || !writeRegisterChecked(kRegFifoConfig11, kFifoWatermarkHigh)) {
    return false;
  }

  /*
   * [13] FIFO_CONFIG1_1 (0x1F)
   * 目的: 设置 FIFO watermark 高字节；当前 watermark=1 frame。
   * 字段: FIFO_WM_TH[15:8]
   * 最终值: 0x00
   */

  /*
   * [14] FIFO_CONFIG2 (0x20)
   * 目的: 设定 watermark 触发条件为 >= threshold，并保持 flush 位清零。
   * 字段: FIFO_WR_WM_GT_TH / FIFO_FLUSH
   * 最终值: 0x08
   */
  if (!writeRegisterChecked(kRegFifoConfig2, kFifoConfig2WmEqOrGt, 0x88U)) {
    return false;
  }

  /*
   * [15] FIFO_CONFIG3 (0x21)
   * 目的: 先声明 FIFO 中要装 accel+gyro 两类数据，但先不打开 FIFO_IF_EN。
   * 字段: FIFO_ACCEL_EN=1，FIFO_GYRO_EN=1，FIFO_IF_EN=0
   * 最终值: 0x06
   */
  if (!writeRegisterChecked(kRegFifoConfig3, kFifoConfig3AccelGyroEnable, 0x07U)) {
    return false;
  }

  /*
   * [16] PWR_MGMT0 (0x10)
   * 目的: 打开 accel 和 gyro 的 low-noise 模式，让 UI/FIFO 数据链路真正开始产数。
   * 字段: ACCEL_MODE=LN，GYRO_MODE=LN
   * 最终值: 0x0F
   */
  if (!writeRegisterChecked(kRegPwrMgmt0, kPwrMgmt0AccelGyroLn, 0x0FU)) {
    return false;
  }
  DelayMs(kPowerUpDelayMs);

  /*
   * [17] FIFO_CONFIG0 (0x1D)
   * 目的: 将 FIFO 切到 stream 模式，开始持续写入新样本。
   * 字段: FIFO_MODE=STREAM，FIFO_DEPTH 维持当前配置
   * 最终值: 0x47
   */
  if (!writeRegisterChecked(kRegFifoConfig0, kFifoConfig0Stream)) {
    return false;
  }

  /*
   * [18] FIFO_CONFIG3 (0x21)
   * 目的: 在 accel/gyro 选择保持不变的前提下，最终打开 FIFO_IF_EN。
   * 字段: FIFO_ACCEL_EN=1，FIFO_GYRO_EN=1，FIFO_IF_EN=1
   * 最终值: 0x07
   */
  if (!writeRegisterChecked(kRegFifoConfig3, kFifoConfig3AccelGyroFifoIf, 0x07U)) {
    return false;
  }

  /*
   * [19] FIFO_CONFIG2 (0x20)
   * 目的: 触发一次 FIFO flush，清掉切换配置过程中的旧数据，再恢复 watermark 模式。
   * 字段: FIFO_FLUSH
   * 最终值: 先写 0x88，再写回 0x08
   */
  if (!flushFifo()) {
    return false;
  }

  /*
   * [20] WHO_AM_I (0x72)
   * 目的: 配置完成后做回读校验，确认器件仍在线且 SPI 通信正常。
   * 字段: 整寄存器只读
   * 期望值: 0xE9
   */
  if (!readRegister(config_.whoami_reg, &who_am_i) || who_am_i != config_.whoami_expected) {
    last_whoami_ = who_am_i;
    LOG_W("id[%d]: cfg verify whoami=0x%02X", id_, who_am_i);
    configured_ = false;
    return false;
  }

  logKeyRegisters("cfg");
  configured_ = true;
  last_whoami_ = who_am_i;
  return true;
}

bool ICM45686::configureWithReferenceDriver() { return configureForPolling(); }

bool ICM45686::probe() {
  uint8_t who_am_i = 0;
  if (!spi_.configure(kSpiMode, config_.spi_max_hz)) {
    return false;
  }

  DelayMs(1);
  for (int attempt = 0; attempt < kProbeRetryCount; ++attempt) {
    if (!readRegister(config_.whoami_reg, &who_am_i)) {
      DelayMs(2);
      continue;
    }

    last_whoami_ = who_am_i;
    if (who_am_i == config_.whoami_expected) {
      active_spi_mode_ = kSpiMode;
      return true;
    }

    DelayMs(2);
  }

  last_whoami_ = who_am_i;
  return false;
}

int ICM45686::DebugInit() {
  configured_ = false;

  if (!initSpi()) {
    return -1;
  }

  DelayMs(3);

  if (!probe()) {
    LOG_W("id[%d]: whoami mismatch read=0x%02X expected=0x%02X", id_, last_whoami_, config_.whoami_expected);
    return -2;
  }

  if (!configureForPolling()) {
    LOG_E("id[%d]: configure for polling failed", id_);
    return -3;
  }

  logKeyRegisters("debug_init");
  LOG_I("id[%d]: probe ok whoami=0x%02X", id_, last_whoami_);
  return 0;
}

void ICM45686::DumpKeyRegisters(const char *stage) { logKeyRegisters(stage); }

bool ICM45686::ReadUiSnapshot(uint8_t *buf, uint16_t len) {
  if (buf == nullptr || len == 0U) {
    return false;
  }

  if (!configured_ && DebugInit() != 0) {
    return false;
  }

  return readRegisters(kRegAccelDataX1Ui, buf, len);
}

bool ICM45686::ReadRaw(IMURawData &data) {
  uint16_t fifo_frame_count = 0;
  uint16_t bytes_to_read = 0;
  int count_retry = 0;

  data.timestamp_us = GetTimeUs();
  data.packet_size = kPacketSize16;
  data.fifo_count = 0;
  rt_memset(data.fifo_data, 0, sizeof(data.fifo_data));

  if (!configured_ && DebugInit() != 0) {
    return false;
  }

  /*
   * 和当前手写配置路径保持一致:
   * - FIFO_COUNT 按 frame count 读取
   * - 刚启动或 flush 后短时间内可能暂时为 0，做一个轻量重试
   */
  for (count_retry = 0; count_retry < 3; ++count_retry) {
    if (!readFifoByteCount(&fifo_frame_count)) {
      return false;
    }
    if (fifo_frame_count != 0U) {
      break;
    }
    DelayMs(2);
  }

  if (fifo_frame_count == 0U) {
    return false;
  }

  /*
   * 不再丢弃“最新一帧”:
   * FIFO 只有 1 帧时，这里减 1 会直接把有效数据吞成 0。
   */
  if (fifo_frame_count > kImuMaxPacketCount) {
    fifo_frame_count = kImuMaxPacketCount;
  }

  bytes_to_read = static_cast<uint16_t>(fifo_frame_count * kPacketSize16);
  if (bytes_to_read == 0U) {
    return false;
  }

  if (!readFifoData(data.fifo_data, bytes_to_read)) {
    return false;
  }

  data.fifo_count = bytes_to_read;
  return true;
}

uint8_t ICM45686::lastWhoAmI() const { return last_whoami_; }

}  // namespace drvf
