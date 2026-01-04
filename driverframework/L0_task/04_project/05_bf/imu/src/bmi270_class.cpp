/**
 * @file accgyro_spi_bmi270.cpp
 *
 * 基于 Betaflight BMI270 寄存器配置的简化版 RT-Thread C++ 驱动：
 * - 只使用数据就绪中断 + 直接寄存器读取，不使用 FIFO；
 * - 将原始加速度/角速度转换为 float，并通过 imu MCN 话题发布。
 * - 支持通过 Kconfig 选择触发方式：硬件中断（DRDY 引脚）或软件定时器；
 * - 支持通过 Kconfig 选择输出数据率（ODR）：800Hz、1600Hz、3200Hz。
 */

#include "bmi270_class.h"
// #include "../mlog/inc/mlog_gyro.hpp"  // 已屏蔽：由 testThread 模块负责 mlog 记录

extern "C" {
#include "timestamp.h"
#include "pinInterface.h"
#include <ulog.h>
#include <stdlib.h>  // for rand()
#include "debugPin.h"
#include "init_sync.h"  // For initSyncNotify
}

/* 定义 IMU 原始数据话题（在本文件内完成定义与发布）
 * 类型 gyro_raw_msg_t 和 acc_raw_msg_t 在 imu_mcn.h 中定义
 */
MCN_DEFINE(gyro_raw, sizeof(gyro_raw_msg_t));
MCN_DEFINE(acc_raw, sizeof(acc_raw_msg_t));

#undef LOG_TAG
#define LOG_TAG "bmi270_bf"
#ifndef LOG_LVL
#define LOG_LVL LOG_LVL_INFO
#endif

// MCN echo 函数
extern "C" {
static int gyro_raw_echo(void* parameter) {
  gyro_raw_msg_t gyro_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &gyro_data) != RT_EOK) {
    return -1;
  }

  LOG_I("gyro_raw: seq=%lu g:%.2f %.2f %.2f", gyro_data.seq,
        gyro_data.gyro[0], gyro_data.gyro[1], gyro_data.gyro[2]);
  return 0;
}

static int acc_raw_echo(void* parameter) {
  acc_raw_msg_t acc_data;

  if (mcn_copy_from_hub((McnHub*)parameter, &acc_data) != RT_EOK) {
    return -1;
  }

  LOG_I("acc_raw: seq=%lu a:%.2f %.2f %.2f", acc_data.seq,
        acc_data.accel[0], acc_data.accel[1], acc_data.accel[2]);
  return 0;
}
}

namespace bf_bmi270 {

namespace {

// BMI270 配置固件数组（从 bmi270_maximum_fifo.c 整合而来）
// Global array that stores the configuration file of BMI270
static const uint8_t bmi270_maximum_fifo_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

// 提供配置数组长度，方便其它模块使用
static const uint32_t bmi270_maximum_fifo_config_file_size = sizeof(bmi270_maximum_fifo_config_file);

// BMI270 部分寄存器（与 accgyro_spi_bmi270.c 中保持一致）
enum bmi270Register_e : uint8_t {
  BMI270_REG_CHIP_ID = 0x00,
  BMI270_REG_ACC_DATA_X_LSB = 0x0C,
  BMI270_REG_GYR_DATA_X_LSB = 0x12,
  BMI270_REG_STATUS = 0x03,
  BMI270_REG_PWR_CONF = 0x7C,
  BMI270_REG_PWR_CTRL = 0x7D,
  BMI270_REG_ACC_CONF = 0x40,
  BMI270_REG_ACC_RANGE = 0x41,
  BMI270_REG_GYRO_CONF = 0x42,
  BMI270_REG_GYRO_RANGE = 0x43,
  BMI270_REG_INT1_IO_CTRL = 0x53,
  BMI270_REG_INT_MAP_DATA = 0x58,
  BMI270_REG_INIT_CTRL = 0x59,
  BMI270_REG_INIT_DATA = 0x5E,
  BMI270_REG_CMD = 0x7E,
};

// 关键配置值，直接参考 accgyro_spi_bmi270.c
static constexpr uint8_t BMI270_CHIP_ID = 0x24;
static constexpr uint8_t BMI270_VAL_CMD_SOFTRESET = 0xB6;
static constexpr uint8_t BMI270_VAL_PWR_CTRL = 0x0E;              // gyro+acc+temp 使能
static constexpr uint8_t BMI270_VAL_PWR_CONF = 0x02;              // 性能模式
static constexpr uint8_t BMI270_VAL_ACC_RANGE_16G = 0x03;
static constexpr uint8_t BMI270_VAL_GYRO_RANGE_2000DPS = 0x08;
static constexpr uint8_t BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04;
static constexpr uint8_t BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A;  // 高电平、推挽、输出

#if defined(SENSOR_BMI270_BF_ODR_3200HZ)
// TO DO: 为什么 ACC的ODR必须是800Hz,否则acc[2]在静止的时候数据是2G
static constexpr uint8_t BMI270_VAL_ACC_CONF_ODR_BITS = 0x0B;  // ACC固定800Hz
static constexpr uint8_t BMI270_VAL_GYRO_CONF_ODR_BITS = 0x0D;  // GYRO 3200Hz
static constexpr float BMI270_SELECTED_ODR_HZ = 3200.0f;  // GYRO ODR
static constexpr float BMI270_ACC_ODR_HZ = 800.0f;  // ACC ODR (固定800Hz)
#elif defined(SENSOR_BMI270_BF_ODR_1600HZ)
static constexpr uint8_t BMI270_VAL_ACC_CONF_ODR_BITS = 0x0B;  // ACC固定800Hz
static constexpr uint8_t BMI270_VAL_GYRO_CONF_ODR_BITS = 0x0C;  // GYRO 1600Hz
static constexpr float BMI270_SELECTED_ODR_HZ = 1600.0f;  // GYRO ODR
static constexpr float BMI270_ACC_ODR_HZ = 800.0f;  // ACC ODR (固定800Hz)
#else  // 默认 800Hz
static constexpr uint8_t BMI270_VAL_ACC_CONF_ODR_BITS = 0x0B;  // ACC 800Hz
static constexpr uint8_t BMI270_VAL_GYRO_CONF_ODR_BITS = 0x0B;  // GYRO 800Hz
static constexpr float BMI270_SELECTED_ODR_HZ = 800.0f;  // GYRO ODR
static constexpr float BMI270_ACC_ODR_HZ = 800.0f;  // ACC ODR
#endif

static constexpr uint8_t BMI270_VAL_ACC_CONF =
    (0x01u << 7) | (0x01u << 4) | BMI270_VAL_ACC_CONF_ODR_BITS;   // 高性能 + osr2 + 选择 ODR
// TO DO : OSR根据用户参数来配置
static constexpr uint8_t BMI270_VAL_GYRO_CONF =
    (0x01u << 7) | (0x01u << 6) | BMI270_VAL_GYRO_CONF_ODR_BITS;  // HP filter/noise + OSR4 + 选择 ODR

// 原始值转换系数（从 Betaflight 抽取，与 accgyro_spi_bmi270.c 保持一致）
// 16G: 32768 -> 16g
static constexpr float ACC_SCALE_16G = 16.0f / 32768.0f;
// 陀螺仪比例因子（对应 accgyro_spi_bmi270.c 中的定义）
// 2000dps: 2000.0f / (1 << 15) = 16.384 dps/lsb scalefactor for 2000dps sensors
static constexpr float GYRO_SCALE_2000DPS = 2000.0f / (1 << 15);
// 4000dps: 4000.0f / (1 << 15) = 8.192 dps/lsb scalefactor for 4000dps sensors
static constexpr float GYRO_SCALE_4000DPS = 4000.0f / (1 << 15);

static inline int16_t combine(uint8_t msb, uint8_t lsb) {
  return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

}  // namespace

BMI270 &BMI270::instance() {
  static BMI270 inst;
  return inst;
}

BMI270::BMI270()
    : spi_{},
      spi_inited_(false),
      init_ok_(false),
      int_pin_(-1),
      event_inited_(false),
      timer_inited_(false),  // 调整顺序，在 worker_inited_ 之前初始化
      worker_thread_(RT_NULL),
      worker_inited_(false),
      gyro_sample_rate_hz_(BMI270_SELECTED_ODR_HZ),
      gyro_sample_dt_(1.0f / BMI270_SELECTED_ODR_HZ),
      acc_sample_rate_hz_(BMI270_ACC_ODR_HZ),
      imu_process_denom_(2),  // 默认分频因子为2（800Hz acc -> 400Hz attitude）
      gyro_scale_(GYRO_SCALE_2000DPS) {  // BMI270 陀螺仪配置为 2000DPS
  cfg_ = {};
  rt_memset(&timer_, 0, sizeof(timer_));
}

BMI270::~BMI270() {
  disableInterrupt();
}

rt_err_t BMI270::init(const RuntimeConfig &cfg) {
  cfg_ = cfg;
  init_ok_ = false;

#ifdef PROJECT_BF_BMI270_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG3_HIGH();  // Debug pin: BMI270 initialization start
#endif

  if (!cfg_.spi_bus_name || !cfg_.spi_device_name) {
    return -RT_EINVAL;
  }

  // 使用 SpiInterface 初始化 SPI 设备和 CS 引脚（方式与 PX4 版本保持一致）
  if (!spi_.init(cfg_.spi_bus_name, cfg_.spi_device_name, cfg_.cs_pin_name)) {
    LOG_E("BMI270 spi init failed");
    return -RT_ERROR;
  }

  // 配置 SPI：MODE3，MSB first，使用 RT_SPI_MODE_MASK
  rt_uint16_t mode = (RT_SPI_MODE_3 | RT_SPI_MSB) & RT_SPI_MODE_MASK;
  if (!spi_.configure(mode, cfg_.spi_max_hz ? cfg_.spi_max_hz : 10000000)) {
    LOG_E("BMI270 spi configure failed");
    return -RT_ERROR;
  }

  spi_inited_ = true;

  if (!resetSensor()) {
    return -RT_ERROR;
  }

  // 加载 Bosch 提供的 BMI270 配置固件（microcode）
  if (!loadConfigFile()) {
    return -RT_ERROR;
  }

  if (!configureSensor()) {
    return -RT_ERROR;
  }

  /* 初始化事件与工作线程（静态创建） */
  if (!event_inited_) {
    if (rt_event_init(&event_, "b270_evt", RT_IPC_FLAG_FIFO) != RT_EOK) {
      LOG_E("BMI270 event init failed");
      return -RT_ERROR;
    }
    event_inited_ = true;
  }

  if (!configureInterrupt()) {
    // 如果中断失败，仍然允许后续通过轮询方式调试使用
    // 这里只记录失败，不直接返回错误
    LOG_W("BMI270 interrupt configure failed, will use polling mode");
  }

  if (!worker_inited_) {
    rt_err_t err = rt_thread_init(&worker_thread_obj_, "b270", &BMI270::workerEntry, this, worker_stack_,
                                  THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (err != RT_EOK) {
      LOG_E("BMI270 worker thread init failed: %d", err);
      return err;
    }
    worker_thread_ = &worker_thread_obj_;
    worker_inited_ = true;
    rt_thread_startup(worker_thread_);
  }

  // 临时测试：初始化 mlog_gyro（使用单例）
  // 已屏蔽：由 testThread 模块负责 mlog 记录
  // bf_mlog::MlogGyro* mlog_gyro = bf_mlog::MlogGyro::getInstance();
  // mlog_gyro->init();
  // mlog_gyro->setParamEnabled(true);  // 临时测试：直接使能

  init_ok_ = true;
  
  // 通知初始化完成
  rt_err_t notify_ret = initSyncNotify(INIT_SYNC_BMI270);
  if (notify_ret != RT_EOK) {
    LOG_E("BMI270 initSyncNotify failed: %d", notify_ret);
  } else {
    LOG_I("BMI270 initialization completed and notified");
  }
  
#ifdef PROJECT_BF_BMI270_DEBUG_PIN_EN
  DEBUG_PIN_DEBUG3_LOW();  // Debug pin: BMI270 initialization completed
#endif
  
  return RT_EOK;
}

bool BMI270::resetSensor() {
  // 软复位
  regWrite(BMI270_REG_CMD, BMI270_VAL_CMD_SOFTRESET, 100);

  // 多次读取 chip id 做探测，增加鲁棒性
  const uint8_t kMaxRetry = 5;
  uint8_t id = 0;

  for (uint8_t i = 0; i < kMaxRetry; i++) {
    id = regRead(BMI270_REG_CHIP_ID);
    if (id == BMI270_CHIP_ID) {
      LOG_I("BMI270 probe OK, chip id: 0x%02X (retry=%u)", id, i);
      return true;
    }
    rt_thread_mdelay(2);
  }

  LOG_E("BMI270 probe failed, unexpected chip id: 0x%02X (expected 0x%02X)", id, BMI270_CHIP_ID);
  return false;
}

bool BMI270::loadConfigFile() {
  if (!spi_inited_) {
    return false;
  }

  // 关闭高级省电，准备写入配置
  regWrite(BMI270_REG_PWR_CONF, 0x00, 1);
  regWrite(BMI270_REG_INIT_CTRL, 0x00, 1);

  // 写入 Bosch 提供的配置文件到 INIT_DATA
  if (spi_.writeMultiReg8(BMI270_REG_INIT_DATA, (uint8_t*)bmi270_maximum_fifo_config_file,
                          (uint16_t)bmi270_maximum_fifo_config_file_size) != RT_EOK) {
    return false;
  }

  // 启动配置解析
  regWrite(BMI270_REG_INIT_CTRL, 0x01, 1);
  rt_thread_mdelay(10);

  return true;
}

bool BMI270::configureSensor() {
  if (!spi_inited_) {
    return false;
  }

  // 电源配置
  regWrite(BMI270_REG_PWR_CONF, BMI270_VAL_PWR_CONF, 1);
  regWrite(BMI270_REG_PWR_CTRL, BMI270_VAL_PWR_CTRL, 1);

  // 加速度计配置：由 Kconfig 选择的 ODR，16G
  regWrite(BMI270_REG_ACC_CONF, BMI270_VAL_ACC_CONF, 1);
  regWrite(BMI270_REG_ACC_RANGE, BMI270_VAL_ACC_RANGE_16G, 1);

  // 陀螺仪配置：由 Kconfig 选择的 ODR，2000dps
  regWrite(BMI270_REG_GYRO_CONF, BMI270_VAL_GYRO_CONF, 1);
  regWrite(BMI270_REG_GYRO_RANGE, BMI270_VAL_GYRO_RANGE_2000DPS, 1);

  // 映射 data ready 中断到 INT1
  regWrite(BMI270_REG_INT_MAP_DATA, BMI270_VAL_INT_MAP_DATA_DRDY_INT1, 1);

  // 配置 INT1 引脚行为（高电平、推挽、输出，边沿触发）
  regWrite(BMI270_REG_INT1_IO_CTRL, BMI270_VAL_INT1_IO_CTRL_PINMODE, 1);

  // 清一次状态寄存器
  (void)regRead(BMI270_REG_STATUS);

  return true;
}

bool BMI270::configureInterrupt() {
  disableInterrupt();

#ifdef SENSOR_BMI270_BF_TRIGGER_INTERRUPT
  // 使用硬件中断方式
  if (!cfg_.int_pin_name || cfg_.int_pin_name[0] == '\0') {
    LOG_E("BMI270 interrupt pin not configured");
    return false;
  }

  // 解析 GPIO 引脚索引（支持 "PA15" 之类的字符串配置）
  int_pin_ = parse_pin_name_from_config(cfg_.int_pin_name);
  if (int_pin_ < 0) {
    int_pin_ = -1;
    LOG_E("BMI270 failed to parse interrupt pin: %s", cfg_.int_pin_name);
    return false;
  }

  rt_pin_mode(int_pin_, PIN_MODE_INPUT_PULLDOWN);

  // 先读一次状态寄存器，清掉可能存在的 pending 中断
  (void)regRead(BMI270_REG_STATUS);

  // 实际硬件为"高电平有效 + 上升沿触发"，使用上升沿 EXTI
  if (rt_pin_attach_irq(int_pin_, PIN_IRQ_MODE_RISING, &BMI270::drdyIsr, this) != RT_EOK) {
    LOG_E("BMI270 failed to attach interrupt");
    int_pin_ = -1;
    return false;
  }

  if (rt_pin_irq_enable(int_pin_, PIN_IRQ_ENABLE) != RT_EOK) {
    LOG_E("BMI270 failed to enable interrupt");
    rt_pin_detach_irq(int_pin_);
    int_pin_ = -1;
    return false;
  }

  LOG_I("BMI270 using hardware interrupt (pin: %s)", cfg_.int_pin_name);
  return true;

#else  // SENSOR_BMI270_BF_TRIGGER_TIMER
  // 使用软件定时器方式
  if (!timer_inited_ && event_inited_) {
    // 根据选择的 ODR 计算定时器周期（tick）
    // 定时器周期 = RT_TICK_PER_SECOND / ODR_HZ
    // 为了确保至少 1 tick，使用最大值
    rt_tick_t timer_period = RT_TICK_PER_SECOND / static_cast<rt_tick_t>(BMI270_SELECTED_ODR_HZ);
    if (timer_period < 1) {
      timer_period = 1;
    }

    // 创建软件定时器
    // 注意：rt_timer_init 返回 void，不返回错误码
    rt_timer_init(&timer_, "b270_tmr", &BMI270::timerCallback, this,
                  timer_period,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);

    // 启动定时器
    rt_err_t ret = rt_timer_start(&timer_);
    if (ret != RT_EOK) {
      LOG_E("BMI270 timer start failed: %d", ret);
      rt_timer_detach(&timer_);
      return false;
    }

    timer_inited_ = true;
    LOG_I("BMI270 using software timer (period: %lu ticks, ODR: %.0f Hz)", 
          timer_period, BMI270_SELECTED_ODR_HZ);
  }

  return true;
#endif
}

void BMI270::disableInterrupt() {
#ifdef SENSOR_BMI270_BF_TRIGGER_INTERRUPT
  // 禁用硬件中断
  if (int_pin_ >= 0) {
    rt_pin_irq_enable(int_pin_, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(int_pin_);
    int_pin_ = -1;
  }
#else  // SENSOR_BMI270_BF_TRIGGER_TIMER
  // 停止软件定时器
  if (timer_inited_) {
    rt_timer_stop(&timer_);
    rt_timer_detach(&timer_);
    timer_inited_ = false;
  }
#endif
}

bool BMI270::readImu(int16_t acc[3], int16_t gyro[3]) {
  if (!spi_inited_) {
    return false;
  }

  // 一次性读取加速度和陀螺仪数据（12字节）
  // BMI270 寄存器布局：0x0C (ACC_X_LSB) 到 0x17 (GYR_Z_MSB)
  // 0x0C-0x11: 加速度数据（6字节）
  // 0x12-0x17: 陀螺仪数据（6字节）
  uint8_t data_buf[12] = {0};

  // 从 ACC_DATA_X_LSB (0x0C) 开始读取 12 字节，一次性获取加速度和陀螺仪数据
  if (spi_.readMultiReg16(BMI270_REG_ACC_DATA_X_LSB, data_buf, 12) != RT_EOK) {
    return false;
  }

  // 解析加速度数据（前 6 字节，offset 0-5）
  acc[0] = combine(data_buf[1], data_buf[0]);
  acc[1] = combine(data_buf[3], data_buf[2]);
  acc[2] = combine(data_buf[5], data_buf[4]);

  // 解析陀螺仪数据（offset 6-11，对应寄存器 0x12-0x17）
  // 0x12 相对于 0x0C 的偏移是 6 字节（0x12 - 0x0C = 6）
  uint8_t gyro_offset = 6;
  gyro[0] = combine(data_buf[gyro_offset + 1], data_buf[gyro_offset + 0]);
  gyro[1] = combine(data_buf[gyro_offset + 3], data_buf[gyro_offset + 2]);
  gyro[2] = combine(data_buf[gyro_offset + 5], data_buf[gyro_offset + 4]);

  return true;
}

bool BMI270::readGyroOnly(int16_t gyro[3]) {
  if (!spi_inited_) {
    return false;
  }

  // 只读取陀螺仪数据（6字节）
  // BMI270 寄存器布局：0x12 (GYR_X_LSB) 到 0x17 (GYR_Z_MSB)
  uint8_t data_buf[6] = {0};

  // 从 GYR_DATA_X_LSB (0x12) 开始读取 6 字节
  if (spi_.readMultiReg16(BMI270_REG_GYR_DATA_X_LSB, data_buf, 6) != RT_EOK) {
    return false;
  }

  // 解析陀螺仪数据
  gyro[0] = combine(data_buf[1], data_buf[0]);
  gyro[1] = combine(data_buf[3], data_buf[2]);
  gyro[2] = combine(data_buf[5], data_buf[4]);

  return true;
}

void BMI270::publishGyro(const int16_t gyro[3]) {
  static rt_uint32_t s_seq = 0;
  gyro_raw_msg_t msg{};

  msg.seq = ++s_seq;
  msg.gyro[0] = GYRO_SCALE_2000DPS * static_cast<float>(gyro[0]);
  msg.gyro[1] = GYRO_SCALE_2000DPS * static_cast<float>(gyro[1]);
  msg.gyro[2] = GYRO_SCALE_2000DPS * static_cast<float>(gyro[2]);
  mcn_publish(MCN_HUB(gyro_raw), &msg);
}

void BMI270::publishAcc(const int16_t acc[3]) {
  static rt_uint32_t s_seq = 0;
  acc_raw_msg_t msg{};

  msg.seq = ++s_seq;
  // Betaflight 风格：acc 保持原始 ADC 值（不缩放），缩放将在 attitude 更新中进行
  msg.accel[0] = static_cast<float>(acc[0]);
  msg.accel[1] = static_cast<float>(acc[1]);
  msg.accel[2] = static_cast<float>(acc[2]);
  mcn_publish(MCN_HUB(acc_raw), &msg);
}

uint8_t BMI270::regRead(uint8_t reg) {
  if (!spi_inited_) {
    return 0;
  }
  uint8_t value[2] = {0};
  if (spi_.readMultiReg16(static_cast<uint8_t>(reg), value, 1) != RT_EOK) {
    return 0;
  }
  return value[0];
}

void BMI270::regWrite(uint8_t reg, uint8_t value, unsigned delayMs) {
  if (!spi_inited_) {
    return;
  }

  uint8_t v = value;
  (void)spi_.writeMultiReg8(static_cast<uint8_t>(reg), &v, 1);

  if (delayMs) {
    rt_thread_mdelay(delayMs);
  }
}

void BMI270::drdyIsr(void *parameter) {
  auto *self = static_cast<BMI270 *>(parameter);
  if (!self || !self->init_ok_ || !self->event_inited_) {
    return;
  }

  /* 在中断中只发送事件，让工作线程去做 SPI 读取和发布 */
  rt_event_send(&self->event_, 0x01);
}

/* 软件定时器回调（通过 Kconfig 选择使用定时器模式时调用） */
void BMI270::timerCallback(void* parameter) {
  auto* self = static_cast<BMI270*>(parameter);
  if (!self || !self->init_ok_ || !self->event_inited_) {
    return;
  }

  /* 在定时器回调中发送事件，和中断回调一样，让工作线程去做 SPI 读取和发布 */
  rt_event_send(&self->event_, 0x01);
}

void BMI270::workerLoop() {
  // 计数器：每4次中断，前3次只读gyro，第4次读acc+gyro
  uint8_t interrupt_counter = 0;
  
  while (true) {
    if (!event_inited_) {
      rt_thread_mdelay(10);
      continue;
    }

    rt_uint32_t recved = 0;
    rt_err_t err = rt_event_recv(&event_, 0x01,
                                 RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                                 RT_WAITING_FOREVER, &recved);
    if (err != RT_EOK) {
      continue;
    }

    interrupt_counter++;
    
    // 前3次中断：只读取和发布gyro数据（只读取6字节）
    if (interrupt_counter < 4) {
      int16_t gyro[3] = {0};
      // 使用readGyroOnly只读取6字节的gyro数据
      if (readGyroOnly(gyro)) {
        publishGyro(gyro);
      }
    } else {
      // 第4次中断：读取acc+gyro，同时发布两个主题
      int16_t acc[3] = {0};
      int16_t gyro[3] = {0};
      
      // 使用readImu一次读取12字节，然后分别发布acc和gyro
      if (readImu(acc, gyro)) {
        publishAcc(acc);
        publishGyro(gyro);
      }
      
      // 重置计数器
      interrupt_counter = 0;
    }
  }
}

void BMI270::workerEntry(void *parameter) {
  auto *self = static_cast<BMI270 *>(parameter);
  if (!self) {
    return;
  }
  self->workerLoop();
}

rt_err_t bf_bmi270_init_default() {
  RuntimeConfig cfg{};

  // 从 Kconfig 中读取默认配置
  cfg.spi_bus_name = SENSOR_SPI_NAME_BMI270_BF;
  cfg.spi_device_name = SENSOR_SPI_SLAVE_NAME_BMI270_BF;
  cfg.cs_pin_name = SENSOR_BMI270_BF_SPI_CS_PIN;
  cfg.int_pin_name = SENSOR_BMI270_BF_INT_PIN;
  cfg.spi_max_hz = SENSOR_BMI270_BF_SPI_MAX_HZ;

  // MCN 话题 gyro_raw 的广告发布
  rt_err_t ret = mcn_advertise(MCN_HUB(gyro_raw), gyro_raw_echo);
  if (ret != RT_EOK && ret != -RT_EBUSY) {
    LOG_E("gyro_raw advertise failed: %d", ret);
    return ret;
  }

  // MCN 话题 acc_raw 的广告发布（acc_raw由IMU发布）
  ret = mcn_advertise(MCN_HUB(acc_raw), acc_raw_echo);
  if (ret != RT_EOK && ret != -RT_EBUSY) {
    LOG_E("acc_raw advertise failed: %d", ret);
    return ret;
  }

  return BMI270::instance().init(cfg);
}

}  // namespace bf_bmi270


extern "C" {

#ifdef PROJECT_BF_BMI270_EN
/* RT-Thread 自动初始化包装函数，调用命名空间内的实现 */
static int bf_bmi270_init_default_wrapper(void) {
  return (int)bf_bmi270::bf_bmi270_init_default();
}
INIT_APP_EXPORT(bf_bmi270_init_default_wrapper);
#endif

}
