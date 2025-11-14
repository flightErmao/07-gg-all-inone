#include "BMI270.hpp"

#include "Bosch_BMI270_registers.hpp"
#include "pinInterface.h"

extern "C" {
#include <rtdef.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include <rtthread.h>
#include <drivers/dev_pin.h>
#include "rtconfig.h"
#include "timestamp.h"

#include <string.h>
}

#ifndef RT_UNUSED
#define RT_UNUSED(x) ((void)(x))
#endif

#include <climits>

#define BMI270_DEBUG

#define LOG_TAG "bmi270"
#ifdef BMI270_DEBUG
#define LOG_LVL LOG_LVL_DBG
#else
#define LOG_LVL LOG_LVL_WARNING
#endif
#include <ulog.h>

namespace bmi270_px4 {

using namespace Bosch_BMI270;

namespace {

static constexpr uint32_t kRateHz = 1600;
static constexpr uint32_t kSampleDtUs = 1000000UL / kRateHz;
static constexpr uint8_t kMaxIdRetry = 5;
static constexpr uint8_t kMaxFailures = 5;
static constexpr uint32_t kResetTimeoutUs = 1000ULL * 1000ULL;
static constexpr uint8_t kDefaultWatermarkSamples = 2;
static inline rt_tick_t us_to_ticks(uint32_t us) {
  const uint32_t min_us = 1000;
  if (us < min_us) {
    us = min_us;
  }
  rt_tick_t ticks = rt_tick_from_millisecond(us / 1000);
  if (ticks == 0) {
    ticks = 1;
  }
  return ticks;
}

static inline int16_t combine(uint8_t msb, uint8_t lsb) {
  return (int16_t)(((uint16_t)msb << 8) | lsb);
}

static inline int16_t safe_negate(int16_t value) { return (value == INT16_MIN) ? INT16_MAX : (int16_t)(-value); }

static uint8_t maximum_fifo_config_file[] = {
    0x5E, 0xC8, 0x2E, 0x00, 0x2E, 0x80, 0x2E, 0x1A, 0x00, 0xC8, 0x2E, 0x00, 0x2E, 0xC8, 0x2E,
    0x00, 0x2E, 0xC8, 0x2E, 0x00, 0x2E, 0xC8, 0x2E, 0x00, 0x2E, 0xC8, 0x2E, 0x00, 0x2E, 0xC8,
    0x2E, 0x00, 0x2E, 0x90, 0x32, 0x21, 0x2E, 0x59, 0xF5, 0x10, 0x30, 0x21, 0x2E, 0x6A, 0xF5,
    0x1A, 0x24, 0x22, 0x00, 0x80, 0x2E, 0x3B, 0x00, 0xC8, 0x2E, 0x44, 0x47, 0x22, 0x00, 0x37,
    0x00, 0xA4, 0x00, 0xFF, 0x0F, 0xD1, 0x00, 0x07, 0xAD, 0x80, 0x2E, 0x00, 0xC1, 0x80, 0x2E,
    0x00, 0xC1, 0x80, 0x2E, 0x00, 0xC1, 0x80, 0x2E, 0x00, 0xC1, 0x80, 0x2E, 0x00, 0xC1, 0x80,
    0x2E, 0x00, 0xC1, 0x80, 0x2E, 0x00, 0xC1, 0x80, 0x2E, 0x00, 0xC1, 0x80, 0x2E, 0x00, 0xC1,
    0x80, 0x2E, 0x00, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xFC, 0xF5, 0x80,
    0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xEB, 0x00, 0x03, 0x30, 0x00, 0x2E,
    0xC1, 0x86, 0x5A, 0x0E, 0xFB, 0x2F, 0x21, 0x2E, 0xFC, 0xF5, 0x13, 0x24, 0x63, 0xF5, 0xE0,
    0x3C, 0x48, 0x00, 0x22, 0x30, 0xF7, 0x80, 0xC2, 0x42, 0xE1, 0x7F, 0x3A, 0x25, 0xFC, 0x86,
    0xF0, 0x7F, 0x41, 0x33, 0x98, 0x2E, 0xC2, 0xC4, 0xD6, 0x6F, 0xF1, 0x30, 0xF1, 0x08, 0xC4,
    0x6F, 0x11, 0x24, 0xFF, 0x03, 0x12, 0x24, 0x00, 0xFC, 0x61, 0x09, 0xA2, 0x08, 0x36, 0xBE,
    0x2A, 0xB9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xBB, 0xD1, 0xBE, 0x94, 0x0A, 0x71, 0x08, 0xD5,
    0x42, 0x21, 0xBD, 0x91, 0xBC, 0xD2, 0x42, 0xC1, 0x42, 0x00, 0xB2, 0xFE, 0x82, 0x05, 0x2F,
    0x50, 0x30, 0x21, 0x2E, 0x21, 0xF2, 0x00, 0x2E, 0x00, 0x2E, 0xD0, 0x2E, 0xF0, 0x6F, 0x02,
    0x30, 0x02, 0x42, 0x20, 0x26, 0xE0, 0x6F, 0x02, 0x31, 0x03, 0x40, 0x9A, 0x0A, 0x02, 0x42,
    0xF0, 0x37, 0x05, 0x2E, 0x5E, 0xF7, 0x10, 0x08, 0x12, 0x24, 0x1E, 0xF2, 0x80, 0x42, 0x83,
    0x84, 0xF1, 0x7F, 0x0A, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3B, 0x82, 0xF0, 0x6F, 0x00, 0x2E,
    0x00, 0x2E, 0xD0, 0x2E, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2E, 0x12, 0x40, 0x52, 0x42, 0x3E,
    0x84, 0x00, 0x40, 0x40, 0x42, 0x7E, 0x82, 0xE1, 0x7F, 0xF2, 0x7F, 0x98, 0x2E, 0x6A, 0xD6,
    0x21, 0x30, 0x23, 0x2E, 0x61, 0xF5, 0xEB, 0x2C, 0xE1, 0x6F};

}  // namespace

BMI270 &BMI270::instance() {
  static BMI270 inst;
  return inst;
}

BMI270::BMI270()
    : worker_thread_(RT_NULL),
      watchdog_timer_(RT_NULL),
      state_(State::RESET),
      worker_thread_inited_(false),
      init_started_(false),
      init_ok_(false),
      use_interrupt_(false),
      event_inited_(false),
      sample_sem_inited_(false),
      sample_mutex_inited_(false),
      timer_started_(false),
      int_pin_(-1),
      fifo_interval_us_(kSampleDtUs * kDefaultWatermarkSamples),
      reset_timestamp_us_(0),
      failure_count_(0) {
  memset(&config_, 0, sizeof(config_));
  memset(&latest_, 0, sizeof(latest_));
  memset(&worker_thread_obj_, 0, sizeof(worker_thread_obj_));
  memset(worker_thread_stack_, 0, sizeof(worker_thread_stack_));
}

BMI270::~BMI270() {
  disableInterrupt();

  if (watchdog_timer_) {
    rt_timer_stop(watchdog_timer_);
    rt_timer_delete(watchdog_timer_);
    watchdog_timer_ = RT_NULL;
  }

  if (event_inited_) {
    rt_event_detach(&event_);
    event_inited_ = false;
  }

  if (sample_sem_inited_) {
    rt_sem_detach(&sample_sem_);
    sample_sem_inited_ = false;
  }

  if (sample_mutex_inited_) {
    rt_mutex_detach(&sample_mutex_);
    sample_mutex_inited_ = false;
  }

  if (worker_thread_inited_) {
    rt_thread_detach(&worker_thread_obj_);
    worker_thread_inited_ = false;
    worker_thread_ = RT_NULL;
  }
  init_started_ = false;
}

rt_err_t BMI270::init(const RuntimeConfig &cfg) {
  config_ = cfg;

  if (init_started_) {
    return RT_EOK;
  }

  if (!config_.spi_bus_name || !config_.spi_device_name) {
    LOG_E("BMI270 config invalid");
    return -RT_EINVAL;
  }

  if (config_.fifo_watermark_samples == 0) {
    config_.fifo_watermark_samples = kDefaultWatermarkSamples;
  }

  if (!spi_.init(config_.spi_bus_name, config_.spi_device_name, config_.cs_pin_name)) {
    LOG_E("BMI270 spi init failed");
    return -RT_ERROR;
  }

  if (!spi_.configure((RT_SPI_MODE_3 | RT_SPI_MSB) & RT_SPI_MODE_MASK, config_.spi_max_hz)) {
    LOG_E("BMI270 spi configure failed");
    return -RT_ERROR;
  }

  if (!event_inited_) {
    if (rt_event_init(&event_, "b270_evt", RT_IPC_FLAG_FIFO) != RT_EOK) {
      LOG_E("BMI270 event init failed");
      return -RT_ERROR;
    }
    event_inited_ = true;
  }

  if (!sample_sem_inited_) {
    if (rt_sem_init(&sample_sem_, "b270_sem", 0, RT_IPC_FLAG_FIFO) != RT_EOK) {
      LOG_E("BMI270 sem init failed");
      return -RT_ERROR;
    }
    sample_sem_inited_ = true;
  }

  if (!sample_mutex_inited_) {
    if (rt_mutex_init(&sample_mutex_, "b270_mtx", RT_IPC_FLAG_FIFO) != RT_EOK) {
      LOG_E("BMI270 mutex init failed");
      return -RT_ERROR;
    }
    sample_mutex_inited_ = true;
  }

  if (!watchdog_timer_) {
    watchdog_timer_ = rt_timer_create("b270_tmr", &BMI270::timerCallback, this, us_to_ticks(fifo_interval_us_),
                                      RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_PERIODIC);
    if (!watchdog_timer_) {
      LOG_E("BMI270 timer create failed");
      return -RT_ERROR;
    }
  }

  state_ = State::RESET;
  init_ok_ = false;
  use_interrupt_ = false;
  timer_started_ = false;
  failure_count_ = 0;
  reset_timestamp_us_ = 0;
  latest_.valid = false;

  if (!worker_thread_inited_) {
    rt_err_t err = rt_thread_init(&worker_thread_obj_, "b270", &BMI270::workerEntry, this, worker_thread_stack_,
                                  THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (err != RT_EOK) {
      LOG_E("BMI270 thread init failed: %d", err);
      return err;
    }
    worker_thread_inited_ = true;
    worker_thread_ = &worker_thread_obj_;
    rt_thread_startup(worker_thread_);
  }

  init_started_ = true;
  return RT_EOK;
}

void BMI270::workerEntry(void *parameter) {
  auto *self = static_cast<BMI270 *>(parameter);
  if (!self) {
    return;
  }
  self->workerLoop();
}

void BMI270::workerLoop() {
  while (true) {
    RunImpl();
  }
}

void BMI270::RunImpl() {
  switch (state_) {
    case State::RESET:
      if (resetSensor()) {
        state_ = State::WAIT_FOR_RESET;
        rt_thread_mdelay(1);
      } else {
        state_ = State::ERROR;
      }
      break;

    case State::WAIT_FOR_RESET:
      if (waitResetAndCheckId()) {
        state_ = State::MICROCODE_LOAD;
      } else {
        if (++failure_count_ >= kMaxFailures) {
          LOG_E("BMI270 wait for reset timeout");
          state_ = State::ERROR;
        } else {
          rt_thread_mdelay(10);
        }
      }
      break;

    case State::MICROCODE_LOAD:
      if (loadMicrocode()) {
        state_ = State::CONFIGURE;
      } else {
        LOG_E("BMI270 load microcode failed");
        state_ = State::ERROR;
      }
      break;

    case State::CONFIGURE:
      if (configureSensor()) {
        configureInterrupt();
        if (watchdog_timer_) {
          rt_tick_t ticks = us_to_ticks(fifo_interval_us_);
          rt_timer_control(watchdog_timer_, RT_TIMER_CTRL_SET_TIME, &ticks);
          rt_timer_start(watchdog_timer_);
          timer_started_ = true;
        }
        state_ = State::FIFO_READ;
      } else {
        if (++failure_count_ >= kMaxFailures) {
          LOG_E("BMI270 configure failed");
          state_ = State::RESET;
        } else {
          rt_thread_mdelay(20);
        }
      }
      break;

    case State::FIFO_READ: {
      uint32_t events = 0;
      if (event_inited_) {
        rt_err_t err = rt_event_recv(&event_, BMI270::EVENT_DRDY | BMI270::EVENT_TIMER,
                                     RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                     us_to_ticks(fifo_interval_us_ * 2), &events);
        if (err == -RT_ETIMEOUT) {
          events = BMI270::EVENT_TIMER;
        }
      }

      if (!fifoCycle()) {
        rt_thread_mdelay(2);
      }
    } break;

    case State::ERROR:
    default:
      disableInterrupt();
      if (watchdog_timer_ && timer_started_) {
        rt_timer_stop(watchdog_timer_);
        timer_started_ = false;
      }
      init_ok_ = false;
      rt_thread_mdelay(100);
      break;
  }
}

bool BMI270::resetSensor() {
  reset_timestamp_us_ = timestamp_micros();
  return regWrite(Register::CMD, 0xB6);
}

bool BMI270::waitResetAndCheckId() {
  for (uint8_t retry = 0; retry < kMaxIdRetry; ++retry) {
    uint8_t id = regRead(Register::CHIP_ID);
    if (id == chip_id) {
      regWrite(Register::PWR_CONF, 0x00);
      rt_thread_mdelay(1);
      return true;
    }
    rt_thread_mdelay(2);
  }

  const uint32_t elapsed = timestamp_micros() - reset_timestamp_us_;
  return elapsed <= kResetTimeoutUs;
}

bool BMI270::loadMicrocode() {
  if (!regWrite(Register::PWR_CONF, 0x00)) {
    return false;
  }

  if (!regWrite(Register::CONFIG1, 0x00)) {
    return false;
  }

  const uint16_t config_len = sizeof(maximum_fifo_config_file);
  if (config_len <= 1) {
    return false;
  }

  if (spi_.writeMultiReg8(static_cast<uint8_t>(Register::CONFIG2), &maximum_fifo_config_file[1], config_len - 1) !=
      RT_EOK) {
    return false;
  }

  rt_thread_mdelay(10);
  if (!regWrite(Register::CONFIG1, 0x01)) {
    return false;
  }

  rt_thread_mdelay(150);
  return true;
}

bool BMI270::configureSensor() {
  // uint8_t internal_status = regRead(Register::INTERNAL_STATUS);
  // if ((internal_status & 0x01) == 0) {
  //   return false;
  // }

  const uint16_t watermark_bytes = config_.fifo_watermark_samples * sizeof(FIFO::Data);

  bool ok = true;
  ok &= regWrite(Register::PWR_CTRL, PWR_CTRL_BIT::accel_en | PWR_CTRL_BIT::gyr_en | PWR_CTRL_BIT::temp_en);
  ok &= regWrite(Register::ACC_CONF,
                 static_cast<uint8_t>(ACC_CONF_BIT::acc_bwp_Normal | ACC_CONF_BIT::acc_odr_1600));
  ok &= regWrite(Register::ACC_RANGE, static_cast<uint8_t>(ACC_RANGE_BIT::acc_range_16g));
  ok &= regWrite(Register::GYR_CONF,
                 static_cast<uint8_t>(GYR_CONF_BIT::gyr_odr_1k6 | GYR_CONF_BIT::gyr_flt_mode_normal |
                                      GYR_CONF_BIT::gyr_noise_hp | GYR_CONF_BIT::gyr_flt_hp));
  ok &= regWrite(Register::FIFO_CONFIG_0,
                 static_cast<uint8_t>(FIFO_CONFIG_0_BIT::BIT1_ALWAYS | FIFO_CONFIG_0_BIT::FIFO_mode));
  ok &= regWrite(Register::FIFO_CONFIG_1,
                 static_cast<uint8_t>(FIFO_CONFIG_1_BIT::BIT4_ALWAYS | FIFO_CONFIG_1_BIT::Acc_en |
                                      FIFO_CONFIG_1_BIT::Gyr_en));
  ok &= regWrite(Register::FIFO_WTM_0, watermark_bytes & 0xFF);
  ok &= regWrite(Register::FIFO_WTM_1, (watermark_bytes >> 8) & 0x07);
  ok &= regWrite(Register::INT1_IO_CTRL,
                 static_cast<uint8_t>(INT1_IO_CONF_BIT::int1_out | INT1_IO_CONF_BIT::int1_lvl));
  ok &= regWrite(Register::INT_MAP_DATA, static_cast<uint8_t>(INT1_INT2_MAP_DATA_BIT::int1_fwm));

  if (!ok) {
    return false;
  }

  fifo_interval_us_ = config_.fifo_watermark_samples * kSampleDtUs;
  if (fifo_interval_us_ < kSampleDtUs) {
    fifo_interval_us_ = kSampleDtUs;
  }

  fifoReset();
  rt_thread_mdelay(5);
  return true;
}

bool BMI270::configureInterrupt() {
  disableInterrupt();

  if (!config_.int_pin_name || config_.int_pin_name[0] == '\0') {
    use_interrupt_ = false;
    return true;
  }

  int_pin_ = parse_pin_name_from_config(config_.int_pin_name);
  if (int_pin_ < 0) {
    use_interrupt_ = false;
    return false;
  }

  rt_pin_mode(int_pin_, PIN_MODE_INPUT_PULLDOWN);
  if (rt_pin_attach_irq(int_pin_, PIN_IRQ_MODE_RISING, &BMI270::drdyIsr, this) != RT_EOK) {
    use_interrupt_ = false;
    return false;
  }

  if (rt_pin_irq_enable(int_pin_, PIN_IRQ_ENABLE) != RT_EOK) {
    rt_pin_detach_irq(int_pin_);
    use_interrupt_ = false;
    return false;
  }

  use_interrupt_ = true;
  return true;
}

void BMI270::disableInterrupt() {
  if (int_pin_ >= 0) {
    rt_pin_irq_enable(int_pin_, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(int_pin_);
    int_pin_ = -1;
  }
  use_interrupt_ = false;
}

bool BMI270::fifoCycle() {
  uint16_t fifo_bytes = fifoLevel();
  if (fifo_bytes == 0) {
    LOG_D("BMI270 fifo empty");
    return false;
  } else {
    LOG_D("BMI270 fifo bytes: %u", fifo_bytes);
  }

  static uint8_t fifo_buffer[FIFO::SIZE];

  if (fifo_bytes > FIFO::SIZE) {
    fifo_bytes = FIFO::SIZE;
  }

  uint16_t remaining = fifo_bytes;
  uint16_t offset = 0;
  while (remaining > 0) {
    uint16_t chunk = remaining > 0xFFu ? 0xFFu : remaining;
    if (spi_.readMultiReg16(static_cast<uint8_t>(Register::FIFO_DATA), &fifo_buffer[offset],
                            static_cast<uint8_t>(chunk)) != RT_EOK) {
      fifoReset();
      return false;
    }
    remaining -= chunk;
    offset += chunk;
  }

  uint8_t* buffer = fifo_buffer;
  uint16_t index = 0;
  bool has_sample = false;
  int16_t accel[3] = {0};
  int16_t gyro[3] = {0};

  while (index < fifo_bytes) {
    const uint8_t header = buffer[index];

    if (header == ((uint8_t)FIFO::Header::sensor_accel_frame |
                   (uint8_t)FIFO::Header::sensor_gyro_frame)) {
      index += 1;
      if (index + 12 > fifo_bytes) {
        break;
      }
      auto *gyro_frame = reinterpret_cast<FIFO::Data *>(&buffer[index]);
      gyro[0] = combine(gyro_frame->x_msb, gyro_frame->x_lsb);
      gyro[1] = combine(gyro_frame->y_msb, gyro_frame->y_lsb);
      gyro[2] = combine(gyro_frame->z_msb, gyro_frame->z_lsb);
      index += 6;

      auto *accel_frame = reinterpret_cast<FIFO::Data *>(&buffer[index]);
      accel[0] = combine(accel_frame->x_msb, accel_frame->x_lsb);
      accel[1] = combine(accel_frame->y_msb, accel_frame->y_lsb);
      accel[2] = combine(accel_frame->z_msb, accel_frame->z_lsb);
      index += 6;
      has_sample = true;

    } else if (header == (uint8_t)FIFO::Header::sensor_gyro_frame) {
      index += 1;
      if (index + 6 > fifo_bytes) {
        break;
      }
      auto *gyro_frame = reinterpret_cast<FIFO::Data *>(&buffer[index]);
      gyro[0] = combine(gyro_frame->x_msb, gyro_frame->x_lsb);
      gyro[1] = combine(gyro_frame->y_msb, gyro_frame->y_lsb);
      gyro[2] = combine(gyro_frame->z_msb, gyro_frame->z_lsb);
      index += 6;
      has_sample = true;

    } else if (header == (uint8_t)FIFO::Header::sensor_accel_frame) {
      index += 1;
      if (index + 6 > fifo_bytes) {
        break;
      }
      auto *accel_frame = reinterpret_cast<FIFO::Data *>(&buffer[index]);
      accel[0] = combine(accel_frame->x_msb, accel_frame->x_lsb);
      accel[1] = combine(accel_frame->y_msb, accel_frame->y_lsb);
      accel[2] = combine(accel_frame->z_msb, accel_frame->z_lsb);
      index += 6;
      has_sample = true;

    } else if (header == (uint8_t)FIFO::Header::skip_frame ||
               header == (uint8_t)FIFO::Header::FIFO_input_config_frame ||
               header == (uint8_t)FIFO::Header::sample_drop_frame) {
      index += 2;

    } else if (header == (uint8_t)FIFO::Header::sensor_time_frame) {
      index += 4;

    } else {
      index += 1;
    }
  }

  if (!has_sample) {
    return false;
  }

  int16_t rotated_accel[3] = {
      accel[0],
      safe_negate(accel[1]),
      safe_negate(accel[2]),
  };

  int16_t rotated_gyro[3] = {
      gyro[0],
      safe_negate(gyro[1]),
      safe_negate(gyro[2]),
  };

  const uint32_t timestamp_us = timestamp_micros();
  publishSample(rotated_accel, rotated_gyro, timestamp_us);
  updateSampleBuffer(rotated_accel, rotated_gyro, timestamp_us);
  init_ok_ = true;
  return true;
}

void BMI270::publishSample(const int16_t accel[3], const int16_t gyro[3], uint32_t timestamp_us) {
  sensorData_t data{};
  data.timestamp = timestamp_us;
  data.acc_raw.x = accel[0];
  data.acc_raw.y = accel[1];
  data.acc_raw.z = accel[2];
  data.gyro_raw.x = gyro[0];
  data.gyro_raw.y = gyro[1];
  data.gyro_raw.z = gyro[2];
  px4ImuMcnPublish(&data);
}

void BMI270::updateSampleBuffer(const int16_t accel[3], const int16_t gyro[3], uint32_t timestamp_us) {
  sensorData_t data{};
  data.timestamp = timestamp_us;
  data.acc_raw.x = accel[0];
  data.acc_raw.y = accel[1];
  data.acc_raw.z = accel[2];
  data.gyro_raw.x = gyro[0];
  data.gyro_raw.y = gyro[1];
  data.gyro_raw.z = gyro[2];

  rt_mutex_take(&sample_mutex_, RT_WAITING_FOREVER);
  memcpy(&latest_.sensor, &data, sizeof(sensorData_t));
  latest_.raw[0] = (uint8_t)(accel[0] & 0xFF);
  latest_.raw[1] = (uint8_t)((accel[0] >> 8) & 0xFF);
  latest_.raw[2] = (uint8_t)(accel[1] & 0xFF);
  latest_.raw[3] = (uint8_t)((accel[1] >> 8) & 0xFF);
  latest_.raw[4] = (uint8_t)(accel[2] & 0xFF);
  latest_.raw[5] = (uint8_t)((accel[2] >> 8) & 0xFF);
  latest_.raw[6] = (uint8_t)(gyro[0] & 0xFF);
  latest_.raw[7] = (uint8_t)((gyro[0] >> 8) & 0xFF);
  latest_.raw[8] = (uint8_t)(gyro[1] & 0xFF);
  latest_.raw[9] = (uint8_t)((gyro[1] >> 8) & 0xFF);
  latest_.raw[10] = (uint8_t)(gyro[2] & 0xFF);
  latest_.raw[11] = (uint8_t)((gyro[2] >> 8) & 0xFF);
  latest_.raw[12] = 0;
  latest_.raw[13] = 0;
  latest_.valid = true;
  rt_mutex_release(&sample_mutex_);

  rt_sem_release(&sample_sem_);
}

uint8_t BMI270::regRead(Register reg) {
  uint8_t value[2] = {0};
  if (spi_.readMultiReg16(static_cast<uint8_t>(reg), value, 1) != RT_EOK) {
    return 0;
  }
  return value[0];
}

bool BMI270::regWrite(Register reg, uint8_t value) {
  return spi_.writeMultiReg8(static_cast<uint8_t>(reg), &value, 1) == RT_EOK;
}

uint16_t BMI270::fifoLevel() {
  uint8_t buffer[2] = {0};
  if (spi_.readMultiReg16(static_cast<uint8_t>(Register::FIFO_LENGTH_0), buffer, 2) != RT_EOK) {
    return 0;
  }
  uint16_t length = ((buffer[1] & 0x3F) << 8) | buffer[0];
  return length;
}

void BMI270::fifoReset() {
  regWrite(Register::CMD, 0xB0);
  rt_thread_mdelay(2);
}

void BMI270::timerCallback(void *parameter) {
  auto *self = static_cast<BMI270 *>(parameter);
  if (!self || !self->event_inited_) {
    return;
  }
  rt_event_send(&self->event_, BMI270::EVENT_TIMER);
}

void BMI270::drdyIsr(void *parameter) {
  auto *self = static_cast<BMI270 *>(parameter);
  if (!self || !self->event_inited_) {
    return;
  }
  rt_event_send(&self->event_, BMI270::EVENT_DRDY);
}

int BMI270::readRaw(uint8_t *buffer, rt_size_t size) {
  if (!buffer || size < sizeof(latest_.raw)) {
    return -RT_EINVAL;
  }

  if (rt_sem_take(&sample_sem_, RT_WAITING_FOREVER) != RT_EOK) {
    return -RT_ERROR;
  }

  rt_mutex_take(&sample_mutex_, RT_WAITING_FOREVER);
  memcpy(buffer, latest_.raw, sizeof(latest_.raw));
  rt_mutex_release(&sample_mutex_);
  return sizeof(latest_.raw);
}

// ---- IMU 桥接 ----

static BMI270 &g_driver = BMI270::instance();

rt_err_t bmi270_px4_init_default() {
  RuntimeConfig cfg{};
  cfg.spi_bus_name = SENSOR_SPI_NAME_BMI270_PX4;
  cfg.spi_device_name = SENSOR_SPI_SLAVE_NAME_BMI270_PX4;
  cfg.cs_pin_name = SENSOR_BMI270_PX4_SPI_CS_PIN;
  cfg.int_pin_name = SENSOR_BMI270_PX4_INT_PIN;
  cfg.spi_max_hz = SENSOR_BMI270_PX4_SPI_MAX_HZ;
  int wm = SENSOR_BMI270_PX4_FIFO_WM;
  if (wm <= 0) {
    wm = kDefaultWatermarkSamples;
  }
  if (wm > 32) {
    wm = 32;
  }
  cfg.fifo_watermark_samples = static_cast<uint8_t>(wm);

  int mcn_ret = px4ImuMcnInit();
  if (mcn_ret != RT_EOK) {
    LOG_E("BMI270 mcn init failed: %d", mcn_ret);
    return (rt_err_t)mcn_ret;
  }

  if (cfg.spi_max_hz == 0) {
    cfg.spi_max_hz = 10000000;
  }

  rt_err_t ret = g_driver.init(cfg);
  if (ret != RT_EOK) {
    LOG_E("BMI270 init failed: %d", ret);
    return ret;
  }

  return RT_EOK;
}

}  // namespace bmi270_px4

extern "C" {

#ifdef BSP_USING_BMI270_PX4
static int bmi270_px4_init_default_wrapper(void) { return (int)bmi270_px4::bmi270_px4_init_default(); }
INIT_APP_EXPORT(bmi270_px4_init_default_wrapper);
#endif

}


