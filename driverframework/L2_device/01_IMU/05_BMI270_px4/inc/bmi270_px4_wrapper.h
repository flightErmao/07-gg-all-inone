#ifndef BMI270_PX4_WRAPPER_H_
#define BMI270_PX4_WRAPPER_H_

#include <rtdef.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const char *spi_bus_name;
  const char *spi_device_name;
  const char *cs_pin_name;
  const char *int_pin_name;
  uint32_t spi_max_hz;
  const char *imu_device_name;
  uint8_t fifo_watermark_samples;
} px4_bmi270_config_t;

rt_err_t px4_bmi270_init(const px4_bmi270_config_t *cfg);
bool px4_bmi270_is_ready(void);
int px4_bmi270_read_raw(uint8_t *buffer, rt_size_t size);

#ifdef __cplusplus
}
#endif

#endif  // BMI270_PX4_WRAPPER_H_


