#ifndef __QMC6308_H
#define __QMC6308_H

#include <stdint.h>

/*ID registers */
#define QMC6308_CHIP_ID_REG 0x00
/*Status registers */
#define QMC6308_STATUS_REG 0x09
/* configuration registers */
#define QMC6308_CTL_REG_ONE 0x0A   /* Contrl register one */
#define QMC6308_CTL_REG_TWO 0x0B   /* Contrl register two */
#define QMC6308_CTL_REG_THREE 0x0D /* Contrl register three */
/*data output register*/
#define QMC6308_DATA_OUT_X_LSB_REG 0x01
#define QMC6308_DATA_OUT_X_MSB_REG 0x02
#define QMC6308_DATA_OUT_Y_LSB_REG 0x03
#define QMC6308_DATA_OUT_Y_MSB_REG 0x04
#define QMC6308_DATA_OUT_Z_LSB_REG 0x05
#define QMC6308_DATA_OUT_Z_MSB_REG 0x06

/* Magnetic Sensor Operating Mode MODE[1:0]*/
#define QMC6308_SUSPEND_MODE 0x00
#define QMC6308_NORMAL_MODE 0x01
#define QMC6308_SINGLE_MODE 0x02
#define QMC6308_CONTINUOUS_MODE 0x03

/* Magnetic Sensor OUTPUT Date Rate[3:2]*/
#define QMC6308_SET_OUTPUT_DATA_RATE_10 0x00
#define QMC6308_SET_OUTPUT_DATA_RATE_50 0x04
#define QMC6308_SET_OUTPUT_DATA_RATE_100 0x08
#define QMC6308_SET_OUTPUT_DATA_RATE_200 0x0C

/* Magnetic Sensor over sample rate1[5:4]*/
#define QMC6308_SET_OVERSAMPLE_RATIO_8 0x00
#define QMC6308_SET_OVERSAMPLE_RATIO_4 0x10
#define QMC6308_SET_OVERSAMPLE_RATIO_2 0x20
#define QMC6308_SET_OVERSAMPLE_RATIO_1 0x30

/* Magnetic Sensor over sample rate2[7:6]*/
#define QMC6308_SET_DOWNSAMPLE_RATIO_1 0x00
#define QMC6308_SET_DOWNSAMPLE_RATIO_2 0x40
#define QMC6308_SET_DOWNSAMPLE_RATIO_4 0x80
#define QMC6308_SET_DOWNSAMPLE_RATIO_8 0xC0

/* Magnetic Sensor set/reset period[1:0]*/
#define QMC6308_SET_RESET_RESET_ON 0x00
#define QMC6308_SET_SET_ONLY_ON 0x01
#define QMC6308_SET_SET_RESET_OFF 0x02
#define QMC6308_SET_SET_RESET_OFF2 0x03

/*Magnetic Sensor range[3:2]*/
#define QMC6308_SET_RANGE_30G 0x00
#define QMC6308_SET_RANGE_12G 0x04
#define QMC6308_SET_RANGE_8G 0x08
#define QMC6308_SET_RANGE_2G 0x0C

#define QMC6308_SET_SRCTRL_ON 0x40
#define QMC6308_SET_SRCTRL_OFF 0x00

#define QMC6308_DEFAULT_DELAY 200
#define QMC6308_MODE_SWITCH

#define QMC6308_SELFTEST_MAX_X (1800)
#define QMC6308_SELFTEST_MIN_X (120)
#define QMC6308_SELFTEST_MAX_Y (1800)
#define QMC6308_SELFTEST_MIN_Y (120)
#define QMC6308_SELFTEST_MAX_Z (1800)
#define QMC6308_SELFTEST_MIN_Z (120)

#define QMC6308_ID_VALUE 0x80

enum {
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2,

  AXIS_TOTAL
};

typedef struct {
  signed char sign[3];
  unsigned char map[3];
} qmc6308_map;

typedef uint8_t (*mqc6308_i2c_read_block)(uint8_t, uint8_t*, uint8_t);
typedef uint8_t (*mqc6308_i2c_write_reg)(uint8_t, uint8_t);

extern mqc6308_i2c_read_block qmc6308_read_block;
extern mqc6308_i2c_write_reg qmc6308_write_reg;

int qmc6308_init(void);
int qmc6308_read_mag_xyz(float* data);

int qmc6308_init_zerozero(void);
int qmc6308_read_mag_xyz_zerozero(int16_t* data);

#endif
