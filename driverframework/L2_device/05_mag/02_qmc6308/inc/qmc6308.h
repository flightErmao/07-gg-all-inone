#ifndef __QMC6308_H
#define __QMC6308_H

#include <stdint.h>
#include <rtdef.h>

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

/* QMC6308 LSB per Gauss conversion table
 * According to datasheet:
 *   Field Range = ±30G: 1000 LSB/G
 *   Field Range = ±12G: 2500 LSB/G
 *   Field Range = ±8G:  3750 LSB/G
 *   Field Range = ±2G:  15000 LSB/G
 */
typedef enum {
  QMC6308_RANGE_30G = 0,  /* ±30G range, 1000 LSB/G */
  QMC6308_RANGE_12G = 1,  /* ±12G range, 2500 LSB/G */
  QMC6308_RANGE_8G = 2,   /* ±8G range,  3750 LSB/G */
  QMC6308_RANGE_2G = 3    /* ±2G range,  15000 LSB/G */
} qmc6308_range_e;

/* LSB per Gauss lookup table */
#define QMC6308_LSB_PER_G_30G  1000   /* LSB per Gauss for ±30G range */
#define QMC6308_LSB_PER_G_12G  2500   /* LSB per Gauss for ±12G range */
#define QMC6308_LSB_PER_G_8G   3750   /* LSB per Gauss for ±8G range */
#define QMC6308_LSB_PER_G_2G   15000  /* LSB per Gauss for ±2G range */

/* LSB to uT conversion (1 Gauss = 100 uT) */
#define QMC6308_LSB_TO_UT_30G  (100.0f / QMC6308_LSB_PER_G_30G)  /* 0.1 uT/LSB */
#define QMC6308_LSB_TO_UT_12G  (100.0f / QMC6308_LSB_PER_G_12G)  /* 0.04 uT/LSB */
#define QMC6308_LSB_TO_UT_8G   (100.0f / QMC6308_LSB_PER_G_8G)   /* 0.02667 uT/LSB */
#define QMC6308_LSB_TO_UT_2G   (100.0f / QMC6308_LSB_PER_G_2G)   /* 0.00667 uT/LSB */

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

/* QMC6308 configuration structure exported from driver */
typedef struct {
  rt_uint16_t range_g;      /* magnetometer range in Gauss (2, 8, 12, 30) */
  rt_uint16_t odr_hz;        /* output data rate in Hz (10, 50, 100, 200) */
  float lsb;                 /* least significant bit scale factor (uT/LSB) */
} qmc6308_config_t;

#endif
