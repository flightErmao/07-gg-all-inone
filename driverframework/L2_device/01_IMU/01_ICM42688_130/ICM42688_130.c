#include "ICM42688_130.h"
#include <rtdevice.h>
#include "stdio.h"
#include "imu.h"
#include <string.h>
#include "cmsis_compiler.h"

#define PACKSIZE_SIZE  16
#define MAX_PACKET_SIZE 32

#define PRTINF_DEBUG 0

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
#define ICM4X6XX_ODR_4000 4000.0f
#define ICM4X6XX_ODR_8000 8000.0f

#define INTVL_TOLERANCE_FACTOR (1.02f)
#define LPF_PARAMETER_N 25
#define MAX_SCP_SAMPLES 20


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

#define FIFO_HEADER_EMPTY_BIT (0x80)
#define FIFO_HEADER_A_BIT (0x40)
#define FIFO_HEADER_G_BIT (0x20)
#define FIFO_HEADER_20_BIT (0x10)
#define WHO_AM_I_ID 0x47


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


typedef enum icm4x6xx_ui_intf {
  AUTO_INTF,
  I2C_INTF = 2,
  SPI_INTF = 3,
} icm4x6xx_ui_intf;

enum {
  PM2G = 2,
  PM4G = 4,
  PM8G = 8,
  PM16G = 16
};

enum {
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

typedef struct{
  uint32_t id;
  uint8_t index;
  int16_t raw_accel[3];
  int16_t raw_gyro[3];
  int16_t raw_temp;
  uint64_t timestamp_us;
  float accel[3];
  float gyro[3];
  float temperature;
  float accel_calib[3];
  float gyro_calib[3];
}IMUData;

float ACCEL_SCALER =(float)(PM16G / (1 << 15) * 9.80665);
float GYRO_SCALER = (float)(PM2000DPS /(1 << 15) * (3.141592653f) / 180.0f);
float TEMPERATURE_SCALER = 132.48f;
float TEMPERATURE_OFFSET = 25.0f;
int id_ = 0;
uint64_t err_cnt_ = 0;
int notify_id_;

IMURawData internal_raw_data_;
IMUData internal_data_;
uint8_t imu_fifo_data[PACKSIZE_SIZE*MAX_PACKET_SIZE] = {0};

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

static struct rt_spi_device* imu_spi_dev;
static struct rt_spi_configuration spi_config;

int icm42688_130_init(bool clkin_enable);
bool icm42688_130_ReadRaw(IMURawData* data);

bool WriteByte(uint8_t reg, uint8_t val);
bool WriteMask(uint8_t reg_addr, uint8_t reg_value, uint8_t mask);
bool ReadBlock(uint8_t first_reg, uint8_t buf[], int len);

//bool ReadWhoAmI(uint8_t *who_am_i);
int ReadAccel(int16_t accel_raw_data[3]);
int ReadGyro(int16_t gyro_raw_data[3]);
int ReadTemp(int16_t* temp_raw_data);

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
bool icm4x6xx_set_accel_odr(float odr);
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
uint32_t icm4x6xx_cal_valid_fifo_len(const uint8_t *buf, uint32_t buf_len,uint16_t *cnt);
uint32_t get_offset(uint16_t early, uint16_t later);

rt_err_t SPI_ReadOneRegister(uint8_t reg, uint8_t* data) {
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};
    tx_data[0] = reg | 0x80;
    rt_err_t ret = rt_spi_transfer(imu_spi_dev, tx_data, rx_data, 2);
    *data = rx_data[1];
    if (ret != 0x02) {
        return -1;
    } else {
        return 0;
    }
}

rt_err_t SPI_WriteOneRegister(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};
    tx_data[0] = reg;
    tx_data[1] = data;
    rt_err_t ret = rt_spi_transfer(imu_spi_dev, tx_data, rx_data, 2);
    if (ret != 0x02) {
        return -1;
    } else {
        return 0;
    }
}

static uint8_t spi_rx_buffer[2049] = {0};
static uint8_t spi_tx_buffer[2049] = {0};

rt_err_t SPI_ReadMultRegister(uint8_t reg, uint8_t* data, uint16_t len) {
    memset(spi_tx_buffer, 0, len);
    spi_tx_buffer[0] = reg | 0x80;
    rt_err_t ret = rt_spi_transfer(imu_spi_dev, spi_tx_buffer, spi_rx_buffer, len + 1);
    if (ret != len + 1) {
        return -1;
    } else {
        memcpy(data, spi_rx_buffer + 1, len);
        return 0;
    }
}

void ICM42688_param_init()
{
  use_hi_res_ = false;
  fifo_info_record_mode_ = false;
  gyro_fsr_ = GYRO_RANGE_2000DPS;
  accel_fsr_ = ACC_RANGE_16G;
  accel_power_mode_ = ICM4X6XX_A_LNM;
  fifo_mode_ = STREAM;
  desire_format_ = ICM4X6XX_FORMAT_16_BYTES;
  en_a_fifo_ = true;
  en_g_fifo_ = true;
  last_odr_timestamp_ = 0;
  sync_time_count_ = 0;
  odr_time_offset_ = 2000;
  // odr_time_offset_before = 2000;
  last_fifo_timestamp_16b_ = 0;

  internal_raw_data_.PACKET_SIZE = PACKSIZE_SIZE;
  internal_raw_data_.MAX_PACKET_COUNT = MAX_PACKET_SIZE;
  internal_raw_data_.fifo_data = imu_fifo_data ;
  internal_raw_data_.fifo_count = 0 ;
  internal_raw_data_.packet_count = 0 ;

}

int imu_check_who_am_i() {
    uint8_t id = 0;
    uint8_t i = 0;
    for (i = 0; i < 10; i++) {
        SPI_ReadOneRegister(REG_WHO_AM_I, &id);
        if (id == WHO_AM_I_ID) {
            break;
        }
        rt_thread_mdelay(20);
    }

    if (id != WHO_AM_I_ID) {
        return -2;
    }
    return 0;
}

int icm42688_130_init(bool clkin_enable) 
{
  if(imu_check_who_am_i())
  {
    printf("icm42688 id read fail\r\n");
    return -1;
  }
  ICM42688_param_init();

  //bool clkin_enable_;
  clkin_enable_ = clkin_enable;

  if (!icm4x6xx_config_ui_intf(SPI_INTF)) {
    printf("[%d]icm4x6xx_config_ui_intf() faild", id_);
    return -3;
  }

  if (!icm4x6xx_enable_rtc_mode(clkin_enable)) {
    printf("[%d]icm4x6xx_enable_rtc_mode(): faild", id_);
    return -4;
  }

#if 0
    /* Enable Timestamp field contains the measurement of time
     * since the last occurrence of ODR.*/
   if (!icm4x6xx_enable_delta_tmst(true))
   {
    printf("[%d]icm4x6xx_enable_delta_tmst(): faild", id_);
     return false;
   }
#else

  if (clkin_enable) {
    /* Enable timestamp register */
    if (!icm4x6xx_enable_tmst(true)) {
      printf("[%d]icm4x6xx_enable_tmst(): faild", id_);
      return -5;
    }
  }
#endif

  /*if (!icm4x6xx_en_int_push_pull(-1)) {
    return false;
  }

  if (!icm4x6xx_en_int_latched_mode(-1)) {
    return false;
  }

  if (!icm4x6xx_config_int_polarity(ICM4X6XX_INT_ACTIVE_HIGH)) {
    return false;
  }*/

  /* Choose big endian mode for fifo count and sensor data
   * default mode for FPGA and chip may be different,
   * so we choose one mode here for both */
  if (!icm4x6xx_en_big_endian_mode(true)) {
    printf("[%d]icm4x6xx_en_big_endian_mode(): faild", id_);
    return -6;
  }

  /* enable fifo hold last data */
  /*if (!icm4x6xx_en_fifo_hold_last_data(-1)) {
    return false;
  }*/

  /* do not tag fsync flag for temperature resolution */
  if (!icm4x6xx_config_fsync(0)) {
    printf("[%d]icm4x6xx_config_fsync():faild", id_);
    return -7;
  }

  /* Choose Accel FSR */
  if (!icm4x6xx_set_accel_fsr(accel_fsr_)) {
    printf("[%d]icm4x6xx_set_accel_fsr() faild", id_);
    return -8;
  }

  /* Choose Gyro FSR */
  if (!icm4x6xx_set_gyro_fsr(gyro_fsr_)) {
    printf("[%d]icm4x6xx_set_gyro_fsr() faild", id_);
    return -9;
  }

  /* Choose Accel filter order */
  if (!icm4x6xx_set_accel_filter_order(THIRD_ORDER))  // USE 3rd order filter
  {
    printf("[%d]icm4x6xx_set_accel_filter_order() faild", id_);
    return -1;
  }

  /* Choose Gyro filter order */
  if (!icm4x6xx_set_gyro_filter_order(THIRD_ORDER))  // USE 3rd order filter
  {
    printf("[%d]icm4x6xx_set_gyro_filter_order() faild", id_);
    return -1;
  }

  /* Enable fifo */
  if (!icm4x6xx_set_fifo_mode(fifo_mode_)) {
    printf("[%d]icm4x6xx_set_fifo_mode() faild", id_);
    return -1;
  }
#if 0
  /* Enable fifo overflow interrupt */
  if (!icm4x6xx_en_fifo_full_int(-1)) {
    return false;
  }

  /* Enable high shock interrupt */
  if (!icm4x6xx_en_high_shock_int(-1)) {
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

  // if (clkin_enable) {
  //   icm4x6xx_enable_nflt_gyro(true);
  // }

  if (!icm4x6xx_disable_aux_pins()) {
    printf("[%d]icm4x6xx_disable_aux_pins() faild", id_);
    return -1;
  }

  if (clkin_enable) {
    if (!icm4x6xx_set_accel_odr(ICM4X6XX_ODR_8000)) {
      printf("[%d]icm4x6xx_set_accel_odr() faild", id_);
      return -1;
    }

    if (!icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_8000)) {
      printf("[%d]icm4x6xx_set_gyro_odr() faild", id_);
      return -1;
    }
  } else {
    if (!icm4x6xx_set_accel_odr(ICM4X6XX_ODR_4000)) {
      printf("[%d]icm4x6xx_set_accel_odr() faild", id_);
      return -1;
    }

    if (!icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_4000)) {
      printf("[%d]icm4x6xx_set_gyro_odr() faild", id_);
      return -1;
    }
  }

  if (!icm4x6xx_set_accel_bandwidth(BW_ODR_DIV_5)) {
    printf("[%d]icm4x6xx_set_accel_bandwidth() faild", id_);
    return -1;
  }

  if (clkin_enable) {
    if (!icm4x6xx_set_gyro_bandwidth(BW_ODR_DIV_2)) {
      printf("[%d]icm4x6xx_set_gyro_bandwidth() faild", id_);
      return -1;
    }
  } else {
    if (!icm4x6xx_set_gyro_bandwidth(BW_ODR_DIV_2)) {
      printf("[%d]icm4x6xx_set_gyro_bandwidth() faild", id_);
      return -1;
    }
  }

  if (!icm4x6xx_en_gyro(true)) {
    printf("[%d]icm4x6xx_en_gyro() faild", id_);
    return -1;
  }

  if (!icm4x6xx_set_accel_mode(accel_power_mode_)) {
    printf("[%d]icm4x6xx_set_accel_mode() faild", id_);
    return -1;
  }

  if (!icm4x6xx_en_fifo(en_a_fifo_, en_g_fifo_)) {
    printf("[%d]icm4x6xx_en_fifo() faild", id_);
    return -20;
  }

  rt_thread_mdelay(20);

  // interrput_pin_.SetIRQDelegate(this);

  printf("Start ok\r\n");
  return 0;
}

bool icm4x6xx_enable_rtc_mode(bool enable) {
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
 *                      -1: disable data ready interrupt
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_en_dri(bool enable) {
#ifdef ICM4X6XX_USE_INT2
  rc += icm4x6xx_write_mask(instance, REG_INT_SOURCE3,
                            enable ? DRI_INT2_EN_MASK : 0, &xfer_bytes, -1,
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
 *                      -1: disable gyro
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_en_gyro(bool enable) {
  return WriteMask(REG_PWR_MGMT_0, enable ? GYRO_LNM_MASK : 0, GYRO_LNM_MASK);
}

/**
 * @brief Enable or Disable Accel.
 *
 * @param[in] enable    Identify enable accel or not.
 *                      true: enable accel
 *                      -1: disable accel
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_set_accel_mode(icm4x6xx_power_mode mode) {
  return WriteMask(REG_PWR_MGMT_0, mode, ACCEL_LNM_MASK);
}

bool icm4x6xx_disable_aux_pins() {
  uint8_t reg_value = 0;

  if (!icm4x6xx_set_reg_bank(2)) {
    return false;
  }

  reg_value = 0x01;

  if (SPI_WriteOneRegister((0x70), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (SPI_WriteOneRegister((0x71), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (SPI_WriteOneRegister((0x72), reg_value)) {
    return false;
  }
  reg_value = 0x01;

  if (SPI_WriteOneRegister((0x73), reg_value)) {
    return false;
  }
  if (!icm4x6xx_set_reg_bank(0)) {
    return false;
  }

  return true;
}

bool icm4x6xx_enable_gyro_periodic_reset() {
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
bool icm4x6xx_enable_int_async_reset(bool enable) {
  return WriteMask(REG_INT_CONFIG1, enable ? BIT_INT_ASY_RST_DIS_MASK : 0,
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
bool icm4x6xx_enable_record_mode(bool enable) {
  return WriteMask(REG_INTF_CONFIG0, enable ? RECORD_MODE_MASK : 0,
                   RECORD_MODE_MASK);
}

bool icm4x6xx_config_highg_parameter() {
  uint8_t reg_value = 0;

  if (!icm4x6xx_set_reg_bank(4)) {
    return false;
  }

  reg_value = ICM4X6XX_APEX_CONFIG6_HIGHG_PEAK_TH_2844MG |
              ICM4X6XX_APEX_CONFIG6_HIGHG_TIME_TH_20MS;

  if (SPI_WriteOneRegister((REG_APEX_CONFIG6),
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
bool icm4x6xx_set_reg_bank(uint8_t bank_num) {
  if (SPI_WriteOneRegister((REG_BANK_SEL), bank_num)) {
    return false;
  }
  return true;
}

bool icm4x6xx_en_high_shock_int(bool enable) {
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
 *                        -1: disable accel fifo
 * @param[in] en_gyro     Identify enable gyro fifo or not.
 *                        true: enable gyro fifo, write gyro data into fifo
 *                        -1: disable gyro fifo
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool icm4x6xx_en_fifo(bool en_accel, bool en_gyro) {
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
 *                      -1: disable fifo overflow interrupt
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_en_fifo_full_int(bool enable) {
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
bool icm4x6xx_set_fifo_mode(icm4x6xx_fifo_mode fifo_mode) {
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
bool icm4x6xx_set_gyro_bandwidth(icm4x6xx_bandwidth bw) {
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
bool icm4x6xx_set_accel_bandwidth(icm4x6xx_bandwidth bw) {
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
bool icm4x6xx_set_gyro_filter_order(icm4x6xx_filter_order order) {
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
bool icm4x6xx_set_accel_filter_order(icm4x6xx_filter_order order) {
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
bool icm4x6xx_set_gyro_fsr(icm4x6xx_gyro_fsr fsr) {
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
bool icm4x6xx_set_accel_fsr(icm4x6xx_accel_fsr fsr) {
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
bool icm4x6xx_config_fsync(uint8_t data) {
  return WriteMask(REG_FSYNC_CONFIG, data, 0x70);
}
//////////////////////////////////////////////////////
bool icm4x6xx_en_fifo_hold_last_data(bool enable) {
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
bool icm4x6xx_config_ui_intf(icm4x6xx_ui_intf intf) {
  return WriteMask(REG_INTF_CONFIG0, intf, UI_INTF_MASK);
}

bool icm4x6xx_en_int_push_pull(bool enable) {
  uint8_t push_pull_mask = INT1_PUSH_PULL_MASK;
  return WriteMask(REG_INT_CONFIG, enable ? push_pull_mask : 0,
                   INT1_PUSH_PULL_MASK);
}

bool icm4x6xx_en_int_latched_mode(bool enable) {
  uint8_t latch_mode_mask = INT1_LATCHED_MODE_MASK;

  return WriteMask(REG_INT_CONFIG, enable ? latch_mode_mask : 0,
                   latch_mode_mask);
}

bool icm4x6xx_enable_nflt_gyro(bool enable) {
  uint8_t tmst_config_reg = 0x0B;
    if (!icm4x6xx_set_reg_bank(1)) {
    return false;
  }
  return WriteMask(tmst_config_reg,
                   0x03,
                   0x03);
}

bool icm4x6xx_enable_nflt_acc(bool enable) {
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
 *                   -1: enable int active low
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_config_int_polarity(icm4x6xx_int_polarity polarity) {
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
 * mode -1: disable fifo count and sensor data big endian mode
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_en_big_endian_mode(bool enable) {
  uint8_t bit_mask = FIFO_COUNT_BIG_ENDIAN_MASK | SENSOR_DATA_BIG_ENDIAN_MASK;
  uint8_t reg = enable ? bit_mask : 0;

  return WriteMask(REG_INTF_CONFIG0, reg, bit_mask);
}

bool WriteMask(uint8_t reg_addr, uint8_t reg_value,uint8_t mask) {
  uint8_t rw_buffer = 0;

  if (SPI_ReadOneRegister(reg_addr, &rw_buffer)) {
    return false;
  }

  /* generate new value */
  rw_buffer = (rw_buffer & (~mask)) | (reg_value & mask);

  /* write new value to this register */
  if (SPI_WriteOneRegister(reg_addr, rw_buffer)) {
    return false;
  }

  // printf("ok\n");
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
bool icm4x6xx_enable_delta_tmst(bool enable) {
  uint8_t tmst_config_reg = REG_TMST_CONFIG_REG;
  return WriteMask(tmst_config_reg,
                   enable ? (BIT_TMST_DELTA_EN | BIT_TMST_TO_REGS_EN) : 0,
                   BIT_TMST_DELTA_EN | BIT_TMST_TO_REGS_EN);
}

/**
 * @brief enable timestamp register
 *
 * @param[in] enable   enable or disable timestamp register
 *
 * @return 0 if success
 *         non-zero value if failed.
 */
bool icm4x6xx_enable_tmst(bool enable) {
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
bool icm4x6xx_enable_tmst_val(bool enable) {
  return WriteMask(REG_SIGNAL_PATH_RESET_REG, enable ? BIT_TMST_STROBE : 0,
                   BIT_TMST_STROBE);
}

bool icm4x6xx_read_tmst_val(uint32_t* tmst_reg) {
  uint8_t buff[3] = {0};

  if (!icm4x6xx_set_reg_bank(1)) {
    return false;
  }

  if (ReadBlock(REG_TMSTVAL0, buff, sizeof(buff))) {
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
bool icm4x6xx_get_packet_size(uint8_t* size) {
  uint8_t packet_size = 0;
  bool result = true;

  // printf("desire_format_ = %d\n", desire_format_ );  //lmy_asi

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
bool icm4x6xx_read_fifo_count(uint16_t* count) {
  uint8_t buff[2];
  uint16_t max_count = 0;

  if (SPI_ReadMultRegister(REG_FIFO_BYTE_COUNT_L, buff, 2)) {
    return false;
  }

  *count = (uint16_t)(buff[0] << 8 | buff[1]);

  /* According DS 6.3 MAXIMUM FIFO STORAGE
   * the largest size FIFO size is 2080 bytes*/
  max_count = ICM4X6XX_YOKOHAMA_MAX_FIFO_SIZE;

  if (*count > max_count) {
    // ICM4X6XX_INST_PRINTF(ERROR, instance,
    //    "FF c %d\r\n", *count);
    *count = max_count;
  }

  return true;
}

bool icm4x6xx_is_fifo_format_match(icm4x6xx_fifo_format format) {
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

uint32_t icm4x6xx_cal_valid_fifo_len(const uint8_t* buf,
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

uint32_t get_offset(uint16_t early, uint16_t later) {
  uint32_t offset = 0;

  if (later < early) {
    offset = 0xffff - early + 1 + later;
  } else {
    offset = later - early;
  }

  return offset;
}

// __STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
// {
//   return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
// }

// static uint32_t getCurrentMicros(void)
// {
//   /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
//   GXT_SYSTICK_IsActiveCounterFlag();
//   uint32_t m = HAL_GetTick();
//   const uint32_t tms = SysTick->LOAD + 1;
//   __IO uint32_t u = tms - SysTick->VAL;
//   if (GXT_SYSTICK_IsActiveCounterFlag()) {
//     m = HAL_GetTick();
//     u = tms - SysTick->VAL;
//   }
//   return (m * 1000 + (u * 1000) / tms);
// }
//获取系统时间，单位us
uint32_t micros(void)
{
  uint32_t fark_time = 0;
  return fark_time;
}


bool icm42688_130_ReadRaw(IMURawData* data) {
  data->is_need_cali_time = false;

  int16_t accel[3] = {0};
  int16_t gyro[3] = {0};

  // uint64_t dsp_time = Timer::Now();
#if PRTINF_DEBUG 
    printf("\r\n");
#endif

  if (clkin_enable_) {
    bool need_sync_time = (sync_time_count_ >= 100) ? false : true;

    if (need_sync_time) {
      sync_time_count_++;
      if (!icm4x6xx_enable_tmst_val(true)) {
        printf("id[%d]: icm4x6xx_enable_tmst_val fail\n", id_);
        return false;
      }
    }

    data->id = 0x47;  // ffc need it //id_;
    data->index = id_; // 0 or 1 ,meas spi port numble in 130driverframework
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
      printf("id[%d]: icm4x6xx_get_packet_size fail\n", id_);
      return false;
    }
    printf("icm4x6xx_get_packet_size = %d\r\n", packet_size);
    if (!icm4x6xx_read_fifo_count(&fifo_count)) {
      printf("id[%d]: icm4x6xx_read_fifo_count fail\n", id_);
      return false;
    }
    printf("fifo_count = %d\r\n", fifo_count);

    if (fifo_count == 0) {
      printf("id[%d]: fifo_count = 0\n", id_);
      return false;
    }

    bytes_to_read = fifo_count;

    if (bytes_to_read > (packet_size * MAX_SCP_SAMPLES)) {
      bytes_to_read = (packet_size * MAX_SCP_SAMPLES);
    }

    uint8_t buf[bytes_to_read];
    if (!icm4x6xx_read_fifo_buf(buf, bytes_to_read)) {
      printf("id[%d]: icm4x6xx_read_fifo_buf(buf, bytes_to_read) fail\n", id_);
      return false;
    }

    uint16_t packet_cnt = 0;
    uint32_t valid_buf_len = icm4x6xx_cal_valid_fifo_len(buf, bytes_to_read, &packet_cnt);
    printf("valid_buf_len is %d\r\n",valid_buf_len);
    // printf("(valid_buf_len == %d || packet_cnt == %d) ok\n",
    // valid_buf_len,
    //          packet_cnt);

    // ICM4X6XX_INST_PRINTF(LOW, instance, "valid buf_len/packet_cnt %d %d\r\n",
    // valid_buf_len, packet_cnt);
    if (valid_buf_len == 0 || packet_cnt == 0) {
      // ICM4X6XX_INST_PRINTF(HIGH, instance, "no valid senor data");
      // goto CLEAN_STATUS;

      printf("id[%d]: (valid_buf_len == 0 || packet_cnt == 0) fail, fifo_count = %d\r\n", id_, fifo_count);
      return false;
    }

    if (packet_cnt != valid_buf_len / packet_size) {
      printf("id[%d]: (packet_cnt = valid_buf_len / packet_size) fail\r\n", id_);
      printf("valid_buf_len %d \r\n",valid_buf_len);
      printf("packet_size %d \r\n",packet_size);
      printf("packet_cnt %d \r\n",packet_cnt);
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
      // printf("fifo_timestamp = %d\n", fifo_timestamp);
    }*/

    if (need_sync_time) {
      if (!icm4x6xx_read_tmst_val(&intl_cnt_20b)) {
        printf("id[%d]: icm4x6xx_read_tmst_val(&intl_cnt_20b) fail\n", id_);
        return false;
      }

      // printf("Timer::Now() = %lld\n", Timer::Now());
      // printf("intl_cnt_20b = %d\n", intl_cnt_20b);

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

      data->is_need_cali_time  = true;
      data->chip_cali_fifo_timestamp = intl_cnt_20b & 0xffff;

    } else {
      /*uint32_t fifo_timestamp_offset =
          get_offset(last_fifo_timestamp_16b_, fifo_timestamp);

      dsp_fifo_timestamp_ += (double)fifo_timestamp_offset * 1.024;

      last_fifo_timestamp_16b_ = fifo_timestamp;*/
    }

    // TO Do ：need have time
    //data->timestamp_us = dsp_time;   //(uint64_t)dsp_fifo_timestamp_;

    // printf("%d,%lld,%lld,%lld,%lld", odr_time_offset_,
    //            data->timestamp_us - last_odr_timestamp_, data->timestamp_us,
    //            before, (int64_t)before - (int64_t)data->timestamp_us);
    // printf("====================data->timestamp_us =
    // %lld\n", data->timestamp_us);

    data->temperature = (float)temperture / 2.07 + TEMPERATURE_OFFSET;

    data->accel[0] = accel[0] * ACCEL_SCALER;
    data->accel[1] = accel[1] * ACCEL_SCALER;
    data->accel[2] = accel[2] * ACCEL_SCALER;

    data->raw_accel[0] = accel[0];
    data->raw_accel[1] = accel[1];
    data->raw_accel[2] = accel[2];

    data->gyro[0] = gyro[0] * GYRO_SCALER;
    data->gyro[1] = gyro[1] * GYRO_SCALER;
    data->gyro[2] = gyro[2] * GYRO_SCALER;

    data->raw_gyro[0] = gyro[0];
    data->raw_gyro[1] = gyro[1];
    data->raw_gyro[2] = gyro[2];
    // last_odr_timestamp_ = data->timestamp_us;

    if (packet_cnt > data->MAX_PACKET_COUNT) {
      packet_cnt = data->MAX_PACKET_COUNT;
    }

    data->fifo_count = packet_size * packet_cnt;
    data->packet_count = packet_cnt;

    // printf("packet_cnt = %d, packet_size = %d , data->fifo_count = %d\n",
    // packet_cnt, packet_size, data->fifo_count);  //lmy_asi

    memcpy(data->fifo_data, p, data->fifo_count);

    return true;

  } else {
    data->id = 0x47;  // ffc need it //id_;
    data->index = id_;

    uint16_t fifo_count = 0;
    uint16_t bytes_to_read = 0;
    uint8_t packet_size = 0;
    // int16_t accel[3] = {0};
    int16_t temperture = 0;
    // int16_t gyro[3] = {0};

    if (!icm4x6xx_get_packet_size(&packet_size)) {
      printf("id[%d]: icm4x6xx_get_packet_size fail\n", id_);
      return false;
    }

    // printf("icm4x6xx_get_packet_size = %d\n", packet_size);
    if (!icm4x6xx_read_fifo_count(&fifo_count)) {
      printf("id[%d]: icm4x6xx_read_fifo_count fail\n", id_);
      return false;
    }
#if PRTINF_DEBUG 
    printf("imu have fifo_count = %d\r\n", fifo_count);
#endif

    if (fifo_count == 0) {
      printf("id[%d]: fifo_count == 0\r\n", id_);
      return false;
    }

    bytes_to_read = fifo_count;

    if (bytes_to_read > (packet_size * MAX_SCP_SAMPLES)) {
      bytes_to_read = (packet_size * MAX_SCP_SAMPLES);
    }
#define MAX_FIFO_BUF_SIZE (16 * MAX_SCP_SAMPLES)
    uint8_t buf[MAX_FIFO_BUF_SIZE] = {0};

    if (!icm4x6xx_read_fifo_buf(buf, bytes_to_read)) {
      printf("id[%d]: icm4x6xx_read_fifo_buf(buf, bytes_to_read) fail, fifo_count = %d\r\n", id_, fifo_count);
      return false;
    }

    uint16_t packet_cnt = 0;
    uint32_t valid_buf_len =
        icm4x6xx_cal_valid_fifo_len(buf, bytes_to_read, &packet_cnt);

    // printf("(valid_buf_len == %d || packet_cnt == %d) ok\n",
    // valid_buf_len,
    //          packet_cnt);

    // ICM4X6XX_INST_PRINTF(LOW, instance, "valid buf_len/packet_cnt %d %d\r\n",
    // valid_buf_len, packet_cnt);
    if (valid_buf_len == 0 || packet_cnt == 0) {
      // ICM4X6XX_INST_PRINTF(HIGH, instance, "no valid senor data");
      // goto CLEAN_STATUS;
#if PRTINF_DEBUG 
      printf("id[%d]: (valid_buf_len == 0 || packet_cnt == 0) fail, fifo_count = %d\r\n", id_, fifo_count);
#endif
      return false;
    }

    if (packet_cnt != valid_buf_len / packet_size) {
#if PRTINF_DEBUG 
      printf("id[%d]: (packet_cnt = valid_buf_len / packet_size) fail\r\n", id_);
      printf("init packet_size is %d\r\n",packet_size);
      printf("valid_buf_len is %d\r\n",valid_buf_len);
      printf("should packet_cnt is %d\r\n",bytes_to_read/packet_size);
      printf("infact packet_cnt is %d\r\n",packet_cnt);
#endif
      return false;
    }
    else
    {
#if PRTINF_DEBUG 
      printf("init packet_size is %d\r\n",packet_size);
      printf("valid packet_cnt is %d\r\n",packet_cnt);
      printf("get fifo success\r\n");        
#endif
    }

    uint16_t index = packet_cnt - 1;
    uint8_t* p = buf;

    temperture = p[index * packet_size + 0x0D];

    //index = 0;
    accel[0] = p[index * packet_size + 0x01] * 256 + p[index * packet_size + 0x02];
    accel[1] = p[index * packet_size + 0x03] * 256 + p[index * packet_size + 0x04];
    accel[2] = p[index * packet_size + 0x05] * 256 + p[index * packet_size + 0x06];
    gyro[0] = p[index * packet_size + 0x07] * 256 + p[index * packet_size + 0x08];
    gyro[1] = p[index * packet_size + 0x09] * 256 + p[index * packet_size + 0x0A];
    gyro[2] = p[index * packet_size + 0x0B] * 256 + p[index * packet_size + 0x0C];
    /*  temperture = p[index * packet_size + 0x0D];

      // printf("fifo_timestamp = %d\n", fifo_timestamp);
    }*/

    // uint16_t fifo_timestamp = 0;
    // static uint16_t fifo_timestamp_latest = 0;
    // fifo_timestamp = p[index * packet_size + 0x0E] * 256 + p[index * packet_size + 0x0F];
    // //printf("%d\r\n",fifo_timestamp);
    // if(fifo_timestamp > fifo_timestamp_latest)
    // {
    //   printf("%d %d %d\r\n",fifo_timestamp-fifo_timestamp_latest,packet_cnt,micros());
    // }
    // else
    // {
    //   printf("%d %d %d\r\n",65535-fifo_timestamp_latest+fifo_timestamp,packet_cnt,micros());
    // }

    // fifo_timestamp_latest = fifo_timestamp;


    // uint64_t before = dsp_time - 2700 - (odr_time_offset_before);

    // TO Do ：need have time
    //data->timestamp_us = dsp_time;

    // printf("%d,%lld,%lld,%lld,%lld", odr_time_offset_,
    //            data->timestamp_us - last_odr_timestamp_, data->timestamp_us,
    //            before, (int64_t)before - (int64_t)data->timestamp_us);
    // printf("====================data->timestamp_us =
    // %lld\n", data->timestamp_us);

    data->accel[0] = accel[0] * ACCEL_SCALER;
    data->accel[1] = accel[1] * ACCEL_SCALER;
    data->accel[2] = accel[2] * ACCEL_SCALER;

    data->raw_accel[0] = accel[0];
    data->raw_accel[1] = accel[1];
    data->raw_accel[2] = accel[2];

    data->gyro[0] = gyro[0] * GYRO_SCALER;
    data->gyro[1] = gyro[1] * GYRO_SCALER;
    data->gyro[2] = gyro[2] * GYRO_SCALER;

    data->raw_gyro[0] = gyro[0];
    data->raw_gyro[1] = gyro[1];
    data->raw_gyro[2] = gyro[2];

    data->temperature = (float)temperture / 2.07 + TEMPERATURE_OFFSET;
    if (packet_cnt > data->MAX_PACKET_COUNT) {
      packet_cnt = data->MAX_PACKET_COUNT;
    }
    data->fifo_count = packet_size * packet_cnt;
    data->packet_count = packet_cnt;

    // printf("packet_cnt = %d, packet_size = %d , data->fifo_count = %d\n",
    // packet_cnt, packet_size, data->fifo_count);  //lmy_asi

    memcpy(data->fifo_data, p, data->fifo_count);
  }

  // this->Notify(data);
  return true;
}

bool WriteByte(uint8_t reg, uint8_t val) {
  if (SPI_WriteOneRegister((reg), val)) {
    return false;
  }
  rt_thread_mdelay(15);
  return true;
}

bool ReadBlock(uint8_t first_reg, uint8_t buf[], int len) {
  if (SPI_ReadMultRegister(first_reg, buf, len)) {
    return false;
  }

  return true;
}

/**
 * @brief Read fifo data
 *
 * @param[out] buf point to fifo data->
 *
 * @return 0 if success,
 *         non-zero value if failed.
 */
bool icm4x6xx_read_fifo_buf(uint8_t* buf, uint32_t len) {
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
bool icm4x6xx_odr_to_reg_val(float odr,
                                          icm4x6xx_sensor_odr* odr_reg) {
  if (odr >= (uint16_t)ICM4X6XX_ODR_8000)
    *odr_reg = ODR_8KHZ;
  else if(odr >= (uint16_t)ICM4X6XX_ODR_4000)
    *odr_reg = ODR_4KHZ;
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
bool icm4x6xx_set_accel_odr(float odr) {
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
bool icm4x6xx_set_gyro_odr(float odr) {
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
bool icm4x6xx_read_int_status(uint8_t* status) {
  if (SPI_ReadOneRegister(REG_INT_STATUS, status)) {
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
bool icm4x6xx_read_int_status2(uint8_t* status) {
  if (SPI_ReadOneRegister(REG_INT_STATUS2, status)) {
    return false;
  }

  return true;
}

static rt_size_t icm42688_read_data(imu_dev_t imu, rt_off_t pos, void *data, rt_size_t size) {
    if (data == NULL) {
        return 0;
    }
    icm42688_130_ReadRaw(&internal_raw_data_);
    memcpy(data, &internal_raw_data_, sizeof(internal_raw_data_));
    return size;
}

const static struct imu_ops icm42688_ops = {
    NULL,
    icm42688_read_data,
};

#define IMU_CONFIGURE                                                     \
    {                                                                     \
        3200,                 /* gyro ODR at 3.2KHz */                    \
        IMU_GYRO_MODE_NORMAL, /* NORMAL MODE (approximate ~751Hz DLPF) */ \
        800,                  /* accel ODR at 800Hz */                    \
        IMU_ACC_MODE_OSR2,    /* 178 dlpf */                              \
        GYRO_SCALE_2000DPS,                                               \
        ACC_SCALE_16G,                                                    \
        IMU_TEMP_SCALE,                                                   \
        IMU_TEMP_OFFSET,                                                  \
    }

static struct imu_device imu_dev = {
    .ops = &icm42688_ops,
    .config = IMU_CONFIGURE,
};

rt_err_t spi_device_init(const char* spi_slave_name)
{
    /* configure spi device */
    imu_spi_dev = (struct rt_spi_device *)rt_device_find(spi_slave_name);

    spi_config.data_width = 8;
    spi_config.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_config.max_hz = 1000000;
    rt_spi_configure(imu_spi_dev, &spi_config);
}

rt_err_t drv_42688_init(const char* spi_slave_name,const char* imu_device_name)
{
    spi_device_init(spi_slave_name);
    
    /* driver low-level init */
    RT_ASSERT(icm42688_130_init(false) == 0);

    /* init spi to 10M bps*/
    spi_config.max_hz = 10000000;
    rt_spi_configure(imu_spi_dev, &spi_config);

    /* register imu hal device */
    hal_imu_register(&imu_dev, imu_device_name, RT_DEVICE_FLAG_RDWR, RT_NULL);

    return RT_EOK;
}