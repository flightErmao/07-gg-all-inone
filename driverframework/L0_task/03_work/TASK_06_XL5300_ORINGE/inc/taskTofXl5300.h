/****************************************************************************
 *
 * TOF XL5300 Task Configuration Header
 *
 ****************************************************************************/

#ifndef __TASK_TOF_XL5300_H__
#define __TASK_TOF_XL5300_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drv_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Thread Configuration */
#define THREAD_PRIORITY     20
#define THREAD_STACK_SIZE   4096
#define THREAD_TIMESLICE    5

/* GPIO Pin Definitions */
#define XSHUT_PIN_NUM       GET_PIN(B, 6)  /* Reset pin PB6 */
#define INT_PIN_NUM         GET_PIN(B, 7)  /* Interrupt pin PB7 */

/* Interrupt Event Flag (refer to mpu6500) */
#ifndef TOF_INT_EVENT_FLAG
#define TOF_INT_EVENT_FLAG  (1u << 0)
#endif

/* VI530x I2C Address */
/* VI530x default 8-bit address is 0xD8, get_i2c_interface needs 7-bit address, so 0xD8 >> 1 = 0x6C */
#define VI530x_I2C_ADDR     0x6C

/* Interrupt Mode Status Values */
#define VI530x_INTERRUPT_MODE_HW    0x88  /* GPIO hardware interrupt */
#define VI530x_INTERRUPT_MODE_SW    0x00  /* Register polling mode */

#ifdef __cplusplus
}
#endif

#endif /* __TASK_TOF_XL5300_H__ */

