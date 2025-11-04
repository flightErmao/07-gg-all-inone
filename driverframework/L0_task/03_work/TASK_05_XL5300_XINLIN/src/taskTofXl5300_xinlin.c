#include <rtthread.h>
#include <rtdevice.h>

#include "XL5300_API.h"
#include "XL5300_UserPlatform.h"
#include "XL5300_Firmware_8.h"
#include "function.h"

#ifdef RT_USING_STM32_HAL
#include "stm32f4xx_hal.h"
#endif

#ifdef RT_USING_STM32_H7XX
#include "stm32h7xx_hal.h"
#endif

#ifdef RT_USING_PY32_HAL
#include "py32f0xx_hal.h"
#endif

#ifndef WORK_TASK_TOF_XL5300_XINLIN_THREAD_STACK
#define WORK_TASK_TOF_XL5300_XINLIN_THREAD_STACK 3072
#endif

#ifndef WORK_TASK_TOF_XL5300_XINLIN_THREAD_PRIO
#define WORK_TASK_TOF_XL5300_XINLIN_THREAD_PRIO 20
#endif

#define THREAD_TIMESLICE 5

extern XL5300_Calibration_TypeDef XL5300_Cali_Data;

static void xl5300_gpio_init(void)
{
    /* 使能GPIO时钟，配置 XSHUT 引脚为推挽输出（宏在 XL5300_API.h 中定义） */
#if defined(__HAL_RCC_GPIOA_CLK_ENABLE)
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = XSHUT_Pin1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(XSHUT_GPIO_Port, &GPIO_InitStruct);

    /* 软I2C使用 PB6/PB7，确保端口时钟已开。xl_sw_i2c.c 内部直接操作寄存器，无需额外模式配置 */
}

static void xl5300_task_entry(void *parameter)
{
    XL5300_Status ret = XL5300_OK;
    XL5300_MEASURE_TypeDef result;

    /* 使用寄存器查询方式读取中断状态（不依赖外部中断引脚） */
    XL5300_Cali_Data.XL5300_Interrupt_Mode_Status = 0x00;

    xl5300_gpio_init();

    rt_kprintf("[XL5300] init...\n");

    /* 芯片上电与寄存器初始化 */
    ret |= VI530x_Chip_Init();
    if (ret) {
        rt_kprintf("[XL5300] chip init fail, ret=0x%02X\n", ret);
        return;
    }

    /* 下固件 */
    ret |= VI530x_Download_Firmware((uint8_t*)Firmware_Ranging, LoadFirmware());
    if (ret) {
        rt_kprintf("[XL5300] download fw fail, ret=0x%02X\n", ret);
    }

    /* 基本配置：帧率/积分次数、温度、连续测距 */
    ret |= XL5300_Set_Integralcounts_Frame(20, 321000);
    ret |= XL5300_Temp_Enable(0x01);
    ret |= XL5300_Start_Continue_Ranging_Cmd();

    if (ret) {
        rt_kprintf("[XL5300] config warn, ret=0x%02X\n", ret);
    } else {
        rt_kprintf("[XL5300] ready.\n");
    }

    while (1) {
        if (XL5300_Get_Measure1_Data(&result) == XL5300_OK) {
            rt_kprintf("[XL5300] tof=%4d mm, conf=%3d, peak=%lu, noise=%u, inte=%lu\n",
                       result.correction_tof,
                       result.confidence,
                       (unsigned long)result.peak,
                       (unsigned)result.noise,
                       (unsigned long)result.intecounts);
        }
        rt_thread_mdelay(5);
    }
}

static int xl5300_task_init(void)
{
    rt_thread_t th = rt_thread_create("tof_xl5300", xl5300_task_entry, RT_NULL,
                                      WORK_TASK_TOF_XL5300_XINLIN_THREAD_STACK,
                                      WORK_TASK_TOF_XL5300_XINLIN_THREAD_PRIO,
                                      THREAD_TIMESLICE);
    if (th == RT_NULL) {
        rt_kprintf("[XL5300] thread create fail\n");
        return -RT_ERROR;
    }
    rt_thread_startup(th);
    rt_kprintf("[XL5300] thread started\n");
    return RT_EOK;
}

#ifdef WORK_TASK_TOF_XL5300_XINLIN_EN
INIT_APP_EXPORT(xl5300_task_init);
#endif


