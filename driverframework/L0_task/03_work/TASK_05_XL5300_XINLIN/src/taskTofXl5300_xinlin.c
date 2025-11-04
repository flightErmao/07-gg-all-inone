#include <rtthread.h>
#include <rtdevice.h>

#include "XL5300_API.h"
#include "XL5300_UserPlatform.h"
#include "XL5300_Firmware_8.h"
#include "I2cInterface.h"
#include <string.h>
#include "das.h"

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

typedef struct
{
    int16_t  correction_tof;
    uint8_t  confidence;
    uint32_t intecounts;
    uint32_t peak;
    uint16_t noise;
    uint16_t xtalk_count;
} xl5300_measure_t;

static XL5300_Status xl5300_read_measure_once(xl5300_measure_t *m)
{
    uint8_t buf[32];
    if (I2C_ReadXBytes(0x0C, buf, 32) != XL5300_OK)
        return XL5300_ERROR;
    int16_t raw_tof = 0;
    uint32_t intecounts = 0, peak = 0;
    uint16_t noise = 0, xtalk = 0;
    memcpy(&raw_tof, &buf[12], 2);
    memcpy(&intecounts, &buf[22], 4);
    intecounts &= 0x00FFFFFFu;
    memcpy(&peak, &buf[28], 4);
    memcpy(&noise, &buf[26], 2);
    memcpy(&xtalk, &buf[14], 2);
    m->correction_tof = raw_tof < 0 ? 0 : raw_tof;
    m->confidence = 100;
    m->intecounts = intecounts;
    m->peak = peak;
    m->noise = noise;
    m->xtalk_count = xtalk;
    return XL5300_OK;
}

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
    xl5300_measure_t result;
    I2cInterface_t i2c;

    /* 使用寄存器查询方式读取中断状态（不依赖外部中断引脚） */
    XL5300_Cali_Data.XL5300_Interrupt_Mode_Status = 0x00;

    xl5300_gpio_init();

    /* 获取 I2C 接口并绑定到 SDK 平台层 */
    if (get_i2c_interface(WORK_TASK_TOF_XL5300_XINLIN_I2C_NAME, (XL5300_DEVICE_ADDR >> 1), &i2c) != RT_EOK)
    {
        rt_kprintf("[XL5300] get i2c fail\n");
        return;
    }
    set_i2c_fast_plus_speed(i2c.i2c_dev);
    XL5300_Bind_I2C_Interface(&i2c);

    rt_kprintf("[XL5300] init...\n");

    /* 芯片上电与寄存器初始化 */
    ret |= VI530x_Chip_Init();
    if (ret) {
        rt_kprintf("[XL5300] chip init fail, ret=0x%02X\n", ret);
        return;
    }

    /* 下固件（若固件长度为0则跳过） */
    {
        unsigned short fw_size = LoadFirmware();
        if (fw_size > 0)
        {
            ret |= VI530x_Download_Firmware((uint8_t*)Firmware_Ranging, fw_size);
            if (ret)
            {
                rt_kprintf("[XL5300] download fw fail, ret=0x%02X\n", ret);
            }
        }
        else
        {
            rt_kprintf("[XL5300] no external fw, skip download\n");
        }
    }

    /* 基本配置：帧率/积分次数、温度、连续测距 */
    ret |= XL5300_Set_Integralcounts_Frame(20, 321000);
    ret |= XL5300_Temp_Enable(0x01);
    ret |= WriteCommand(0x0F);

    if (ret) {
        rt_kprintf("[XL5300] config warn, ret=0x%02X\n", ret);
    } else {
        rt_kprintf("[XL5300] ready.\n");
    }

    while (1) {
        if (xl5300_read_measure_once(&result) == XL5300_OK) {
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


