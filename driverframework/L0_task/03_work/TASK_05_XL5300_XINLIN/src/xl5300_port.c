#include <rtthread.h>
#include <stdint.h>
#include "das.h"

void XL5300_Delay_Ms(uint16_t nMs)
{
    rt_thread_mdelay(nMs);
}

/* 提供 SDK 需要的全局校准数据对象定义 */
XL5300_Calibration_TypeDef XL5300_Cali_Data;


