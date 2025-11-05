/****************************************************************************
 *
 * TOF XL530X Data Send via ANOTC_TELEM
 * Sends TOF sensor data through ANOTC_TELEM protocol instead of UART
 *
 ****************************************************************************/

#include <rtthread.h>

#if defined(TASK_TOOL_01_ANOTC_TELEM_EN) && defined(WORK_TASK_TOF_XL530X_ANOTC_TELEM_EN)

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"
#include "../inc/taskTofXl530x.h"
#include "../inc/VI530x_API.h"

void sendTofXl530xData(uint16_t count_ms) {
    VI530x_MEASURE_TypeDef* tof_data = getTofXl530xData();
    if (tof_data == RT_NULL) {
        return;
    }
    
    /* Send TOF data every 10ms */
    if (!(count_ms % PERIOD_10ms)) {
        /* Send TOF data: correction_tof, confidence, peak_low, peak_high, noise_low, noise_high, intecounts_low, intecounts_high */
        /* Split uint32_t values into two uint16_t values */
        uint16_t peak_low = (uint16_t)(tof_data->peak & 0xFFFF);
        uint16_t peak_high = (uint16_t)((tof_data->peak >> 16) & 0xFFFF);
        uint16_t noise_low = (uint16_t)(tof_data->noise & 0xFFFF);
        uint16_t noise_high = (uint16_t)((tof_data->noise >> 16) & 0xFFFF);
        uint16_t intecounts_low = (uint16_t)(tof_data->intecounts & 0xFFFF);
        uint16_t intecounts_high = (uint16_t)((tof_data->intecounts >> 16) & 0xFFFF);
        
        /* Send as uint16_8: correction_tof, confidence, peak_low, peak_high, noise_low, noise_high, intecounts_low, intecounts_high */
        sendUserDataUint16_8(1, 
                             (uint16_t)tof_data->correction_tof,  /* correction_tof (int16_t -> uint16_t) */
                             (uint16_t)tof_data->confidence,     /* confidence */
                             peak_low,                            /* peak low 16 bits */
                             peak_high,                           /* peak high 16 bits */
                             noise_low,                           /* noise low 16 bits */
                             noise_high,                          /* noise high 16 bits */
                             intecounts_low,                      /* intecounts low 16 bits */
                             intecounts_high);                    /* intecounts high 16 bits */
    }
}

int addPeriodFunListTofXl530x(void) {
    anotcTelemAddSensorFunc(sendTofXl530xData);
    return 0;
}

#ifdef WORK_TASK_TOF_XL530X_EN
INIT_APP_EXPORT(addPeriodFunListTofXl530x);
#endif

#endif /* TASK_TOOL_01_ANOTC_TELEM_EN && WORK_TASK_TOF_XL530X_ANOTC_TELEM_EN */

