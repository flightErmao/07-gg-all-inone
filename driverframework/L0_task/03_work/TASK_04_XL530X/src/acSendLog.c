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
        float payload[] = {
            (float)tof_data->correction_tof,
            (float)tof_data->confidence,
            (float)tof_data->peak,
            (float)tof_data->noise,
            (float)tof_data->intecounts
        };

        sendUserDatafloatN(1, payload, sizeof(payload) / sizeof(payload[0]));
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

