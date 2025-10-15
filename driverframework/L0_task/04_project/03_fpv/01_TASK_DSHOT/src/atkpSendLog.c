#include <rtthread.h>
#include "taskDshot.h"
#include "floatConvert.h"

#ifdef TASK_TOOL_01_ANOTC_TELEM_EN

#include "protocolAtkpInterface.h"
#include "taskAnotcTelem.h"

void sendDshotMotorData(uint16_t count_ms) {
  if (!(count_ms % PERIOD_30ms)) {
    uint16_t m1, m2, m3, m4;
    task_dshot_get_mapped(&m1, &m2, &m3, &m4);

#ifdef PROJECT_MINIFLY_TASK_DSHOT_ATKP_LOG_GROUP
    uint8_t group = PROJECT_MINIFLY_TASK_DSHOT_ATKP_LOG_GROUP;
#else
    uint8_t group = 1;
#endif
    
    sendUserDatauint16_4(group, m1, m2, m3, m4);
  }
}

int addPeriodFunListDshot(void) {
  anotcTelemAddSensorFunc(sendDshotMotorData);
  return 0;
}

#ifdef PROJECT_MINIFLY_TASK_DSHOT_ATKP_LOG_EN
INIT_APP_EXPORT(addPeriodFunListDshot);
#endif

#endif
