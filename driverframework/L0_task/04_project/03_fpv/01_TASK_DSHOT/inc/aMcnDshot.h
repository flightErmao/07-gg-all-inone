#ifndef __A_MCN_DSHOT_H__
#define __A_MCN_DSHOT_H__

#include <rtthread.h>
#include "uMCN.h"

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
/* RPM data structure for MCN */
typedef struct {
  float rpm[4];      /* RPM values for 4 motors */
  uint32_t timestamp;
} rpm_data_bus_t;

/* MCN topic declaration */
MCN_DECLARE(rpm_data);

/* RPM data MCN functions */
int mcnRpmDataInit(void);
int mcnRpmDataPublish(const rpm_data_bus_t* rpm_data);
int mcnRpmDataAcquire(rpm_data_bus_t* rpm_data);
#endif

#endif /* __A_MCN_DSHOT_H__ */

