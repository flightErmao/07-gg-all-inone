#ifndef MCN_MAG_REPORT_H__
#define MCN_MAG_REPORT_H__

#include "mag.h"
#include "uMCN.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MCN topic for magnetometer data */
MCN_DECLARE(mag);

/* Initialize MCN magnetometer reporting */
int mcnMagReportInit(void);

/* Publish magnetometer data to MCN */
int mcnMagReportPublish(const mag_report_t* mag_data);

/* Acquire magnetometer data from MCN */
int mcnMagReportAcquire(mag_report_t* mag_data);

#ifdef __cplusplus
}
#endif

#endif /* MCN_MAG_REPORT_H__ */

