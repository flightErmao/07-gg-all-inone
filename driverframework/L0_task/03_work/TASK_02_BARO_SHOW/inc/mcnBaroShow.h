#ifndef MCN_BARO_REPORT_H__
#define MCN_BARO_REPORT_H__

#include "barometer.h"
#include "uMCN.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MCN topic for barometer data */
MCN_DECLARE(baro);

/* Initialize MCN barometer reporting */
int mcnBaroReportInit(void);

/* Publish barometer data to MCN */
int mcnBaroReportPublish(const baro_report_t* baro_data);

/* Acquire barometer data from MCN */
int mcnBaroReportAcquire(baro_report_t* baro_data);

#ifdef __cplusplus
}
#endif

#endif /* MCN_BARO_REPORT_H__ */