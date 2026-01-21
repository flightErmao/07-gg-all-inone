#ifndef MCN_MAG_REPORT_H__
#define MCN_MAG_REPORT_H__

#include "mag.h"
#include "uMCN.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MCN topic for magnetometer data */
MCN_DECLARE(mag);

/* MCN topic for raw magnetometer data (LSB values) */
MCN_DECLARE(mag_raw_data);

/* Initialize MCN magnetometer reporting */
int mcnMagReportInit(void);

/* Initialize MCN raw magnetometer data reporting */
int mcnMagRawDataInit(void);

/* Publish magnetometer data to MCN */
int mcnMagReportPublish(const mag_report_t* mag_data);

/* Publish raw magnetometer data to MCN */
int mcnMagRawDataPublish(const mag_report_t* raw_data);

/* Acquire magnetometer data from MCN */
int mcnMagReportAcquire(mag_report_t* mag_data);

/* Acquire raw magnetometer data from MCN */
int mcnMagRawDataAcquire(mag_report_t* raw_data);

#ifdef __cplusplus
}
#endif

#endif /* MCN_MAG_REPORT_H__ */

