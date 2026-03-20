#ifndef __MAG_SD_EVAL_H__
#define __MAG_SD_EVAL_H__

#include <rtdef.h>
#include "mag.h"

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t magSdEvalStartStream(void);
rt_err_t magSdEvalStopStream(void);
rt_err_t magSdEvalMarkTestStart(void);
rt_err_t magSdEvalMarkTestStop(void);

rt_bool_t magSdEvalIsStreamEnabled(void);
rt_bool_t magSdEvalIsTestActive(void);

const char* magSdEvalGetUartName(void);
rt_uint32_t magSdEvalGetBaudRate(void);
rt_uint32_t magSdEvalGetPeriodMs(void);

void magSdEvalOnRawSample(const mag_report_t* raw_data);

#ifdef __cplusplus
}
#endif

#endif /* __MAG_SD_EVAL_H__ */
