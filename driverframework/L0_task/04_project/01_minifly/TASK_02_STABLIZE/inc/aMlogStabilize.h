#ifndef __MLOG_STABILIZER_H__
#define __MLOG_STABILIZER_H__

#include <rtthread.h>
#include "stabilizerTypes.h"

#ifdef TASK_TOOL_02_SD_MLOG
#include "mlog.h"
#endif

/* Function declarations */
void mlogStabilizerInit(void);
void mlogStabilizerCopyAngleRateData(const attitude_t* rate_desired, const attitude_t* rate_current);
void mlogStabilizerPushAngleRateData(uint32_t timestamp);

#endif /* __MLOG_STABILIZER_H__ */
