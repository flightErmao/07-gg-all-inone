#ifndef __MLOG_IMU_H__
#define __MLOG_IMU_H__

#include <rtthread.h>
#include "sensorsTypes.h"

#ifdef TASK_TOOL_02_SD_MLOG
#include "mlog.h"
#endif

/* Function declarations */
void mlogImuInit(void);
void mlogImuCopyAccData(const Axis3f* acc_before, const Axis3f* acc_after);
void mlogImuCopyGyroData(const Axis3f* gyro_before, const Axis3f* gyro_after);
void mlogImuPushData(uint32_t timestamp);

#endif /* __MLOG_IMU_H__ */
