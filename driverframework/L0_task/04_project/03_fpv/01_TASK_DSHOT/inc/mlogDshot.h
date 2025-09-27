#ifndef __MLOG_DSHOT_H__
#define __MLOG_DSHOT_H__

#include <rtthread.h>
#include "taskDshot.h"

/* Function declarations */
void mlogDshotInit(void);
void mlogDshotPush(const uint16_t* dshot_mapped, uint32_t timestamp);

#endif /* __MLOG_DSHOT_H__ */
