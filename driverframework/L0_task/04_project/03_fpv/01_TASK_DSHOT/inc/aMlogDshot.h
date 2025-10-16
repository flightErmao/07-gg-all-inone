#ifndef __MLOG_DSHOT_H__
#define __MLOG_DSHOT_H__

#include <rtthread.h>
#include "taskDshot.h"

/* Mlog DShot data structure */
typedef struct {
  uint32_t timestamp;
  uint16_t dshot_mapped[4];  // mapped DShot values (48~2048)
#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
  uint16_t erpm[4];  // Raw eRPM/100 values from bidirectional DShot device
#endif
} __attribute__((aligned(4))) mlogDshotData_t;

/* Function declarations */
void mlogDshotInit(void);
void mlogDshotPush(const mlogDshotData_t* data);

#endif /* __MLOG_DSHOT_H__ */
