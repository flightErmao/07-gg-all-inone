#ifndef MCN_STABLIZE_H__
#define MCN_STABLIZE_H__

#include "stabilizerTypes.h"
#include "uMCN.h"
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* State MCN functions */
int mcnStateReportInit(void);
int mcnStateReportPublish(const state_t* state_data);
void stabilizerGetState(state_t* state);

#ifdef PROJECT_MINIFLY_TASK_DSHOT_EN
/* DShot command data structure */
typedef struct {
  uint16_t motor_val[4]; /* 0~65535 from mixerControl */
  uint32_t timestamp;
} dshot_cmd_bus_t;

/* MCN topic declaration */
MCN_DECLARE(dshot_cmd);

/* DShot command MCN functions */
int mcnDshotCmdInit(void);
int mcnDshotCmdPublish(const dshot_cmd_bus_t* cmd_data);
int mcnDshotCmdAcquire(dshot_cmd_bus_t* cmd_data);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MCN_STABLIZE_H__ */
