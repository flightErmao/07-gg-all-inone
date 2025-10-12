#ifndef MCN_STABLIZE_H__
#define MCN_STABLIZE_H__

#include "stabilizerTypes.h"
#include "uMCN.h"
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t motor_val[4]; /* 0~65535 from mixerControl */
  uint32_t timestamp;
} mixer_data_t;

int mcnStatePub(const state_t* state_data);
void mcnStateAcquire(state_t* state);

int mcnMixerPublish(const mixer_data_t* cmd_data);
int mcnMixerAcquire(mixer_data_t* cmd_data);

#ifdef __cplusplus
}
#endif

#endif /* MCN_STABLIZE_H__ */
