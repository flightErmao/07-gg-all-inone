#ifndef MCN_STABLIZE_H__
#define MCN_STABLIZE_H__

#include "stabilizerTypes.h"
#include "uMCN.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize MCN state reporting */
int mcnStateReportInit(void);

/* Publish state data to MCN */
int mcnStateReportPublish(const state_t* state_data);

void stabilizerGetState(state_t* state);

#ifdef __cplusplus
}
#endif

#endif /* MCN_STABLIZE_H__ */
