#ifndef OPT3101_PARAM_H
#define OPT3101_PARAM_H

#include <stddef.h>
#include <stdint.h>

#include "uparam.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPT3101_PARAM_MAGIC (0x4F503031UL)  /* "OP01" */
#define OPT3101_PARAM_VERSION (1U)
#define OPT3101_PARAM_MAX_REG_COUNT (48U)

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t count;
  uint8_t address[OPT3101_PARAM_MAX_REG_COUNT];
  uint32_t value[OPT3101_PARAM_MAX_REG_COUNT];
} opt3101_param_blob_t;

param_list* opt3101Param_list(void);
size_t opt3101Param_count(void);

opt3101_param_blob_t* opt3101_param_blob(void);
void opt3101_param_reset(opt3101_param_blob_t* blob);

#ifdef __cplusplus
}
#endif

#endif /* OPT3101_PARAM_H */

