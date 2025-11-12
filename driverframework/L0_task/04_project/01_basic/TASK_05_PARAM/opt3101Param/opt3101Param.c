#include "opt3101Param.h"

#include <rtdef.h>
#include <string.h>

static opt3101_param_blob_t g_opt3101_blob;

static void opt3101_param_default(void* address, uint8_t size) {
  opt3101_param_blob_t* blob = (opt3101_param_blob_t*)address;
  RT_ASSERT(size == sizeof(opt3101_param_blob_t));
  opt3101_param_reset(blob);
}

void opt3101_param_reset(opt3101_param_blob_t* blob) {
  if (blob == RT_NULL) {
    return;
  }
  memset(blob, 0, sizeof(opt3101_param_blob_t));
  blob->magic = OPT3101_PARAM_MAGIC;
  blob->version = OPT3101_PARAM_VERSION;
}

opt3101_param_blob_t* opt3101_param_blob(void) {
  return &g_opt3101_blob;
}

static param_list g_opt3101_params[] = {
    {(void*)&g_opt3101_blob, sizeof(g_opt3101_blob), "opt3101_calibration", "vb", opt3101_param_default},
};

param_list* opt3101Param_list(void) {
  return g_opt3101_params;
}

size_t opt3101Param_count(void) {
  return sizeof(g_opt3101_params) / sizeof(g_opt3101_params[0]);
}

