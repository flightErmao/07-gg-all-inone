#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "axis.h"

typedef struct
{
    Axis3f bias;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 *buffer;
    uint32_t bufferSize;
} BiasObj;

void bias_init(BiasObj *bias, Axis3i16 *buffer, uint32_t buffer_size);
void bias_add_sample(BiasObj *bias, int16_t x, int16_t y, int16_t z);
void bias_calc_var_mean(const BiasObj *bias, Axis3f *variance_out, Axis3f *mean_out);

void apply_axis3f_lpf(lpf2pData *filters, Axis3f *in);

#endif

