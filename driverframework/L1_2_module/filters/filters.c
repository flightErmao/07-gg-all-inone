#include "filters.h"

void bias_init(BiasObj *bias, Axis3i16 *buffer, uint32_t buffer_size)
{
    bias->isBufferFilled = false;
    bias->isBiasValueFound = false;
    bias->buffer = buffer;
    bias->bufferSize = buffer_size;
    bias->bufHead = buffer;
}

void bias_add_sample(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;
    if (bias->bufHead >= &bias->buffer[bias->bufferSize])
    {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

void bias_calc_var_mean(const BiasObj *bias, Axis3f *variance_out, Axis3f *mean_out)
{
    int64_t sum[3] = {0};
    int64_t sumsq[3] = {0};
    for (uint32_t i = 0; i < bias->bufferSize; i++)
    {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumsq[0] += (int64_t)bias->buffer[i].x * bias->buffer[i].x;
        sumsq[1] += (int64_t)bias->buffer[i].y * bias->buffer[i].y;
        sumsq[2] += (int64_t)bias->buffer[i].z * bias->buffer[i].z;
    }
    variance_out->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / bias->bufferSize);
    variance_out->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / bias->bufferSize);
    variance_out->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / bias->bufferSize);

    mean_out->x = (float)sum[0] / bias->bufferSize;
    mean_out->y = (float)sum[1] / bias->bufferSize;
    mean_out->z = (float)sum[2] / bias->bufferSize;
}

void apply_axis3f_lpf(lpf2pData *filters, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        in->axis[i] = lpf2pApply(&filters[i], in->axis[i]);
    }
}

