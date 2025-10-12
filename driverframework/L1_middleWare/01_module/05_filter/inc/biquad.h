#ifndef BIQUAD_H__
#define BIQUAD_H__

#include <stdint.h>

#define M_SQRT2    1.41421356237309504880   // sqrt(2)s

#ifdef __cplusplus
extern "C" {
#endif

typedef struct biquad_filter_s {
    float q;
    float b0, b1, b2;
    float a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH,
    FILTER_BPF,
} biquad_filter_type_e;

void copyBiquadFilterCoeWeight(biquadFilter_t *src, biquadFilter_t *dst);
void updateBiquadFilter(biquadFilter_t *filter, float frequency, float time_interval, float Q, biquad_filter_type_e type, float weight);
void initBiquadFilter(biquadFilter_t *filter, float frequency, float time_interval, float Q, biquad_filter_type_e type, float weight);
float applyDF1BiquadFilter(biquadFilter_t *filter, float input);
float applyDF1WeightedBiquadFilter(biquadFilter_t* filter, float input);
float applyDF2BiquadFilter(biquadFilter_t *filter, float input);

#ifdef __cplusplus
}
#endif

#endif  // BIQUAD_H__
