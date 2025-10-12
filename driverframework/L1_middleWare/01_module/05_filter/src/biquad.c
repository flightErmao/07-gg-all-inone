#include "biquad.h"
#include "trigonometric.h"

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float getBiquadNotchQ(float center_freq, float cutoff_freq) {
    return center_freq * cutoff_freq / (center_freq * center_freq - cutoff_freq * cutoff_freq);
}

void copyBiquadFilterCoeWeight(biquadFilter_t *src, biquadFilter_t *dst) {
    dst->b0 = src->b0;
    dst->b1 = src->b1;
    dst->b2 = src->b2;
    dst->a1 = src->a1;
    dst->a2 = src->a2;
    dst->weight = src->weight;
}

void updateBiquadFilter(biquadFilter_t *filter, float frequency, float time_interval, float Q,
                        biquad_filter_type_e type, float weight) {
    // setup variables
    const float omega = 2.0f * M_PIf * frequency * time_interval;
    const float sn = sin_fast(omega);
    const float cs = cos_fast(omega);
    const float alpha = sn / (2.0f * Q);

    switch (type) {
        case FILTER_LPF:
            // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
            // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
            filter->b1 = 1 - cs;
            filter->b0 = filter->b1 * 0.5f;
            filter->b2 = filter->b0;
            filter->a1 = -2 * cs;
            filter->a2 = 1 - alpha;
            break;
        case FILTER_NOTCH:
            filter->b0 = 1;
            filter->b1 = -2 * cs;
            filter->b2 = 1;
            filter->a1 = filter->b1;
            filter->a2 = 1 - alpha;
            break;
        case FILTER_BPF:
            filter->b0 = alpha;
            filter->b1 = 0;
            filter->b2 = -alpha;
            filter->a1 = -2 * cs;
            filter->a2 = 1 - alpha;
            break;
    }

    const float a0 = 1 + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    filter->weight = weight;
}

void initBiquadFilter(biquadFilter_t *filter, float frequency, float time_interval, float Q, biquad_filter_type_e type,
                      float weight) {
    updateBiquadFilter(filter, frequency, time_interval, Q, type, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
float applyDF1BiquadFilter(biquadFilter_t *filter, float input) {
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 -
                         filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

/* Computes a biquadFilter_t filter in df1 and crossfades input with output */
float applyDF1WeightedBiquadFilter(biquadFilter_t *filter, float input) {
    // compute result
    const float result = applyDF1BiquadFilter(filter, input);

    // crossfading of input and output to turn filter on/off gradually
    return filter->weight * result + (1 - filter->weight) * input;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in
 * coefficients */
float applyDF2BiquadFilter(biquadFilter_t *filter, float input) {
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}
