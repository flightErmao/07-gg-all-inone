#include "ptx.h"
#include <math.h>
#include "trigonometric.h"

static float bf_pt1FilterCalcGain(float f_cut, float dT) {
    float RC = 1 / (2 * M_PIf * f_cut);
    return dT / (RC + dT);
}

void bf_pt1FilterInit(bf_pt1Filter_t *filter, float f_cut, float dT) {
    filter->state = 0.0f;
    filter->k = bf_pt1FilterCalcGain(f_cut, dT);
}

void bf_pt1FilterUpdateCutoff(bf_pt1Filter_t *filter, float f_cut, float dT) {
    filter->k = bf_pt1FilterCalcGain(f_cut, dT);
}

float bf_pt1FilterApply(bf_pt1Filter_t *filter, float input) {
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

static float bf_pt2FilterCalcGain(float f_cut, float dT) {
    const float order = 2.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    return dT / (RC + dT);
}

void bf_pt2FilterInit(bf_pt2Filter_t *filter, float f_cut, float dT) {
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = bf_pt2FilterCalcGain(f_cut, dT);
}

void bf_pt2FilterUpdateCutoff(bf_pt2Filter_t *filter, float f_cut, float dT) {
    filter->k = bf_pt2FilterCalcGain(f_cut, dT);
}

float bf_pt2FilterApply(bf_pt2Filter_t *filter, float input) {
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}

static float bf_pt3FilterCalcGain(float f_cut, float dT) {
    const float order = 3.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    return dT / (RC + dT);
}

void bf_pt3FilterInit(bf_pt3Filter_t *filter, float f_cut, float dT) {
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = bf_pt3FilterCalcGain(f_cut, dT);
}

void bf_pt3FilterUpdateCutoff(bf_pt3Filter_t *filter, float f_cut, float dT) {
    filter->k = bf_pt3FilterCalcGain(f_cut, dT);
}

float bf_pt3FilterApply(bf_pt3Filter_t *filter, float input) {
    filter->state2 = filter->state2 + filter->k * (input - filter->state2);
    filter->state1 = filter->state1 + filter->k * (filter->state2 - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}

static float qs_pt1FilterCalcGain(float f_cut, float dT) {
    float filtertime = 1.0f / f_cut;
    return (1.0f - (6.0f * dT) / (3.0f * dT + filtertime));
}

void qs_pt1FilterInit(bf_pt1Filter_t *filter, float f_cut, float dT) {
    filter->state = 0.0f;
    filter->k = qs_pt1FilterCalcGain(f_cut, dT);
}

void qs_pt1FilterUpdateCutoff(bf_pt1Filter_t *filter, float f_cut, float dT) {
    filter->k = qs_pt1FilterCalcGain(f_cut, dT);
}

float qs_pt1FilterApply(bf_pt1Filter_t *filter, float input) {
    filter->state = filter->state * filter->k + input * (1 - filter->k);
    return filter->state;
}
