#ifndef PTX_H__
#define PTX_H__

#ifdef __cplusplus
extern "C" {
#endif

// betaflight style ptx filter
// which are essentially discrete RC filters
typedef struct bf_pt1Filter_s {
    float state;
    float k;
} bf_pt1Filter_t;

typedef struct bf_pt2Filter_s {
    float state;
    float state1;
    float k;
} bf_pt2Filter_t;

typedef struct bf_pt3Filter_s {
    float state;
    float state1;
    float state2;
    float k;
} bf_pt3Filter_t;

// float bf_pt1FilterCalcGain(float f_cut, float dT);
void bf_pt1FilterInit(bf_pt1Filter_t *filter, float f_cut, float dT);
void bf_pt1FilterUpdateCutoff(bf_pt1Filter_t *filter, float f_cut, float dT);
float bf_pt1FilterApply(bf_pt1Filter_t *filter, float input);

void bf_pt2FilterInit(bf_pt2Filter_t *filter, float f_cut, float dT);
void bf_pt2FilterUpdateCutoff(bf_pt2Filter_t *filter, float f_cut, float dT);
float bf_pt2FilterApply(bf_pt2Filter_t *filter, float input);

void bf_pt3FilterInit(bf_pt3Filter_t *filter, float f_cut, float dT);
void bf_pt3FilterUpdateCutoff(bf_pt3Filter_t *filter, float f_cut, float dT);
float bf_pt3FilterApply(bf_pt3Filter_t *filter, float input);

void qs_pt1FilterInit(bf_pt1Filter_t *filter, float f_cut, float dT);
void qs_pt1FilterUpdateCutoff(bf_pt1Filter_t *filter, float f_cut, float dT);
float qs_pt1FilterApply(bf_pt1Filter_t *filter, float input);

#ifdef __cplusplus
}
#endif

#endif  // PTX_H__