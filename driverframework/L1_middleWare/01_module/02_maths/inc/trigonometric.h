#ifndef TRIGONOMETRIC_H__
#define TRIGONOMETRIC_H__

#define M_PIf       3.14159265358979323846f

#ifdef __cplusplus
extern "C" {
#endif

float sin_fast(float x);
float cos_fast(float x);
float tan_fast(float x);

#ifdef __cplusplus
}
#endif

#endif  // TRIGONOMETRIC_H__