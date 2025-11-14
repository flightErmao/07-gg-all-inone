#pragma once

/**
 * @file defines.h
 * PX4 platform common defines - compatibility header for RT-Thread
 */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mathematical constants
#ifndef M_PI_PRECISE
#define M_PI_PRECISE 3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// Check if a floating point value is finite
#ifndef PX4_ISFINITE
#define PX4_ISFINITE(x) (isfinite(x))
#endif

// Check if a floating point value is NaN
#ifndef PX4_ISNAN
#define PX4_ISNAN(x) (isnan(x))
#endif

// Check if a value is finite (for any type)
#ifndef PX4_ISFINITE_F
#define PX4_ISFINITE_F(x) (isfinite((float)(x)))
#endif

// Check if a value is NaN (for any type)
#ifndef PX4_ISNAN_F
#define PX4_ISNAN_F(x) (isnan((float)(x)))
#endif

#ifdef __cplusplus
}
#endif

