#pragma once

/**
 * @file Functions.hpp
 * Math library functions - compatibility header for RT-Thread
 */

#include <cmath>
#include <cfloat>
#include <limits>

namespace math
{

/**
 * Check if a floating point value is finite
 * @param x value to check
 * @return true if finite, false otherwise
 */
template<typename T>
inline bool isFinite(const T &x)
{
	return std::isfinite(static_cast<double>(x));
}

/**
 * Check if a floating point value is NaN
 * @param x value to check
 * @return true if NaN, false otherwise
 */
template<typename T>
inline bool isNaN(const T &x)
{
	return std::isnan(static_cast<double>(x));
}

/**
 * Get maximum value for a type
 * @return maximum value
 */
template<typename T>
inline T max()
{
	return std::numeric_limits<T>::max();
}

/**
 * Get maximum of two values
 * @param a first value
 * @param b second value
 * @return maximum of a and b
 */
template<typename T>
inline T max(const T &a, const T &b)
{
	return (a > b) ? a : b;
}

/**
 * Get minimum value for a type
 * @return minimum value
 */
template<typename T>
inline T min()
{
	return std::numeric_limits<T>::lowest();
}

/**
 * Get minimum of two values
 * @param a first value
 * @param b second value
 * @return minimum of a and b
 */
template<typename T>
inline T min(const T &a, const T &b)
{
	return (a < b) ? a : b;
}

/**
 * Constrain a value between min and max
 * @param val value to constrain
 * @param min_val minimum value
 * @param max_val maximum value
 * @return constrained value
 */
template<typename T>
inline T constrain(const T &val, const T &min_val, const T &max_val)
{
	if (val < min_val) {
		return min_val;
	} else if (val > max_val) {
		return max_val;
	}
	return val;
}

} // namespace math

