/****************************************************************************
 *
 * C wrapper header for MulticopterRateControl C++ class
 *
 ****************************************************************************/

#ifndef __MULTICOPTER_RATE_CONTROL_WRAPPER_H__
#define __MULTICOPTER_RATE_CONTROL_WRAPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MulticopterRateControl instance
 * @param vtol VTOL mode flag (0 = false, non-zero = true)
 * @return 0 on success, -1 on failure
 */
int multicopter_rate_control_init(int vtol);

/**
 * @brief Run one cycle of rate control
 * This calls the Run() method of MulticopterRateControl
 */
void multicopter_rate_control_run(void);

/**
 * @brief Cleanup MulticopterRateControl instance
 */
void multicopter_rate_control_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __MULTICOPTER_RATE_CONTROL_WRAPPER_H__ */

