#ifndef LED_BLINK_INTERFACE_H
#define LED_BLINK_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief LED blink interface step function
 *
 * This function executes one step of the LED blink model.
 * It should be called periodically according to the model's timing requirements.
 *
 * @param timestamp Current timestamp in milliseconds
 */
void led_blink_interface_step(uint32_t timestamp);

/**
 * @brief LED blink interface initialization
 *
 * This function initializes the LED blink model and related structures.
 * It should be called once before using the model.
 */
void led_blink_interface_init(void);

/**
 * @brief Get current LED output state
 *
 * @return true if LED should be ON, false if OFF
 */
bool led_blink_get_output(void);

/**
 * @brief Get LED blink model step count
 *
 * @return Current step count
 */
uint32_t led_blink_get_step_count(void);

/**
 * @brief Check if LED blink model is initialized
 *
 * @return true if initialized, false otherwise
 */
bool led_blink_is_initialized(void);

/**
 * @brief Get LED blink model period
 *
 * @return Model execution period in milliseconds
 */
uint32_t led_blink_get_period_ms(void);

/**
 * @brief Set LED blink model period
 *
 * @param period_ms New period in milliseconds
 */
void led_blink_set_period_ms(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif  // LED_BLINK_INTERFACE_H