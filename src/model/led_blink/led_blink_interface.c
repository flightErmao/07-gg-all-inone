/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Include LED blink model header */
#include "lib/LED_blink.h"

/* LED blink model handler structure */
static struct LED_Blink_Handler {
  bool led_output;     /* Current LED output state */
  uint32_t step_count; /* Step counter for timing */
  bool initialized;    /* Initialization flag */
} led_blink_handle;

/* LED blink model info */
static struct {
  uint32_t period_ms;                       /* Model execution period in milliseconds */
  const char* model_info;                   /* Model information string */
} led_blink_model_info = {.period_ms = 500, /* Default 500ms period */
                          .model_info = "LED Blink Simulink Model v1.2"};

/**
 * @brief LED blink interface step function
 *
 * This function executes one step of the LED blink model.
 * It should be called periodically according to the model's timing requirements.
 *
 * @param timestamp Current timestamp in milliseconds
 */
void led_blink_interface_step(uint32_t timestamp) {
  /* Execute one step of the LED blink model */
  LED_blink_step();

  /* Update handler with current output state */
  led_blink_handle.led_output = LED_blink_Y.LED_gg;
  led_blink_handle.step_count++;

/* Print debug information if needed */
#ifdef DEBUG
  printf("[LED_BLINK] Step %u: LED output = %s\n", led_blink_handle.step_count,
         led_blink_handle.led_output ? "ON" : "OFF");
#endif
}

/**
 * @brief LED blink interface initialization
 *
 * This function initializes the LED blink model and related structures.
 * It should be called once before using the model.
 */
void led_blink_interface_init(void) {
  /* Initialize LED blink model */
  LED_blink_initialize();

  /* Initialize handler structure */
  led_blink_handle.led_output = false;
  led_blink_handle.step_count = 0;
  led_blink_handle.initialized = true;

  /* Print initialization information */
  printf("[LED_BLINK] Initialized successfully\n");
  printf("[LED_BLINK] Model info: %s\n", led_blink_model_info.model_info);
  printf("[LED_BLINK] Period: %u ms\n", led_blink_model_info.period_ms);
}

/**
 * @brief Get current LED output state
 *
 * @return true if LED should be ON, false if OFF
 */
bool led_blink_get_output(void) { return led_blink_handle.led_output; }

/**
 * @brief Get LED blink model step count
 *
 * @return Current step count
 */
uint32_t led_blink_get_step_count(void) { return led_blink_handle.step_count; }

/**
 * @brief Check if LED blink model is initialized
 *
 * @return true if initialized, false otherwise
 */
bool led_blink_is_initialized(void) { return led_blink_handle.initialized; }

/**
 * @brief Get LED blink model period
 *
 * @return Model execution period in milliseconds
 */
uint32_t led_blink_get_period_ms(void) { return led_blink_model_info.period_ms; }

/**
 * @brief Set LED blink model period
 *
 * @param period_ms New period in milliseconds
 */
void led_blink_set_period_ms(uint32_t period_ms) {
  led_blink_model_info.period_ms = period_ms;
  printf("[LED_BLINK] Period updated to %u ms\n", period_ms);
}