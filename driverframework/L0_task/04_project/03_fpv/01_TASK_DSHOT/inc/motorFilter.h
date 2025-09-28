#ifndef MOTOR_FILTER_H
#define MOTOR_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* Filter configuration parameters */
#define MOTOR_FILTER_WINDOW_SIZE 4  /* Moving average window size, balance filter effect and delay */
#define MOTOR_FILTER_MOTOR_COUNT 4  /* Motor count */

/* Filter structure */
typedef struct {
    uint16_t buffer[MOTOR_FILTER_MOTOR_COUNT][MOTOR_FILTER_WINDOW_SIZE];  /* Historical data buffer for each motor */
    uint8_t index[MOTOR_FILTER_MOTOR_COUNT];                              /* Current write position index */
    uint8_t count[MOTOR_FILTER_MOTOR_COUNT];                              /* Current valid data count */
    bool initialized;                                                      /* Initialization flag */
} motor_filter_t;

/* Filter function interface */
void motor_filter_init(motor_filter_t* filter);
void motor_filter_apply(motor_filter_t* filter, const uint16_t* input, uint16_t* output);
void motor_filter_reset(motor_filter_t* filter);

#endif /* MOTOR_FILTER_H */
