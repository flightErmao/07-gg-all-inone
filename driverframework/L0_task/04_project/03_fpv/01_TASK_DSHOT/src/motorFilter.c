#include "motorFilter.h"
#include <string.h>

/**
 * @brief Initialize motor filter structure
 * @param filter Filter structure pointer
 */
void motor_filter_init(motor_filter_t* filter) {
    if (filter == NULL) {
        return;
    }
    
    /* Clear all buffers */
    memset(filter->buffer, 0, sizeof(filter->buffer));
    memset(filter->index, 0, sizeof(filter->index));
    memset(filter->count, 0, sizeof(filter->count));
    
    filter->initialized = true;
}

/**
 * @brief Apply motor data filtering
 * @param filter Filter structure pointer
 * @param input Input motor data array (4 motors)
 * @param output Output filtered data array (4 motors)
 */
void motor_filter_apply(motor_filter_t* filter, const uint16_t* input, uint16_t* output) {
    if (filter == NULL || input == NULL || output == NULL || !filter->initialized) {
        return;
    }
    
    for (int motor = 0; motor < MOTOR_FILTER_MOTOR_COUNT; motor++) {
        /* Add new data to circular buffer */
        filter->buffer[motor][filter->index[motor]] = input[motor];
        filter->index[motor] = (filter->index[motor] + 1) % MOTOR_FILTER_WINDOW_SIZE;
        
        /* Update valid data count */
        if (filter->count[motor] < MOTOR_FILTER_WINDOW_SIZE) {
            filter->count[motor]++;
        }
        
        /* Calculate moving average */
        uint32_t sum = 0;
        for (int i = 0; i < filter->count[motor]; i++) {
            sum += filter->buffer[motor][i];
        }
        
        /* Output average value */
        output[motor] = (uint16_t)(sum / filter->count[motor]);
    }
}

/**
 * @brief Reset filter state
 * @param filter Filter structure pointer
 */
void motor_filter_reset(motor_filter_t* filter) {
    if (filter == NULL) {
        return;
    }
    
    /* Clear all buffers */
    memset(filter->buffer, 0, sizeof(filter->buffer));
    memset(filter->index, 0, sizeof(filter->index));
    memset(filter->count, 0, sizeof(filter->count));
    
    filter->initialized = true;
}
