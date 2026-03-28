#ifndef APPLICATIONS_LED_STATUS_H
#define APPLICATIONS_LED_STATUS_H

typedef enum
{
    LED_STATUS_IDLE = 0,
    LED_STATUS_RUNNING,
    LED_STATUS_ERROR,
} led_status_mode_t;

int led_status_set_mode(led_status_mode_t mode);

#endif
