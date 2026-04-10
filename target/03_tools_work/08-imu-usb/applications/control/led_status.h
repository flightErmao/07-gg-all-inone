#ifndef APPLICATIONS_LED_STATUS_H
#define APPLICATIONS_LED_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LED_STATUS_IDLE = 0,
    LED_STATUS_RUNNING,
    LED_STATUS_ERROR,
} led_status_mode_t;

int led_status_set_mode(led_status_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
