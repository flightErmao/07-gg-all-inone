#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t timestamp;
    float roll;
    float pitch;
    float yaw;
    float thrust;
    bool arm_status;
    bool ctrl_mode; /* 0: angle, 1: rate (placeholder) */
} crsf_rc_data_t;

int crsf_init(void);
crsf_rc_data_t crsf_get_data(void);

#endif /* CRSF_H */


