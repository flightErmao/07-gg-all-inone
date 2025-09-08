#ifndef SBUS_H
#define SBUS_H

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
} sbus_rc_data_t;

/* Initialize SBUS on configured UART and start reception */
int sbus_init(void);

/* Get latest parsed RC data (thread-safe snapshot) */
sbus_rc_data_t sbus_get_data(void);

/* Blocking read with timeout (ms). Returns RT_EOK on fresh data, <0 on timeout/error */
int sbus_read(sbus_rc_data_t* out, int timeout_ms);

/* Optional: encode and send SBUS frame (for passthrough or test) */
int sbus_send_channels(const uint16_t ch[16], bool ch17, bool ch18);

#endif /* SBUS_H */


