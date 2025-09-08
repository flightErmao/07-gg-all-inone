#ifndef SBUS_PROTO_H
#define SBUS_PROTO_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SBUS protocol constants */
#define SBUS_FRAME_SIZE        25
#define SBUS_HEADER            0x0F
#define SBUS_FOOTER            0x00

/* SBUS flags (frame[23]) */
#define SBUS_FLAG_CH17         (1 << 0)
#define SBUS_FLAG_CH18         (1 << 1)
#define SBUS_FLAG_FRAME_LOST   (1 << 2)
#define SBUS_FLAG_FAILSAFE     (1 << 3)

/* Validate a 25-byte buffer as SBUS frame */
rt_inline int sbus_proto_is_valid_frame(const uint8_t *frame, rt_size_t len)
{
    if (len != SBUS_FRAME_SIZE) return 0;
    if (frame[0] != SBUS_HEADER) return 0;
    /* Some receivers don't fix footer to 0x00, lower nibble can be 0 */
    if (!(frame[24] == SBUS_FOOTER || (frame[24] & 0x0F) == 0x00)) return 0;
    return 1;
}

/* Decode 16 x 11-bit SBUS channels (0..2047) and flags from a frame */
void sbus_proto_decode_channels(const uint8_t *frame, uint16_t ch_out[16], uint8_t *flags_out);

/* Encode channels/flags into a SBUS frame */
int sbus_proto_encode_frame(const uint16_t ch[16], bool ch17, bool ch18, uint8_t out_frame[SBUS_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_PROTO_H */


