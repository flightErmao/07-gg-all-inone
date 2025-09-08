#include "sbus_proto.h"

static int16_t sbus_proto_parse_ch(const uint8_t *buf, uint8_t index)
{
    const uint8_t *p = buf + 1; /* skip header */
    uint32_t v = 0;
    uint8_t bit_offset = (index * 11) % 8;
    uint32_t byte_index = (index * 11) / 8;
    v = p[byte_index] | ((uint32_t)p[byte_index + 1] << 8) | ((uint32_t)p[byte_index + 2] << 16);
    v >>= bit_offset;
    return (int16_t)(v & 0x07FF); /* 0..2047 */
}

void sbus_proto_decode_channels(const uint8_t *frame, uint16_t ch_out[16], uint8_t *flags_out)
{
    for (int i = 0; i < 16; i++)
    {
        ch_out[i] = (uint16_t)sbus_proto_parse_ch(frame, (uint8_t)i);
    }
    if (flags_out) *flags_out = frame[23];
}

int sbus_proto_encode_frame(const uint16_t ch[16], bool ch17, bool ch18, uint8_t out_frame[SBUS_FRAME_SIZE])
{
    if (!out_frame) return -1;

    for (int i = 0; i < SBUS_FRAME_SIZE; i++) out_frame[i] = 0;
    out_frame[0] = SBUS_HEADER;

    uint32_t bit_pos = 0;
    for (int i = 0; i < 16; i++)
    {
        uint32_t val = ch[i] & 0x07FF;
        uint32_t byte_index = 1 + (bit_pos >> 3);
        uint8_t bit_offset = bit_pos & 0x07;
        uint32_t current = val << bit_offset;
        out_frame[byte_index]     |= current & 0xFF;
        out_frame[byte_index + 1] |= (current >> 8) & 0xFF;
        out_frame[byte_index + 2] |= (current >> 16) & 0xFF;
        bit_pos += 11;
    }

    uint8_t flags = 0;
    if (ch17) flags |= SBUS_FLAG_CH17;
    if (ch18) flags |= SBUS_FLAG_CH18;
    out_frame[23] = flags;
    out_frame[24] = SBUS_FOOTER;

    return SBUS_FRAME_SIZE;
}


