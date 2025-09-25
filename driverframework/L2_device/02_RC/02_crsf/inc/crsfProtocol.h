#ifndef CRSF_PROTOCOL_H
#define CRSF_PROTOCOL_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CRSF_CHANNEL 16

/* CRSF framing */
#define CRSF_SYNC_BYTE 0xC8
/* frame: [0]=sync, [1]=len, [2]=type, [3..]=payload, [end]=crc */
#define CRSF_FRAME_TYPE_RC_CHANNELS 0x16

#define CRSF_INVALID_CHANNEL_VALUE 1500
#define CRSF_READ_TIMEOUT_MS 100

#define EVENT_CRSF_DATA_RECEIVED (1 << 0)
#define EVENT_CRSF_DATA_READY (1 << 1)

typedef struct {
  uint16_t rc_count;
  uint16_t max_channels;
  uint8_t crsf_lock;
  bool crsf_data_ready;
  struct rt_ringbuffer* crsf_rb;
  uint16_t crsf_val[MAX_CRSF_CHANNEL];
  struct rt_event* crsf_data_received_event;
  struct rt_event* crsf_data_ready_event;
  struct rt_mutex* crsf_mutex;

  /* incremental sync/parse state */
  uint8_t syncing;
  uint8_t frame_len_expected; /* length field from header */
  uint8_t frame_buf[64];      /* enough for RC frame */
  uint8_t frame_index;
} crsf_decoder_t;

rt_inline void crsf_lock(crsf_decoder_t* decoder) {
  if (decoder->crsf_mutex) {
    rt_mutex_take(decoder->crsf_mutex, RT_WAITING_FOREVER);
  }
  decoder->crsf_lock = 1;
}

rt_inline void crsf_unlock(crsf_decoder_t* decoder) {
  decoder->crsf_lock = 0;
  if (decoder->crsf_mutex) {
    rt_mutex_release(decoder->crsf_mutex);
  }
}

rt_inline uint8_t crsf_islock(crsf_decoder_t* decoder) { return decoder->crsf_lock; }

rt_inline uint8_t crsf_data_ready(crsf_decoder_t* decoder) { return decoder->crsf_data_ready; }

rt_inline void crsf_data_clear(crsf_decoder_t* decoder) { decoder->crsf_data_ready = 0; }

rt_err_t crsf_decoder_init(crsf_decoder_t* decoder);
bool crsf_parse_channels(crsf_decoder_t* decoder, const uint8_t* payload, unsigned len);
bool crsf_update(crsf_decoder_t* decoder);
uint32_t crsf_input(crsf_decoder_t* decoder, const uint8_t* values, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_PROTOCOL_H */


