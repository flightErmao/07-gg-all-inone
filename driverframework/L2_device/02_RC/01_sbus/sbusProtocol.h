#ifndef SBUS_PROTOCOL_H
#define SBUS_PROTOCOL_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SBUS_CHANNEL 16
#define SBUS_FRAME_SIZE 25
#define SBUS_START_SYMBOL 0x0f
#define SBUS_END_SYMBOL 0x00
#define SBUS_INPUT_CHANNELS 16
#define SBUS_FLAGS_BYTE 23
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2

#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

#define SBUS_INVALID_CHANNEL_VALUE 1500
#define SBUS_READ_TIMEOUT_MS 100

#define EVENT_SBUS_DATA_RECEIVED (1 << 0)
#define EVENT_SBUS_DATA_READY (1 << 1)

struct sbus_bit_pick {
  uint8_t byte;
  uint8_t rshift;
  uint8_t mask;
  uint8_t lshift;
};

typedef struct {
  uint16_t rc_count;
  uint16_t max_channels;
  bool sbus_failsafe;
  bool sbus_frame_drop;
  uint32_t sbus_frame_drops;
  bool sbus_data_ready;
  uint8_t sbus_lock;
  struct rt_ringbuffer* sbus_rb;
  uint16_t sbus_val[MAX_SBUS_CHANNEL];
  struct rt_event* sbus_data_received_event;
  struct rt_event* sbus_data_ready_event;
  struct rt_mutex* sbus_mutex;
  /* incremental sync/parse state */
  uint8_t sbus_syncing;
  uint8_t sbus_frame_buf[SBUS_FRAME_SIZE];
  uint8_t sbus_frame_index;
} sbus_decoder_t;

rt_inline void sbus_lock(sbus_decoder_t* decoder) {
  if (decoder->sbus_mutex) {
    rt_mutex_take(decoder->sbus_mutex, RT_WAITING_FOREVER);
  }
  decoder->sbus_lock = 1;
}

rt_inline void sbus_unlock(sbus_decoder_t* decoder) {
  decoder->sbus_lock = 0;
  if (decoder->sbus_mutex) {
    rt_mutex_release(decoder->sbus_mutex);
  }
}

rt_inline uint8_t sbus_islock(sbus_decoder_t* decoder) { return decoder->sbus_lock; }

rt_inline uint8_t sbus_data_ready(sbus_decoder_t* decoder) { return decoder->sbus_data_ready; }

rt_inline void sbus_data_clear(sbus_decoder_t* decoder) { decoder->sbus_data_ready = 0; }

/* Function declarations */
rt_err_t sbus_decoder_init(sbus_decoder_t* decoder);
bool sbus_parse(sbus_decoder_t* decoder, uint8_t* frame, unsigned len);
bool sbus_update(sbus_decoder_t* decoder);
uint32_t sbus_input(sbus_decoder_t* decoder, const uint8_t* values, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_PROTOCOL_H */