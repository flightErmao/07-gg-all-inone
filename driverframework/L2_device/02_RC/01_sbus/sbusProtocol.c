#include "sbusProtocol.h"

static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
    /*  0 */ {{0, 0, 0xff, 0}, {1, 0, 0x07, 8}, {0, 0, 0x00, 0}},
    /*  1 */ {{1, 3, 0x1f, 0}, {2, 0, 0x3f, 5}, {0, 0, 0x00, 0}},
    /*  2 */ {{2, 6, 0x03, 0}, {3, 0, 0xff, 2}, {4, 0, 0x01, 10}},
    /*  3 */ {{4, 1, 0x7f, 0}, {5, 0, 0x0f, 7}, {0, 0, 0x00, 0}},
    /*  4 */ {{5, 4, 0x0f, 0}, {6, 0, 0x7f, 4}, {0, 0, 0x00, 0}},
    /*  5 */ {{6, 7, 0x01, 0}, {7, 0, 0xff, 1}, {8, 0, 0x03, 9}},
    /*  6 */ {{8, 2, 0x3f, 0}, {9, 0, 0x1f, 6}, {0, 0, 0x00, 0}},
    /*  7 */ {{9, 5, 0x07, 0}, {10, 0, 0xff, 3}, {0, 0, 0x00, 0}},
    /*  8 */ {{11, 0, 0xff, 0}, {12, 0, 0x07, 8}, {0, 0, 0x00, 0}},
    /*  9 */ {{12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, {0, 0, 0x00, 0}},
    /* 10 */ {{13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10}},
    /* 11 */ {{15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, {0, 0, 0x00, 0}},
    /* 12 */ {{16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, {0, 0, 0x00, 0}},
    /* 13 */ {{17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03, 9}},
    /* 14 */ {{19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, {0, 0, 0x00, 0}},
    /* 15 */ {{20, 5, 0x07, 0}, {21, 0, 0xff, 3}, {0, 0, 0x00, 0}}};

rt_err_t sbus_decoder_init(sbus_decoder_t* decoder) {
  if (decoder == NULL) {
    return RT_EINVAL;
  }

  rt_memset(decoder, 0, sizeof(sbus_decoder_t));

  decoder->max_channels = MAX_SBUS_CHANNEL;

  static uint8_t sbus_rb_pool[SBUS_FRAME_SIZE * 4];
  decoder->sbus_rb = rt_malloc(sizeof(struct rt_ringbuffer));
  if (decoder->sbus_rb == NULL) {
    rt_kprintf("[SBUS] malloc ringbuffer failed\n");
    return RT_ENOMEM;
  }
  rt_ringbuffer_init(decoder->sbus_rb, sbus_rb_pool, sizeof(sbus_rb_pool));

  static struct rt_event event_data_received;
  static struct rt_event event_data_ready;
  rt_event_init(&event_data_received, "sbus_rx", RT_IPC_FLAG_FIFO);
  rt_event_init(&event_data_ready, "sbus_rdy", RT_IPC_FLAG_FIFO);
  decoder->sbus_data_received_event = &event_data_received;
  decoder->sbus_data_ready_event = &event_data_ready;

  static struct rt_mutex sbus_mutex;
  rt_mutex_init(&sbus_mutex, "sbus_mtx", RT_IPC_FLAG_FIFO);
  decoder->sbus_mutex = &sbus_mutex;

  return RT_EOK;
}

bool sbus_parse(sbus_decoder_t* decoder, uint8_t* frame, unsigned len) {
  if (len != SBUS_FRAME_SIZE) {
    decoder->sbus_frame_drops++;
    return false;
  }

  if (frame[0] != SBUS_START_SYMBOL) {
    decoder->sbus_frame_drops++;
    return false;
  }

  /* optional end symbol check for better integrity */
  if (frame[SBUS_FRAME_SIZE - 1] != SBUS_END_SYMBOL) {
    decoder->sbus_frame_drops++;
    return false;
  }

  unsigned chancount = (decoder->max_channels > SBUS_INPUT_CHANNELS) ? SBUS_INPUT_CHANNELS : decoder->max_channels;

  for (unsigned channel = 0; channel < chancount; channel++) {
    unsigned value = 0;

    for (unsigned pick = 0; pick < 3; pick++) {
      const struct sbus_bit_pick* decode = &sbus_decoder[channel][pick];

      if (decode->mask != 0) {
        unsigned piece = frame[1 + decode->byte];
        piece >>= decode->rshift;
        piece &= decode->mask;
        piece <<= decode->lshift;

        value |= piece;
      }
    }

    uint16_t scaled = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    /* drop frame if any value out of expected range */
    if (scaled < (uint16_t)SBUS_TARGET_MIN || scaled > (uint16_t)SBUS_TARGET_MAX) {
      decoder->sbus_frame_drops++;
      return false;
    }
    decoder->sbus_val[channel] = scaled;
  }

  if (decoder->max_channels > 17 && chancount > 15) {
    chancount = 18;
    decoder->sbus_val[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
    decoder->sbus_val[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
  }

  decoder->rc_count = chancount;
  decoder->sbus_failsafe = (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT));
  decoder->sbus_frame_drop = (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT));

  return true;
}

bool sbus_update(sbus_decoder_t* decoder) {
  /* Incrementally consume bytes to find and assemble a full frame */
  uint8_t byte;
  int got;

  while (rt_ringbuffer_data_len(decoder->sbus_rb) > 0) {
    got = rt_ringbuffer_get(decoder->sbus_rb, &byte, 1);
    if (got != 1) {
      break;
    }

    if (!decoder->sbus_syncing) {
      if (byte == SBUS_START_SYMBOL) {
        decoder->sbus_syncing = 1;
        decoder->sbus_frame_index = 0;
        decoder->sbus_frame_buf[decoder->sbus_frame_index++] = byte;
      }
      /* else: discard until we see start symbol */
      continue;
    }

    /* syncing: accumulate bytes */
    decoder->sbus_frame_buf[decoder->sbus_frame_index++] = byte;

    if (decoder->sbus_frame_index >= SBUS_FRAME_SIZE) {
      /* full frame collected, attempt parse */
      bool parsed_ok = sbus_parse(decoder, decoder->sbus_frame_buf, SBUS_FRAME_SIZE);
      /* reset sync for next frame regardless of success */
      decoder->sbus_syncing = 0;
      decoder->sbus_frame_index = 0;

      if (parsed_ok) {
        bool ready = !decoder->sbus_failsafe && !decoder->sbus_frame_drop;
        decoder->sbus_data_ready = ready;
        if (ready && decoder->sbus_data_ready_event != NULL) {
          rt_event_send(decoder->sbus_data_ready_event, EVENT_SBUS_DATA_READY);
        }
        return ready;
      }
      /* if parse failed, continue scanning from next byte already in ringbuffer */
    }
  }

  return false;
}

uint32_t sbus_input(sbus_decoder_t* decoder, const uint8_t* values, uint32_t size) {
  return rt_ringbuffer_put(decoder->sbus_rb, values, size);
}
