#include "crsfProtocol.h"

static uint8_t crsf_rb_pool[64 * 4];

rt_err_t crsf_decoder_init(crsf_decoder_t* decoder) {
  if (decoder == RT_NULL) {
    return RT_EINVAL;
  }

  rt_memset(decoder, 0, sizeof(crsf_decoder_t));
  decoder->max_channels = MAX_CRSF_CHANNEL;

  decoder->crsf_rb = rt_malloc(sizeof(struct rt_ringbuffer));
  if (decoder->crsf_rb == RT_NULL) {
    rt_kprintf("[CRSF] malloc ringbuffer failed\n");
    return RT_ENOMEM;
  }
  rt_ringbuffer_init(decoder->crsf_rb, crsf_rb_pool, sizeof(crsf_rb_pool));

  static struct rt_event event_data_received;
  static struct rt_event event_data_ready;
  rt_event_init(&event_data_received, "crsf_rx", RT_IPC_FLAG_FIFO);
  rt_event_init(&event_data_ready, "crsf_rdy", RT_IPC_FLAG_FIFO);
  decoder->crsf_data_received_event = &event_data_received;
  decoder->crsf_data_ready_event = &event_data_ready;

  static struct rt_mutex crsf_mutex;
  rt_mutex_init(&crsf_mutex, "crsf_mtx", RT_IPC_FLAG_FIFO);
  decoder->crsf_mutex = &crsf_mutex;

  return RT_EOK;
}

/* simple checksum per CRSF v3: sum over [type..payload] */
static uint8_t crsf_crc_sum(const uint8_t* data, uint8_t len) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < len; i++) s += data[i];
  return s;
}

bool crsf_parse_channels(crsf_decoder_t* decoder, const uint8_t* payload, unsigned len) {
  if (len < 22) {
    return false;
  }

  uint16_t ch[16] = {0};
  uint32_t bitpos = 0;
  for (int i = 0; i < 16; i++) {
    uint32_t byte_index = bitpos >> 3;
    uint8_t bit_offset = bitpos & 0x07;
    uint32_t v = payload[byte_index] | ((uint32_t)payload[byte_index + 1] << 8) |
                 ((uint32_t)payload[byte_index + 2] << 16);
    v >>= bit_offset;
    ch[i] = (uint16_t)(v & 0x07FF);
    bitpos += 11;
  }

  crsf_lock(decoder);
  for (int i = 0; i < 16 && i < decoder->max_channels; i++) {
    /* Map 172..1811 to 1000..2000 roughly */
    float scaled = 1000.0f + (ch[i] - 172) * (1000.0f / 1639.0f);
    if (scaled < 1000.0f) scaled = 1000.0f;
    if (scaled > 2000.0f) scaled = 2000.0f;
    decoder->crsf_val[i] = (uint16_t)scaled;
  }
  decoder->rc_count = 16;
  decoder->crsf_data_ready = true;
  if (decoder->crsf_data_ready_event) {
    rt_event_send(decoder->crsf_data_ready_event, EVENT_CRSF_DATA_READY);
  }
  crsf_unlock(decoder);

  return true;
}

bool crsf_update(crsf_decoder_t* decoder) {
  uint8_t byte;
  int got;

  while (rt_ringbuffer_data_len(decoder->crsf_rb) > 0) {
    got = rt_ringbuffer_get(decoder->crsf_rb, &byte, 1);
    if (got != 1) {
      break;
    }

    if (!decoder->syncing) {
      if (byte == CRSF_SYNC_BYTE) {
        decoder->syncing = 1;
        decoder->frame_index = 0;
        decoder->frame_buf[decoder->frame_index++] = byte;
      }
      continue;
    }

    decoder->frame_buf[decoder->frame_index++] = byte;

    if (decoder->frame_index == 2) {
      decoder->frame_len_expected = byte;
      if (decoder->frame_len_expected > sizeof(decoder->frame_buf) - 2) {
        decoder->syncing = 0;
        decoder->frame_index = 0;
      }
    }

    if (decoder->frame_index >= 3) {
      uint8_t total_need = (uint8_t)(2 + decoder->frame_len_expected);
      if (decoder->frame_index >= total_need) {
        uint8_t type = decoder->frame_buf[2];
        uint8_t crc_rx = decoder->frame_buf[total_need - 1];
        uint8_t crc_calc = crsf_crc_sum(&decoder->frame_buf[2], decoder->frame_len_expected - 1);

        if (crc_calc == crc_rx) {
          if (type == CRSF_FRAME_TYPE_RC_CHANNELS) {
            crsf_parse_channels(decoder, &decoder->frame_buf[3], decoder->frame_len_expected - 2);
          }
        }

        decoder->syncing = 0;
        decoder->frame_index = 0;
        return true;
      }
      if (decoder->frame_index >= sizeof(decoder->frame_buf)) {
        decoder->syncing = 0;
        decoder->frame_index = 0;
      }
    }
  }

  return false;
}

uint32_t crsf_input(crsf_decoder_t* decoder, const uint8_t* values, uint32_t size) {
  return rt_ringbuffer_put(decoder->crsf_rb, values, size);
}


