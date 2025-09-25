/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <rtthread.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <dfs_posix.h>

#include "fileManager.h"
#include "mlog.h"

#define TAG "MLog"

#define MLOG_BUFFER_SIZE 64 * 1024
#define MLOG_SECTOR_SIZE 4096
#define MLOG_MAX_SECTOR_TO_WRITE 5

/* Define FMT error codes as RT error codes */
#define FMT_ERROR -RT_ERROR
#define FMT_EOK RT_EOK
#define FMT_EINVAL -RT_EINVAL
#define FMT_ENOMEM -RT_ENOMEM
#define FMT_EFULL -RT_EFULL
#define FMT_EBUSY -RT_EBUSY
#define FMT_EEMPTY -RT_EEMPTY

/* Use definitions from mlog.h instead of redefining */

/* Define missing functions */
#define param_get_group_count() 0
#define param_get_table() NULL
#define systime_now_ms() rt_tick_get_millisecond()
#define ulog_w(tag, fmt, ...) rt_kprintf("[W/%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ulog_e(tag, fmt, ...) rt_kprintf("[E/%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ulog_i(tag, fmt, ...) rt_kprintf("[I/%s] " fmt "\n", tag, ##__VA_ARGS__)
#define console_printf(fmt, ...) rt_kprintf(fmt, ##__VA_ARGS__)
#define console_write(data, len) rt_kprintf("%.*s", len, data)

/* Define parameter types */
#define PARAM_TYPE_INT8 1
#define PARAM_TYPE_UINT8 2
#define PARAM_TYPE_INT16 3
#define PARAM_TYPE_UINT16 4
#define PARAM_TYPE_INT32 5
#define PARAM_TYPE_UINT32 6
#define PARAM_TYPE_FLOAT 7
#define PARAM_TYPE_DOUBLE 8

/* Model info disabled by default in mlog.h; only define when enabled */
#if MLOG_ENABLE_MODEL_INFO
typedef struct {
  char info[64];
} model_info_t;

#ifndef FMT_USING_SIH
#define FMT_USING_SIH 0
#endif

static model_info_t ins_model_info = {"INS Model"};
static model_info_t fms_model_info = {"FMS Model"};
static model_info_t control_model_info = {"Control Model"};
#if FMT_USING_SIH
static model_info_t plant_model_info = {"Plant Model"};
#endif
#endif /* MLOG_ENABLE_MODEL_INFO */
#define PERIOD_EXECUTE(name, period, code)                  \
  do {                                                      \
    static uint32_t name##_last = 0;                        \
    if (rt_tick_get_millisecond() - name##_last > period) { \
      name##_last = rt_tick_get_millisecond();              \
      code;                                                 \
    }                                                       \
  } while (0)

/* Define critical section macros */
#define OS_ENTER_CRITICAL rt_hw_interrupt_disable()
#define OS_EXIT_CRITICAL rt_hw_interrupt_enable(level)

/* Define list macros */
struct list_head {
  struct list_head *next, *prev;
};

#define LIST_HEAD_INIT(name) {&(name), &(name)}
#define LIST_HEAD(name) struct list_head name = LIST_HEAD_INIT(name)
#define INIT_LIST_HEAD(ptr) \
  do {                      \
    (ptr)->next = (ptr);    \
    (ptr)->prev = (ptr);    \
  } while (0)

#define list_add_tail(new, head) \
  do {                           \
    (new)->next = (head);        \
    (new)->prev = (head)->prev;  \
    (head)->prev->next = (new);  \
    (head)->prev = (new);        \
  } while (0)

#define list_del(entry)                  \
  do {                                   \
    (entry)->prev->next = (entry)->next; \
    (entry)->next->prev = (entry)->prev; \
  } while (0)

/* Correct container/list helpers */
#ifndef container_of
#define container_of(ptr, type, member) ((type*)((char*)(ptr) - offsetof(type, member)))
#endif
#define list_entry(ptr, type, member) container_of(ptr, type, member)
#define list_for_each_entry(pos, type, head, member)                         \
  for (pos = list_entry((head)->next, type, member); &pos->member != (head); \
       pos = list_entry(pos->member.next, type, member))

#define WRITE_PAYLOAD(_payload, _len) write(mlog_handle.fid, _payload, _len);

struct mlog_cb {
  void (*func)(void);
  struct list_head link;
};

typedef struct {
  uint32_t total_msg;
  uint32_t lost_msg;
} mlog_stats_t;

struct fmt_mlog {
  int fid;
  uint8_t is_open;
  char file_name[50];
  uint8_t log_status;
  mlog_header_t header;
  mlog_buffer_t buffer;
  struct rt_mutex lock;
  mlog_stats_t* stats;
};

extern const int __fmt_mlog_start;
extern const int __fmt_mlog_end;

static uint8_t __mlog_data_buffer[MLOG_BUFFER_SIZE];
static mlog_bus_t* __mlog_table;
static uint8_t __mlog_bus_num;
static struct fmt_mlog mlog_handle;
static LIST_HEAD(__start_cb_list_head);
static LIST_HEAD(__stop_cb_list_head);
static LIST_HEAD(__update_cb_list_head);

static void invoke_callback_func(mlog_cb_type type) {
  struct mlog_cb* pos;

  if (type == MLOG_CB_START) {
    list_for_each_entry(pos, struct mlog_cb, &__start_cb_list_head, link) {
      /* invoke registered callback function */
      pos->func();
    }
  } else if (type == MLOG_CB_STOP) {
    list_for_each_entry(pos, struct mlog_cb, &__stop_cb_list_head, link) {
      /* invoke registered callback function */
      pos->func();
    }
  } else if (type == MLOG_CB_UPDATE) {
    list_for_each_entry(pos, struct mlog_cb, &__update_cb_list_head, link) {
      /* invoke registered callback function */
      pos->func();
    }
  }
}

static void buffer_putc(uint8_t ch) {
  uint32_t free_space_in_sector = MLOG_SECTOR_SIZE - mlog_handle.buffer.index;

  if (free_space_in_sector < 1) {
    // move head point to next sector
    mlog_handle.buffer.head = (mlog_handle.buffer.head + 1) % mlog_handle.buffer.num_sector;
    mlog_handle.buffer.index = 0;

    /* we have a new sector data, inform callback functions */
    invoke_callback_func(MLOG_CB_UPDATE);
  }

  mlog_handle.buffer.data[mlog_handle.buffer.head * MLOG_SECTOR_SIZE + mlog_handle.buffer.index] = ch;
  mlog_handle.buffer.index += 1;
}

static void buffer_write(const uint8_t* data, uint16_t len) {
  uint32_t free_space_in_sector = MLOG_SECTOR_SIZE - mlog_handle.buffer.index;

  // TODO: add support with len larger than MLOG_SECTOR_SIZE

  if (free_space_in_sector < len) {
    memcpy(&mlog_handle.buffer.data[mlog_handle.buffer.head * MLOG_SECTOR_SIZE + mlog_handle.buffer.index], data,
           free_space_in_sector);

    // move head point to next sector
    mlog_handle.buffer.head = (mlog_handle.buffer.head + 1) % mlog_handle.buffer.num_sector;
    mlog_handle.buffer.index = 0;

    memcpy(&mlog_handle.buffer.data[mlog_handle.buffer.head * MLOG_SECTOR_SIZE + mlog_handle.buffer.index],
           &data[free_space_in_sector], len - free_space_in_sector);
    mlog_handle.buffer.index += len - free_space_in_sector;

    /* we have a new sector data, inform callback functions */
    invoke_callback_func(MLOG_CB_UPDATE);
  } else {
    memcpy(&mlog_handle.buffer.data[mlog_handle.buffer.head * MLOG_SECTOR_SIZE + mlog_handle.buffer.index], data, len);
    mlog_handle.buffer.index += len;
  }
}

static uint16_t get_max_write_sector(uint32_t head_p, uint32_t tail_p) {
  uint16_t sector_to_end;
  uint16_t sector_in_buffer;
  uint16_t sector_to_write;

  sector_to_end = mlog_handle.buffer.num_sector - tail_p;

  if (head_p >= tail_p) {
    sector_in_buffer = head_p - tail_p;
  } else {
    sector_in_buffer = sector_to_end + head_p;
  }

  sector_to_write = sector_in_buffer < sector_to_end ? sector_in_buffer : sector_to_end;

  return sector_to_write <= MLOG_MAX_SECTOR_TO_WRITE ? sector_to_write : MLOG_MAX_SECTOR_TO_WRITE;
}

static bool buffer_is_full(uint32_t len_to_write) {
  uint32_t free_space_in_sector = MLOG_SECTOR_SIZE - mlog_handle.buffer.index;

  // TODO: check if write multiple sectors at once

  /* check if buffer is full */
  if (free_space_in_sector < len_to_write) {
    if ((mlog_handle.buffer.head + 1) % mlog_handle.buffer.num_sector == mlog_handle.buffer.tail) {
      return true;
    }
  }

  return false;
}

/**
 * Get current logging status
 *
 * @return mlog status: MLOG_STATUS_IDLE | MLOG_STATUS_WRITE_HEAD | MLOG_STATUS_LOGGING | MLOG_STATUS_STOPPING
 */
uint8_t mlog_get_status(void) { return mlog_handle.log_status; }

/**
 * Get current logging file name
 *
 * @return mlog logging file name
 */
char* mlog_get_file_name(void) { return mlog_handle.file_name; }

/**
 * Show the mlog logging statistics
 *
 */
void mlog_show_statistic(void) {
  for (int i = 0; i < __mlog_bus_num; i++) {
    rt_kprintf("%-20s id:%-3d record:%-8ld lost:%-5ld\n", __mlog_table[i].name, i, mlog_handle.stats[i].total_msg,
               mlog_handle.stats[i].lost_msg);
  }
}

/**
 * Register mlog callback function
 *
 * @param cb_type MLOG_CB_START | MLOG_CB_STOP | MLOG_CB_UPDATE
 * @param cb callback function
 *
 * @return FMT Error
 */
rt_err_t mlog_register_callback(mlog_cb_type type, void (*cb_func)(void)) {
  struct mlog_cb* node;

  if (cb_func == NULL) {
    return FMT_EINVAL;
  }

  node = (struct mlog_cb*)rt_malloc(sizeof(struct mlog_cb));
  if (node == NULL) {
    return FMT_ENOMEM;
  }

  INIT_LIST_HEAD(&node->link);
  node->func = cb_func;

  if (type == MLOG_CB_START) {
    list_add_tail(&node->link, &__start_cb_list_head);
  } else if (type == MLOG_CB_STOP) {
    list_add_tail(&node->link, &__stop_cb_list_head);
  } else if (type == MLOG_CB_UPDATE) {
    list_add_tail(&node->link, &__update_cb_list_head);
  }

  return FMT_EOK;
}

/**
 * @brief Deregister mlog callback function
 *
 * @param type MLOG_CB_START | MLOG_CB_STOP | MLOG_CB_UPDATE
 * @param cb_func callback function
 * @return fmt_err_t
 */
rt_err_t mlog_deregister_callback(mlog_cb_type type, void (*cb_func)(void)) {
  struct mlog_cb* pos;

  if (cb_func == NULL) {
    return FMT_EINVAL;
  }

  if (type == MLOG_CB_START) {
    list_for_each_entry(pos, struct mlog_cb, &__start_cb_list_head, link) {
      if (pos->func == cb_func) {
        list_del(&pos->link);
        rt_free(pos);
        return FMT_EOK;
      }
    }
  } else if (type == MLOG_CB_STOP) {
    list_for_each_entry(pos, struct mlog_cb, &__stop_cb_list_head, link) {
      if (pos->func == cb_func) {
        list_del(&pos->link);
        rt_free(pos);
        return FMT_EOK;
      }
    }
  } else if (type == MLOG_CB_UPDATE) {
    list_for_each_entry(pos, struct mlog_cb, &__update_cb_list_head, link) {
      if (pos->func == cb_func) {
        list_del(&pos->link);
        rt_free(pos);
        return FMT_EOK;
      }
    }
  }

  return FMT_ERROR;
}

/**
 * @brief Return bus id based on bus_name
 *
 * @param bus_name The bus name
 * @return int Finded bus id, -1 means not found
 */
int mlog_get_bus_id(const char* bus_name) {
  /* this function may be called before mlog_init */
  mlog_bus_t* mlog_table = (mlog_bus_t*)&__fmt_mlog_start;
  uint8_t mlog_bus_num = (mlog_bus_t*)&__fmt_mlog_end - mlog_table;

  for (uint8_t n = 0; n < mlog_bus_num; n++) {
    if (strcmp(bus_name, mlog_table[n].name) == 0) {
      /* the bus id is the same as index */
      return n;
    }
  }

  return -1;
}

/**
 * Add log description into mlog header
 *
 * @param desc description text, should not longer than MLOG_DESCRIPTION_SIZE
 * @return FMT Error
 */
rt_err_t mlog_add_desc(char* desc) {
#if MLOG_ENABLE_DESCRIPTION
  if (strlen(desc) > MLOG_DESCRIPTION_SIZE - 1) {
    ulog_w(TAG, "description too long.");
    return FMT_ENOMEM;
  }
  strcpy(mlog_handle.header.description, desc);
  return FMT_EOK;
#else
  (void)desc;
  return FMT_EOK;
#endif
}

/**
 * Push a mlog message into buffer
 *
 * @param payload msg payload
 * @param msg_id msg id
 * @param len msg length
 *
 * @return FMT Error
 */
rt_err_t mlog_push_msg(const uint8_t* payload, uint8_t msg_id, uint16_t len) {
  /*                           MLOG MSG Format                                 */
  /*   ======================================================================= */
  /*   | MLOG_BEGIN_MSG1 | MLOG_BEGIN_MSG2 | MSG_ID | PAYLOAD | MLOG_END_MSG | */
  /*   ======================================================================= */

  if (msg_id + 1 > __mlog_bus_num) {
    /* invalid msg id */
    return FMT_EINVAL;
  }

  /* check log status */
  if (mlog_handle.log_status != MLOG_STATUS_LOGGING) {
    return FMT_EEMPTY;
  }

  /* check if buffer has enough space to store msg */
  if (buffer_is_full(len + 4)) {
    /* do not let it print too fast */
    PERIOD_EXECUTE(mlog_buff_full, 1000, ulog_w(TAG, "buffer is full!"););

    mlog_handle.stats[msg_id].lost_msg += 1;

    return FMT_EFULL;
  }

  rt_mutex_take(&mlog_handle.lock, RT_WAITING_FOREVER);

  /* write msg begin flag */
  buffer_putc(MLOG_BEGIN_MSG1);
  buffer_putc(MLOG_BEGIN_MSG2);
  /* write msg id */
  buffer_putc(msg_id);
  /* write payload */
  buffer_write(payload, len);
  /* write msg end flag */
  buffer_putc(MLOG_END_MSG);

  rt_mutex_release(&mlog_handle.lock);

  mlog_handle.stats[msg_id].total_msg += 1;

  return FMT_EOK;
}

/**
 * Call this function to start the binary log
 *
 * @param file_name mlog_handle file name with full path
 * @return FMT Error
 */
rt_err_t mlog_start(char* file_name) {
  if (mlog_handle.log_status != MLOG_STATUS_IDLE) {
    ulog_w(TAG, "%s is logging, stop it first", mlog_handle.file_name);
    return FMT_EBUSY;
  }

  /*********************** create log file ***********************/
  mlog_handle.fid = open(file_name, O_CREAT | O_WRONLY);

  if (mlog_handle.fid < 0) {
    ulog_e(TAG, "%s open fail", file_name);
    return FMT_ERROR;
  }
  /* set log file open flag */
  mlog_handle.is_open = 1;
  /* get current time stamp */
  mlog_handle.header.timestamp = systime_now_ms();

  /*********************** init log buffer ***********************/
  mlog_handle.buffer.head = mlog_handle.buffer.tail = 0;
  mlog_handle.buffer.index = 0;

  /*********************** write log header ***********************/
  mlog_handle.log_status = MLOG_STATUS_WRITE_HEAD;

  /* write log info */
  WRITE_PAYLOAD(&mlog_handle.header.version, sizeof(mlog_handle.header.version));
  WRITE_PAYLOAD(&mlog_handle.header.timestamp, sizeof(mlog_handle.header.timestamp));
  WRITE_PAYLOAD(&mlog_handle.header.max_name_len, sizeof(mlog_handle.header.max_name_len));
  WRITE_PAYLOAD(&mlog_handle.header.max_desc_len, sizeof(mlog_handle.header.max_desc_len));
  WRITE_PAYLOAD(&mlog_handle.header.max_model_info_len, sizeof(mlog_handle.header.max_model_info_len));
#if MLOG_ENABLE_DESCRIPTION
  WRITE_PAYLOAD(mlog_handle.header.description, MLOG_DESCRIPTION_SIZE);
  /* clear the description after it has been written */
  memset(mlog_handle.header.description, 0, MLOG_DESCRIPTION_SIZE);
#endif

  /* write model information */
#if MLOG_ENABLE_MODEL_INFO
#ifdef FMT_USING_SIH
  sprintf(mlog_handle.header.model_info, "%s\n%s\n%s\n%s", ins_model_info.info, fms_model_info.info,
          control_model_info.info, plant_model_info.info);
#else
  sprintf(mlog_handle.header.model_info, "%s\n%s\n%s", ins_model_info.info, fms_model_info.info,
          control_model_info.info);
#endif
  WRITE_PAYLOAD(mlog_handle.header.model_info, MLOG_MODEL_INFO_SIZE);
#endif

  /* write bus information */
  WRITE_PAYLOAD(&mlog_handle.header.num_bus, sizeof(mlog_handle.header.num_bus));
  for (uint8_t n = 0; n < mlog_handle.header.num_bus; n++) {
    uint8_t msg_id = n;

    /* write bus list */
    WRITE_PAYLOAD(mlog_handle.header.bus_list[n].name, MLOG_MAX_NAME_LEN);
    WRITE_PAYLOAD(&msg_id, sizeof(msg_id));
    WRITE_PAYLOAD(&mlog_handle.header.bus_list[n].num_elem, sizeof(mlog_handle.header.bus_list[n].num_elem));
    /* write bus element */
    for (int k = 0; k < mlog_handle.header.bus_list[n].num_elem; k++) {
      WRITE_PAYLOAD(mlog_handle.header.bus_list[n].elem_list[k].name, MLOG_MAX_NAME_LEN);
      WRITE_PAYLOAD(&mlog_handle.header.bus_list[n].elem_list[k].type,
                    sizeof(mlog_handle.header.bus_list[n].elem_list[k].type));
      WRITE_PAYLOAD(&mlog_handle.header.bus_list[n].elem_list[k].number,
                    sizeof(mlog_handle.header.bus_list[n].elem_list[k].number));
    }
  }

  /*********************** write parameter information ***********************/
  /* parameter information */
#if MLOG_ENABLE_PARAM
  char name_buffer[MLOG_MAX_NAME_LEN + 1];
  WRITE_PAYLOAD(&mlog_handle.header.num_param_group, sizeof(mlog_handle.header.num_param_group));
  for (int n = 0; n < mlog_handle.header.num_param_group; n++) {
    memset(name_buffer, 0, MLOG_MAX_NAME_LEN);
    strncpy(name_buffer, mlog_handle.header.param_group_list[n].name, MLOG_MAX_NAME_LEN);

    WRITE_PAYLOAD(name_buffer, MLOG_MAX_NAME_LEN);
    WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_num,
                  sizeof(mlog_handle.header.param_group_list[n].param_num));

    for (int k = 0; k < mlog_handle.header.param_group_list[n].param_num; k++) {
      memset(name_buffer, 0, MLOG_MAX_NAME_LEN);
      strncpy(name_buffer, mlog_handle.header.param_group_list[n].param_list[k].name, MLOG_MAX_NAME_LEN);

      WRITE_PAYLOAD(name_buffer, MLOG_MAX_NAME_LEN);
      WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].type,
                    sizeof(mlog_handle.header.param_group_list[n].param_list[k].type));

      int type = mlog_handle.header.param_group_list[n].param_list[k].type;

      if (type == PARAM_TYPE_INT8) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.i8, sizeof(int8_t));
      } else if (type == PARAM_TYPE_UINT8) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.u8, sizeof(uint8_t));
      } else if (type == PARAM_TYPE_INT16) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.i16, sizeof(int16_t));
      } else if (type == PARAM_TYPE_UINT16) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.u16, sizeof(uint16_t));
      } else if (type == PARAM_TYPE_INT32) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.i32, sizeof(int32_t));
      } else if (type == PARAM_TYPE_UINT32) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.u32, sizeof(uint32_t));
      } else if (type == PARAM_TYPE_FLOAT) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.f, sizeof(float));
      } else if (type == PARAM_TYPE_DOUBLE) {
        WRITE_PAYLOAD(&mlog_handle.header.param_group_list[n].param_list[k].val.lf, sizeof(double));
      } else {
        ulog_w(TAG, "unknown parameter type:%d", type);
      }
    }
  }
#endif

  /*********************** set log status ***********************/
  strncpy(mlog_handle.file_name, file_name, sizeof(mlog_handle.file_name) - 1);

  for (uint8_t i = 0; i < __mlog_bus_num; i++) {
    mlog_handle.stats[i].total_msg = 0;
    mlog_handle.stats[i].lost_msg = 0;
  }

  /* start logging, set flag */
  mlog_handle.log_status = MLOG_STATUS_LOGGING;

  /* invoke callback function */
  invoke_callback_func(MLOG_CB_START);

  ulog_i(TAG, "start logging:%s", file_name);

  return FMT_EOK;
}

/**
 * Call this function to stop the binary log
 *
 */
void mlog_stop(void) {
  /* here we just set log status to stopping, it actually stops
     when the mlog_async_output() is called later */
  if (mlog_handle.log_status == MLOG_STATUS_LOGGING) {
    mlog_handle.log_status = MLOG_STATUS_STOPPING;
  }
}

/**
 * Asynchronous binary logs to storage device
 *
 * @note you must call this function periodically
 */
void mlog_async_output(void) {
  uint32_t head_p, tail_p;
  uint8_t need_sync = 0;

  if (!mlog_handle.is_open) {
    /* no log file is opened */
    return;
  }

  rt_base_t level = OS_ENTER_CRITICAL;
  head_p = mlog_handle.buffer.head;
  tail_p = mlog_handle.buffer.tail;
  OS_EXIT_CRITICAL;

  /* check if we need synchronous the output */
  need_sync = (head_p != tail_p);
  /* write log buffer sector into storage device */
  while (head_p != tail_p) {
    /* check how many sectors that we can write at once */
    uint16_t sector_to_write = get_max_write_sector(head_p, tail_p);
    /* write data to the storage device */
    write(mlog_handle.fid, &mlog_handle.buffer.data[tail_p * MLOG_SECTOR_SIZE], sector_to_write * MLOG_SECTOR_SIZE);
    /* update buffer pointer */
    tail_p = (tail_p + sector_to_write) % mlog_handle.buffer.num_sector;
    level = OS_ENTER_CRITICAL;
    mlog_handle.buffer.tail = tail_p;
    OS_EXIT_CRITICAL;
  }

  /* synchronous the disk to make sure data have been written */
  if (need_sync) {
    fsync(mlog_handle.fid);
  }

  /* if logging is off, clean up the buffer. */
  if (mlog_handle.log_status == MLOG_STATUS_STOPPING) {
    /* dump rest data in buffer */
    if (mlog_handle.buffer.index) {
      write(mlog_handle.fid, &mlog_handle.buffer.data[tail_p * MLOG_SECTOR_SIZE], mlog_handle.buffer.index);
      fsync(mlog_handle.fid);
    }
    /* close the file if needed */
    if (mlog_handle.is_open) {
      close(mlog_handle.fid);
      mlog_handle.fid = -1;
      mlog_handle.is_open = 0;
    }

    /* set log status to idle */
    mlog_handle.log_status = MLOG_STATUS_IDLE;

    /* invoke callback function */
    invoke_callback_func(MLOG_CB_STOP);

    ulog_i(TAG, "stop logging:%s", mlog_handle.file_name);
    for (uint8_t i = 0; i < __mlog_bus_num; i++) {
      ulog_i(TAG, "%-20s id:%-3d record:%-8d lost:%-5d", __mlog_table[i].name, i, mlog_handle.stats[i].total_msg,
             mlog_handle.stats[i].lost_msg);
    }
  }
}

/**
 * Initialize mat log module.
 *
 * @return FMT Errors.
 */
rt_err_t mlog_init(void) {
  static bool init_flag = false;
  if (init_flag == true) {
    return FMT_EOK;
  }

  __mlog_table = (mlog_bus_t*)&__fmt_mlog_start;
  __mlog_bus_num = (mlog_bus_t*)&__fmt_mlog_end - __mlog_table;

  /* initialize mlog_handle status */
  mlog_handle.is_open = 0;
  mlog_handle.log_status = MLOG_STATUS_IDLE;
  /* initialize log header */
  mlog_handle.header.version = MLOG_VERSION;
  mlog_handle.header.timestamp = 0;
  mlog_handle.header.max_name_len = MLOG_MAX_NAME_LEN;
  mlog_handle.header.max_desc_len = MLOG_DESCRIPTION_SIZE;
  mlog_handle.header.max_model_info_len = MLOG_MODEL_INFO_SIZE;
  mlog_handle.header.num_bus = __mlog_bus_num;
  mlog_handle.header.bus_list = __mlog_table;
  /* init param section */
#if MLOG_ENABLE_PARAM
  mlog_handle.header.num_param_group = param_get_group_count();
  mlog_handle.header.param_group_list = param_get_table();
#else
  mlog_handle.header.num_param_group = 0;
#endif
#if MLOG_ENABLE_DESCRIPTION
  memset(mlog_handle.header.description, 0, MLOG_DESCRIPTION_SIZE);
#endif

  /* initialize mlog_handle buffer */
  mlog_handle.buffer.data = __mlog_data_buffer;

  mlog_handle.stats = (mlog_stats_t*)rt_malloc(__mlog_bus_num * sizeof(mlog_stats_t));
  if (mlog_handle.stats == NULL) {
    return FMT_ENOMEM;
  }

  if (mlog_handle.buffer.data == NULL) {
    console_printf("mlog_handle buffer malloc fail!\n");
    return FMT_ENOMEM;
  } else {
    /* initialize buffer */
    mlog_handle.buffer.num_sector = MLOG_BUFFER_SIZE / MLOG_SECTOR_SIZE;
    mlog_handle.buffer.head = 0;
    mlog_handle.buffer.tail = 0;
    mlog_handle.buffer.index = 0;
  }

  /* create write lock */
  if (rt_mutex_init(&mlog_handle.lock, "mlog_lock", RT_IPC_FLAG_FIFO) != RT_EOK) {
    console_printf("fail to create mlog lock!\n");
    return FMT_ERROR;
  }

  init_flag = true;
  return FMT_EOK;
}