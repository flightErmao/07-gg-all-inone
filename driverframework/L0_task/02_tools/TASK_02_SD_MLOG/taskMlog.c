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
#include <rtdevice.h>
#include <string.h>
#include <stdio.h>

#include "taskMlog.h"
#include "fileManager.h"
#include "mlog.h"

#define TAG "TaskMlog"

/* Thread configuration */
#define THREAD_PRIORITY 15
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

/* Event definitions */
#define EVENT_MLOG_UPDATE (1 << 0)

/* Global variables */
static struct rt_event _mlog_event;
static int _mlog_event_inited = 0;
static const mount_entry_t mnt_table[] = {{"sd0", "/", "elm", 0, NULL}, {NULL}};

static rt_err_t task_mlog_init(void);
/* Mlog update callback */
static void mlog_update_cb(void) { rt_event_send(&_mlog_event, EVENT_MLOG_UPDATE); }

/* Mlog task entry function */
static void task_mlog_entry(void* parameter) {
  rt_err_t rt_err;
  rt_uint32_t recv_set = 0;
  rt_uint32_t wait_set = EVENT_MLOG_UPDATE;
  rt_err_t result;

  /* Initialize mlog task */
  result = task_mlog_init();
  if (result != RT_EOK) {
    while (1) {
      rt_thread_mdelay(3000);
      rt_kprintf("[%s] Failed to initialize mlog task: %d\n", TAG, result);
    }
  }

  rt_kprintf("[%s] Mlog task started\n", TAG);

  /* Register mlog callback */
  mlog_register_callback(MLOG_CB_UPDATE, mlog_update_cb);

  while (1) {
    /* Wait for event */
    rt_err = rt_event_recv(&_mlog_event, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 100, &recv_set);

    if (rt_err == RT_EOK) {
      if (recv_set & EVENT_MLOG_UPDATE) {
        mlog_async_output();
      }
    } else if (rt_err == -RT_ETIMEOUT) {
      /* Timeout, check if there are log data need to send */
      mlog_async_output();
    } else {
      /* Some other error happened */
      rt_kprintf("[%s] Event receive error: %d\n", TAG, rt_err);
    }
  }
}

/* Mlog task initialization */
static rt_err_t task_mlog_init(void) {
  rt_err_t result;

  /* Create mlog event */
  if (!_mlog_event_inited) {
    result = rt_event_init(&_mlog_event, "mlog_event", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK) {
      rt_kprintf("[%s] Failed to create mlog event: %d\n", TAG, result);
      return -1;
    }
    _mlog_event_inited = 1;
  }

  /* Initialize file manager */
  result = file_manager_init(mnt_table);
  if (result != RT_EOK) {
    while (1) {
      rt_kprintf("[%s] Failed to initialize file manager: %d\n", TAG, result);
      rt_thread_mdelay(3000);
    }
    return -2;
  }

  /* Initialize mlog */
  result = mlog_init();
  if (result != RT_EOK) {
    rt_kprintf("[%s] Failed to initialize mlog: %d\n", TAG, result);
    return -3;
  }

  rt_kprintf("[%s] Mlog task initialized successfully\n", TAG);
  return RT_EOK;
}

/* Start mlog logging */
rt_err_t task_mlog_start_logging(char* file_path) {
  rt_err_t result;
  char log_name[100];
  char file_name[50];
  static uint8_t mlog_id = 0;

  if (file_path) {
    /* If a valid path is provided, use it for mlog */
    return mlog_start(file_path);
  }

  /* Get current log session */
  if (current_log_session(log_name) != RT_EOK) {
    rt_kprintf("[%s] No available log session\n", TAG);
    return -RT_ERROR;
  }

  sprintf(file_name, "/mlog%d.bin", ++mlog_id);
  strcat(log_name, file_name);

  result = mlog_start(log_name);
  if (result == RT_EOK) {
    rt_kprintf("[%s] Started logging to: %s\n", TAG, log_name);
  } else {
    rt_kprintf("[%s] Failed to start logging: %d\n", TAG, result);
  }

  return result;
}

/* Stop mlog logging */
void task_mlog_stop_logging(void) {
  mlog_stop();
  rt_kprintf("[%s] Stopped logging\n", TAG);
}

/* Get mlog status */
uint8_t task_mlog_get_status(void) { return mlog_get_status(); }

/* Get current log file name */
char* task_mlog_get_file_name(void) { return mlog_get_file_name(); }

/* Show mlog statistics */
void task_mlog_show_statistics(void) { mlog_show_statistic(); }

/* Thread stack and control block */
rt_align(RT_ALIGN_SIZE) static rt_uint8_t task_mlog_stack[THREAD_STACK_SIZE];
static struct rt_thread task_mlog_tid;

/* Thread creation function */
static int taskMlogAutoStart(void) {
  rt_err_t result;

  /* Create mlog thread */
  result = rt_thread_init(&task_mlog_tid, "mlog", task_mlog_entry, RT_NULL, task_mlog_stack, THREAD_STACK_SIZE,
                          THREAD_PRIORITY, THREAD_TIMESLICE);
  if (result != RT_EOK) {
    rt_kprintf("[%s] Failed to create mlog thread: %d\n", TAG, result);
    return -1;
  }

  /* Start the thread */
  rt_thread_startup(&task_mlog_tid);
  rt_kprintf("[%s] Mlog thread created and started\n", TAG);

  return 0;
}

/* Auto start the task */
#ifdef TASK_TOOL_02_SD_MLOG
INIT_APP_EXPORT(taskMlogAutoStart);
#endif