/******************************************************************************
 * Copyright 2020-2021 The Firmament-Autopilot Authors. All Rights Reserved.
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
#include <stdlib.h>
#include <stdio.h>
#include <dfs_posix.h>
#include "fileManager.h"
#include "unistd.h"

#ifndef FM_RETRY_DELAY_MS
#define FM_RETRY_DELAY_MS 3000
#endif

#define MAX_LOG_SESSION_NUM 20
#define LOG_SESSION_FILE "/log/session_id"

static int cws_id; /* current work session id */

static const char* rfs_folder[] = {
    "/sys", "/usr", "/log", "/mnt", NULL /* NULL indicate the end */
};

/**
 * Read log session id from /log/session_id.
 *
 * @return FMT Errors.
 */
static int read_log_session_id(void) {
  char id_buffer[5] = {0};
  struct stat sta;
  int id = -1;
  int fd = -1;

  /* if file existed, read it */
  if (stat(LOG_SESSION_FILE, &sta) == 0) {
    int i;
    fd = open(LOG_SESSION_FILE, O_WRONLY);
    if (fd < 0) {
      return -1;
    }
    /* read session id from file */
    int rb = read(fd, id_buffer, sizeof(id_buffer) - 1);
    /* check if it's a valid number */
    for (i = 0; i < rb; i++) {
      /* session id starts from 1 */
      if (id_buffer[i] < '0' || id_buffer[i] > '9' || id_buffer[0] == '0') {
        /* illegal session id */
        close(fd);
        return -1;
      }
    }

    id = atoi(id_buffer);
    close(fd);
  }

  return id;
}

/**
 * Update log session id into /log/session_id.
 *
 * @return FMT Errors.
 */
static rt_err_t update_log_session(void) {
  int fd;

  /* read current log session id */
  cws_id = read_log_session_id();
  if (cws_id < 0) {
    cws_id = 0;
  }
  rt_kprintf("read_log_session_id: %d\n", cws_id);
  cws_id = (cws_id % MAX_LOG_SESSION_NUM) + 1;
  rt_kprintf("update_log_session: %d\n", cws_id);
  /* open or create the log session id file */
  fd = open(LOG_SESSION_FILE, O_TRUNC | O_CREAT | O_RDWR);
  if (fd < 0) {
    rt_kprintf("fail to create log session id file! err:%ld\n", rt_get_errno());
    return RT_ERROR;
  }
  fm_fprintf(fd, "%d", cws_id);
  close(fd);

  return RT_EOK;
}

/**
 * Create working log session folder.
 *
 * @return FMT Errors.
 */
static rt_err_t create_log_session(void) {
  char path[50];
  struct stat sta;
  /* get log session full path */
  sprintf(path, "/log/session_%d", cws_id);
  rt_kprintf("create_log_session path: %s\n", path);

  if (stat(path, &sta) == 0) {
    /* delete existed folder */
    if (fm_deldir(path) < 0) {
      rt_kprintf("fail to delete %s\n", path);
      return RT_ERROR;
    }
  }
  /* create log session */
  // if (mkdir(path, 0x777) < 0) {
  //     rt_kprintf("fail to create %s, errno:%ld\n", path, rt_get_errno());
  //     return RT_ERROR;
  // }

  while (1) {
    int ret = mkdir(path, 0x777);
    if (ret == 0) {
      break;
    } else {
      rt_kprintf("fail to create %s, errno:%ld\n", path, rt_get_errno());
      rt_thread_mdelay(3000);
    }
  }

  return RT_EOK;
}

/**
 * Create rootfs folders if not existed.
 *
 * @return FMT Errors.
 */
static rt_err_t create_rootfs_dirs(void) {
  struct stat buf;
  int i = 0;

  /* clear mnt directory */
  fm_deldir("/mnt");

  /* create rootfs folder structure */
  while (1) {
    if (rfs_folder[i] != NULL) {
      if (stat(rfs_folder[i], &buf) < 0) {
        if (mkdir(rfs_folder[i], 0x777) < 0) {
          rt_kprintf("fail to create %s, errno:%ld\n", rfs_folder[i], rt_get_errno());
          return RT_ERROR;
        }
      }
      i++;
    } else {
      break;
    }
  }

  return RT_EOK;
}

/**
 * Mount root device to "/".
 */
static rt_err_t mount_root(const mount_entry_t* mnt) {
  if (strcmp("/", mnt->path) != 0) {
    rt_kprintf("fail, you should mount / first!\n");
    return RT_ERROR;
  }

  if (dfs_mount(mnt->device_name, mnt->path, mnt->filesystemtype, mnt->rwflag, mnt->data) != 0) {
    rt_kprintf("Fail to mount %s at %s!\n", mnt->device_name, mnt->path);
    return RT_ERROR;
  }

  return RT_EOK;
}

/**
 * Get the current log session path.
 *
 * @param path store the full path of current log session
 * @return FMT Errors.
 */
rt_err_t current_log_session(char* path) {
  if (cws_id > 0) {
    /* get log session full path */
    sprintf(path, "/log/session_%d", cws_id);
    return RT_EOK;
  } else {
    return RT_ERROR;
  }
}

/**
 * Initialize the file system.
 *
 * @return FMT Errors.
 */
rt_err_t file_manager_init(const mount_entry_t* mnt_table) {
  // struct stat sta;

  if (mnt_table[0].device_name == NULL) {
    /* empty mount table, just return */
    rt_kprintf("empty mount table, just return!\n");
    return FM_INIT_ERR_EMPTY_TABLE;
  }

  while (1) {
    if (rt_device_find(mnt_table[0].device_name) != RT_NULL) {
      break;
    } else {
      rt_kprintf("fail to find %s!\n", mnt_table[0].device_name);
      rt_thread_mdelay(FM_RETRY_DELAY_MS);
    }
  }

  if (strcmp("/", mnt_table[0].path) != 0) {
    rt_kprintf("fail, you should mount / first!\n");
    return FM_INIT_ERR_INVALID_ROOT_PATH;
  }

  /* step 1: mount root, retry until success */
  while (1) {
    if (mount_root(&mnt_table[0]) == RT_EOK) {
      break;
    }
    rt_kprintf("mount_root failed, retry after %d ms\n", FM_RETRY_DELAY_MS);
    rt_thread_mdelay(FM_RETRY_DELAY_MS);
  }

  /* step 2: create rootfs dirs, retry until success */
  while (1) {
    if (create_rootfs_dirs() == RT_EOK) {
      break;
    }
    rt_kprintf("create_rootfs_dirs failed, retry after %d ms\n", FM_RETRY_DELAY_MS);
    rt_thread_mdelay(FM_RETRY_DELAY_MS);
  }

  /* step 3: update log session id, retry until success */
  while (1) {
    if (update_log_session() == RT_EOK) {
      break;
    }
    rt_kprintf("update_log_session failed, retry after %d ms\n", FM_RETRY_DELAY_MS);
    rt_thread_mdelay(FM_RETRY_DELAY_MS);
  }

  /* step 4: create log session dir, retry until success */
  while (1) {
    if (create_log_session() == RT_EOK) {
      break;
    }
    rt_kprintf("create_log_session failed, retry after %d ms\n", FM_RETRY_DELAY_MS);
    rt_thread_mdelay(FM_RETRY_DELAY_MS);
  }

  /* mount other devices */
  // for (int i = 1;; i++) {
  //     if (mnt_table[i].path == NULL) {
  //         break;
  //     }
  //     /* if path doesn't exit, create it */
  //     if (stat(mnt_table[i].path, &sta) < 0) {
  //         if (mkdir(mnt_table[i].path, 0x777) < 0) {
  //             rt_kprintf("fail to create %s, errno:%ld\n", mnt_table[i].path, rt_get_errno());
  //             return RT_ERROR;
  //         }
  //     }

  //     if (dfs_mount(mnt_table[i].device_name,
  //                   mnt_table[i].path,
  //                   mnt_table[i].filesystemtype,
  //                   mnt_table[i].rwflag,
  //                   mnt_table[i].data)
  //         < 0) {
  //         rt_kprintf("Fail to mount %s at %s!\n",
  //                mnt_table[i].device_name,
  //                mnt_table[i].path);
  //         /* delete the failed mount path */
  //         fm_deldir(mnt_table[i].path);
  //     }
  // }

  return FM_INIT_OK;
}