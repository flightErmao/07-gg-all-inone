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

#ifndef FILE_MANAGER_H__
#define FILE_MANAGER_H__

#include <rtthread.h>
#include <dfs_fs.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mount_entry {
  const char* device_name;
  const char* path;
  const char* filesystemtype;
  unsigned long rwflag;
  const void* data;
} mount_entry_t;

/* file_manager_init error codes (kept compatible with rt_err_t) */
typedef enum {
  FM_INIT_OK = 0,
  FM_INIT_ERR_EMPTY_TABLE = -1,
  FM_INIT_ERR_DEVICE_NOT_FOUND = -2,
  FM_INIT_ERR_INVALID_ROOT_PATH = -3,
  FM_INIT_ERR_MOUNT_FAILED = -4,
  FM_INIT_ERR_CREATE_ROOTFS = -5,
  FM_INIT_ERR_UPDATE_SESSION = -6,
  FM_INIT_ERR_CREATE_SESSION = -7
} fm_init_err_t;

rt_err_t file_manager_init(const mount_entry_t* mnt_table);
rt_err_t current_log_session(char* path);

/* file extended operation */
int fm_fprintf(int fd, const char* fmt, ...);
int fm_deldir(const char* path);

#ifdef __cplusplus
}
#endif

#endif
