#ifndef APPLICATIONS_FATFS_SDCARD_PORT_H
#define APPLICATIONS_FATFS_SDCARD_PORT_H

#include <rtthread.h>
#include "ff.h"

int fatfs_sdcard_mount(void);
void fatfs_sdcard_unmount(void);
int fatfs_sdcard_open_log(void);
void fatfs_sdcard_close_log(void);
int fatfs_sdcard_append_line(const char *line, rt_size_t len, rt_bool_t sync_now);
int fatfs_sdcard_write_file(const char *path, const char *data, rt_size_t len, rt_bool_t overwrite);
int fatfs_sdcard_open_raw_file(FIL *file, const char *path, rt_bool_t overwrite);
int fatfs_sdcard_write_raw_file(FIL *file, const void *data, rt_size_t len, rt_bool_t sync_now);
void fatfs_sdcard_close_raw_file(FIL *file, rt_bool_t sync_now);
int fatfs_sdcard_ensure_dir(const char *path);
int fatfs_sdcard_find_max_bin_index(const char *dir_path, rt_uint32_t *max_index);
rt_bool_t fatfs_sdcard_is_mounted(void);

#endif
