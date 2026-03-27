#ifndef APPLICATIONS_FATFS_SDCARD_PORT_H
#define APPLICATIONS_FATFS_SDCARD_PORT_H

#include <rtthread.h>

int fatfs_sdcard_mount(void);
void fatfs_sdcard_unmount(void);
int fatfs_sdcard_open_log(void);
void fatfs_sdcard_close_log(void);
int fatfs_sdcard_append_line(const char *line, rt_size_t len, rt_bool_t sync_now);
int fatfs_sdcard_write_fake_imu_test(rt_uint32_t lines, rt_bool_t overwrite, rt_uint32_t *written_lines);
rt_bool_t fatfs_sdcard_is_mounted(void);

#endif
