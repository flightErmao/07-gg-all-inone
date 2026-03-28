#ifndef APPLICATIONS_FILE_NAMING_H
#define APPLICATIONS_FILE_NAMING_H

#include <rtthread.h>

int file_naming_make_path(const char *test_name, const char *extension, char *buffer, rt_size_t size);
int file_naming_make_next_bin_path(const char *test_name,
                                   char *dir_buffer,
                                   rt_size_t dir_size,
                                   char *path_buffer,
                                   rt_size_t path_size,
                                   rt_uint32_t *index_out);

#endif
