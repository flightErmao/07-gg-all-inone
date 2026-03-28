#include <rtthread.h>
#include <string.h>

#include "fatfs_sdcard_port.h"
#include "file_naming.h"

typedef struct test_dir_entry
{
    const char *test_name;
    const char *dir_name;
} test_dir_entry_t;

static const test_dir_entry_t g_test_dirs[] =
{
    { "bias",      "01_bias" },
    { "temp",      "02_temp" },
    { "temp_drift","02_temp" },
    { "arw",       "03_arw" },
    { "vibration", "04_vibration" },
    { "vibe",      "04_vibration" },
    { "shock",     "05_shock" },
};

static const char *file_naming_dir_name(const char *test_name)
{
    rt_size_t index;

    if (test_name == RT_NULL)
    {
        return RT_NULL;
    }

    for (index = 0; index < sizeof(g_test_dirs) / sizeof(g_test_dirs[0]); ++index)
    {
        if (rt_strcmp(test_name, g_test_dirs[index].test_name) == 0)
        {
            return g_test_dirs[index].dir_name;
        }
    }

    return test_name;
}

int file_naming_make_path(const char *test_name, const char *extension, char *buffer, rt_size_t size)
{
    if ((test_name == RT_NULL) || (buffer == RT_NULL) || (size == 0))
    {
        return -RT_ERROR;
    }

    if (extension == RT_NULL)
    {
        extension = "bin";
    }

    rt_snprintf(buffer,
                size,
                "0:/%s/%08lu.%s",
                test_name,
                (unsigned long)rt_tick_get(),
                extension);
    return RT_EOK;
}

int file_naming_make_next_bin_path(const char *test_name,
                                   char *dir_buffer,
                                   rt_size_t dir_size,
                                   char *path_buffer,
                                   rt_size_t path_size,
                                   rt_uint32_t *index_out)
{
    const char *dir_name;
    rt_uint32_t max_index = 0;
    rt_uint32_t next_index;

    if ((test_name == RT_NULL) || (dir_buffer == RT_NULL) || (path_buffer == RT_NULL))
    {
        return -RT_EINVAL;
    }

    dir_name = file_naming_dir_name(test_name);
    if (dir_name == RT_NULL)
    {
        return -RT_EINVAL;
    }

    rt_snprintf(dir_buffer, dir_size, "0:/%s", dir_name);
    if (fatfs_sdcard_ensure_dir(dir_buffer) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (fatfs_sdcard_find_max_bin_index(dir_buffer, &max_index) != RT_EOK)
    {
        return -RT_ERROR;
    }

    next_index = max_index + 1U;
    rt_snprintf(path_buffer, path_size, "%s/%03lu.BIN", dir_buffer, (unsigned long)next_index);

    if (index_out != RT_NULL)
    {
        *index_out = next_index;
    }

    return RT_EOK;
}
