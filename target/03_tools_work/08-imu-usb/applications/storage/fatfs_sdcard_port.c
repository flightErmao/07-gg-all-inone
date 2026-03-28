#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/classes/block.h>
#include <ctype.h>

#include "ff.h"
#include "diskio.h"

#include "fatfs_sdcard_port.h"

#define DBG_TAG "fatfs.sd"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define FATFS_SDCARD_DRIVE        "0:"
#define FATFS_SDCARD_LOG_PATH     "0:/imu_log.csv"
#define FATFS_SDCARD_TEST_PATH    "0:/imu_test.csv"
#define FATFS_SDCARD_BLOCK_DEVICE "sd0"

static FATFS g_fatfs;
static FIL g_log_file;
static rt_bool_t g_fs_mounted;
static rt_bool_t g_log_opened;
static rt_device_t g_sd_dev;
static struct rt_device_blk_geometry g_sd_geometry;
static rt_mutex_t g_fatfs_lock;

static int fatfs_sdcard_disk_open(void)
{
    if (g_sd_dev != RT_NULL)
    {
        return RT_EOK;
    }

    g_sd_dev = rt_device_find(FATFS_SDCARD_BLOCK_DEVICE);
    if (g_sd_dev == RT_NULL)
    {
        LOG_E("block device %s not found", FATFS_SDCARD_BLOCK_DEVICE);
        return -RT_ERROR;
    }

    if (rt_device_open(g_sd_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        LOG_E("open %s failed", FATFS_SDCARD_BLOCK_DEVICE);
        g_sd_dev = RT_NULL;
        return -RT_ERROR;
    }

    if (rt_device_control(g_sd_dev,
                          RT_DEVICE_CTRL_BLK_GETGEOME,
                          &g_sd_geometry) != RT_EOK)
    {
        LOG_E("get geometry failed");
        rt_device_close(g_sd_dev);
        g_sd_dev = RT_NULL;
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void fatfs_sdcard_disk_close(void)
{
    if (g_sd_dev != RT_NULL)
    {
        rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
        rt_device_close(g_sd_dev);
        g_sd_dev = RT_NULL;
        rt_memset(&g_sd_geometry, 0, sizeof(g_sd_geometry));
    }
}

rt_bool_t fatfs_sdcard_is_mounted(void)
{
    return g_fs_mounted;
}

int fatfs_sdcard_mount(void)
{
    FRESULT res;

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted == RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return RT_EOK;
    }

    if (fatfs_sdcard_disk_open() != RT_EOK)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    res = f_mount(&g_fatfs, FATFS_SDCARD_DRIVE, 1);
    if (res != FR_OK)
    {
        LOG_E("f_mount failed: %d", res);
        fatfs_sdcard_disk_close();
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    g_fs_mounted = RT_TRUE;
    rt_mutex_release(g_fatfs_lock);
    return RT_EOK;
}

void fatfs_sdcard_unmount(void)
{
    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_log_opened == RT_TRUE)
    {
        f_sync(&g_log_file);
        f_close(&g_log_file);
        g_log_opened = RT_FALSE;
    }

    if (g_fs_mounted == RT_TRUE)
    {
        f_mount(RT_NULL, FATFS_SDCARD_DRIVE, 0);
        g_fs_mounted = RT_FALSE;
    }

    fatfs_sdcard_disk_close();
    rt_mutex_release(g_fatfs_lock);
}

int fatfs_sdcard_open_log(void)
{
    FRESULT res;

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted != RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    if (g_log_opened == RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return RT_EOK;
    }

    res = f_open(&g_log_file, FATFS_SDCARD_LOG_PATH, FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        LOG_E("f_open failed: %d", res);
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    res = f_lseek(&g_log_file, f_size(&g_log_file));
    if (res != FR_OK)
    {
        LOG_E("f_lseek failed: %d", res);
        f_close(&g_log_file);
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    g_log_opened = RT_TRUE;
    rt_mutex_release(g_fatfs_lock);
    return RT_EOK;
}

void fatfs_sdcard_close_log(void)
{
    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_log_opened == RT_TRUE)
    {
        f_sync(&g_log_file);
        f_close(&g_log_file);
        g_log_opened = RT_FALSE;
    }

    if (g_sd_dev != RT_NULL)
    {
        rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
    }

    rt_mutex_release(g_fatfs_lock);
}

int fatfs_sdcard_append_line(const char *line, rt_size_t len, rt_bool_t sync_now)
{
    FRESULT res;
    UINT written = 0;

    if ((line == RT_NULL) || (len == 0U))
    {
        return 0;
    }

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if ((g_fs_mounted != RT_TRUE) || (g_log_opened != RT_TRUE))
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    res = f_write(&g_log_file, line, (UINT)len, &written);
    if ((res == FR_OK) && (sync_now == RT_TRUE))
    {
        res = f_sync(&g_log_file);
        if (g_sd_dev != RT_NULL)
        {
            rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
        }
    }

    rt_mutex_release(g_fatfs_lock);

    if ((res != FR_OK) || (written != len))
    {
        LOG_E("f_write failed: res=%d written=%u len=%u", res, written, (unsigned int)len);
        return -RT_ERROR;
    }

    return (int)written;
}

int fatfs_sdcard_write_fake_imu_test(rt_uint32_t lines, rt_bool_t overwrite, rt_uint32_t *written_lines)
{
    FRESULT res;
    FIL test_file;
    UINT written = 0;
    rt_uint32_t i;
    rt_uint32_t committed = 0;
    char line[96];
    BYTE open_mode;

    if (written_lines != RT_NULL)
    {
        *written_lines = 0;
    }

    if (lines == 0U)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted != RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    open_mode = FA_WRITE | FA_OPEN_ALWAYS;
    if (overwrite == RT_TRUE)
    {
        open_mode |= FA_CREATE_ALWAYS;
    }

    res = f_open(&test_file, FATFS_SDCARD_TEST_PATH, open_mode);
    if (res != FR_OK)
    {
        LOG_E("fake test f_open failed: %d", res);
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    if (overwrite != RT_TRUE)
    {
        res = f_lseek(&test_file, f_size(&test_file));
        if (res != FR_OK)
        {
            LOG_E("fake test f_lseek failed: %d", res);
            f_close(&test_file);
            rt_mutex_release(g_fatfs_lock);
            return -RT_ERROR;
        }
    }

    for (i = 0; i < lines; i++)
    {
        int len = rt_snprintf(line,
                              sizeof(line),
                              "%lu,%lu,%ld,%ld,%ld\r\n",
                              (unsigned long)(i + 1U),
                              (unsigned long)rt_tick_get(),
                              (long)(((i * 3U) % 200U) - 100),
                              (long)(((i * 5U) % 200U) - 100),
                              (long)(((i * 7U) % 200U) - 100));

        if (len <= 0)
        {
            res = FR_INT_ERR;
            break;
        }

        written = 0;
        res = f_write(&test_file, line, (UINT)len, &written);
        if ((res != FR_OK) || (written != (UINT)len))
        {
            LOG_E("fake test f_write failed: res=%d written=%u len=%d", res, written, len);
            break;
        }

        committed++;
    }

    if (res == FR_OK)
    {
        res = f_sync(&test_file);
        if ((res == FR_OK) && (g_sd_dev != RT_NULL))
        {
            rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
        }
    }

    f_close(&test_file);
    rt_mutex_release(g_fatfs_lock);

    if (written_lines != RT_NULL)
    {
        *written_lines = committed;
    }

    return res == FR_OK ? RT_EOK : -RT_ERROR;
}

int fatfs_sdcard_open_raw_file(FIL *file, const char *path, rt_bool_t overwrite)
{
    FRESULT res;
    BYTE mode = FA_WRITE | FA_OPEN_ALWAYS;

    if ((file == RT_NULL) || (path == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if (overwrite == RT_TRUE)
    {
        mode = FA_WRITE | FA_CREATE_ALWAYS;
    }

    rt_memset(file, 0, sizeof(*file));

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted != RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    res = f_open(file, path, mode);
    if ((res == FR_OK) && (overwrite != RT_TRUE))
    {
        res = f_lseek(file, f_size(file));
    }

    rt_mutex_release(g_fatfs_lock);

    if (res != FR_OK)
    {
        LOG_E("raw f_open(%s) failed: %d", path, res);
        rt_memset(file, 0, sizeof(*file));
        return -RT_ERROR;
    }

    return RT_EOK;
}

int fatfs_sdcard_ensure_dir(const char *path)
{
    FRESULT res;

    if (path == RT_NULL)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted != RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    res = f_mkdir(path);
    rt_mutex_release(g_fatfs_lock);

    if ((res != FR_OK) && (res != FR_EXIST))
    {
        LOG_E("f_mkdir(%s) failed: %d", path, res);
        return -RT_ERROR;
    }

    return RT_EOK;
}

int fatfs_sdcard_find_max_bin_index(const char *dir_path, rt_uint32_t *max_index)
{
    DIR dir;
    FILINFO info;
    FRESULT res;
    rt_uint32_t max_found = 0;

    if ((dir_path == RT_NULL) || (max_index == RT_NULL))
    {
        return -RT_EINVAL;
    }

    *max_index = 0;

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted != RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    rt_memset(&dir, 0, sizeof(dir));
    rt_memset(&info, 0, sizeof(info));

    res = f_opendir(&dir, dir_path);
    if (res != FR_OK)
    {
        rt_mutex_release(g_fatfs_lock);
        LOG_E("f_opendir(%s) failed: %d", dir_path, res);
        return -RT_ERROR;
    }

    while (1)
    {
        const char *name;
        rt_uint32_t value = 0;
        int has_digit = 0;

        res = f_readdir(&dir, &info);
        if ((res != FR_OK) || (info.fname[0] == '\0'))
        {
            break;
        }

        if ((info.fattrib & AM_DIR) != 0)
        {
            continue;
        }

        name = info.fname;
        while ((*name != '\0') && isdigit((unsigned char)*name))
        {
            has_digit = 1;
            value = (value * 10U) + (rt_uint32_t)(*name - '0');
            name++;
        }

        if (has_digit && (value > max_found))
        {
            max_found = value;
        }
    }

    f_closedir(&dir);
    rt_mutex_release(g_fatfs_lock);

    if (res != FR_OK)
    {
        LOG_E("f_readdir(%s) failed: %d", dir_path, res);
        return -RT_ERROR;
    }

    *max_index = max_found;
    return RT_EOK;
}

int fatfs_sdcard_write_raw_file(FIL *file, const void *data, rt_size_t len, rt_bool_t sync_now)
{
    FRESULT res;
    UINT written = 0;

    if ((file == RT_NULL) || (data == RT_NULL) || (len == 0U))
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted != RT_TRUE)
    {
        rt_mutex_release(g_fatfs_lock);
        return -RT_ERROR;
    }

    res = f_write(file, data, (UINT)len, &written);
    if ((res == FR_OK) && (sync_now == RT_TRUE))
    {
        res = f_sync(file);
        if (g_sd_dev != RT_NULL)
        {
            rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
        }
    }

    rt_mutex_release(g_fatfs_lock);

    if ((res != FR_OK) || (written != len))
    {
        LOG_E("raw f_write failed: res=%d written=%u len=%u", res, written, (unsigned int)len);
        return -RT_ERROR;
    }

    return (int)written;
}

void fatfs_sdcard_close_raw_file(FIL *file, rt_bool_t sync_now)
{
    if (file == RT_NULL)
    {
        return;
    }

    rt_mutex_take(g_fatfs_lock, RT_WAITING_FOREVER);

    if (g_fs_mounted == RT_TRUE)
    {
        if (sync_now == RT_TRUE)
        {
            f_sync(file);
            if (g_sd_dev != RT_NULL)
            {
                rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
            }
        }

        f_close(file);
    }

    rt_memset(file, 0, sizeof(*file));
    rt_mutex_release(g_fatfs_lock);
}

DWORD get_fattime(void)
{
    return ((DWORD)(2026 - 1980) << 25)
         | ((DWORD)3 << 21)
         | ((DWORD)27 << 16)
         | ((DWORD)0 << 11)
         | ((DWORD)0 << 5)
         | ((DWORD)0 >> 1);
}

static int fatfs_sdcard_port_init(void)
{
    g_fatfs_lock = rt_mutex_create("fatfs0", RT_IPC_FLAG_PRIO);
    return g_fatfs_lock == RT_NULL ? -RT_ENOMEM : RT_EOK;
}
INIT_APP_EXPORT(fatfs_sdcard_port_init);

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0U)
    {
        return STA_NOINIT;
    }

    return fatfs_sdcard_disk_open() == RT_EOK ? 0 : STA_NOINIT;
}

DSTATUS disk_status(BYTE pdrv)
{
    if ((pdrv != 0U) || (g_sd_dev == RT_NULL))
    {
        return STA_NOINIT;
    }

    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if ((pdrv != 0U) || (buff == RT_NULL) || (count == 0U))
    {
        return RES_PARERR;
    }

    if (fatfs_sdcard_disk_open() != RT_EOK)
    {
        return RES_NOTRDY;
    }

    return rt_device_read(g_sd_dev, sector, buff, count) == count ? RES_OK : RES_ERROR;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if ((pdrv != 0U) || (buff == RT_NULL) || (count == 0U))
    {
        return RES_PARERR;
    }

    if (fatfs_sdcard_disk_open() != RT_EOK)
    {
        return RES_NOTRDY;
    }

    return rt_device_write(g_sd_dev, sector, buff, count) == count ? RES_OK : RES_ERROR;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0U)
    {
        return RES_PARERR;
    }

    if (fatfs_sdcard_disk_open() != RT_EOK)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        rt_device_control(g_sd_dev, RT_DEVICE_CTRL_BLK_SYNC, RT_NULL);
        return RES_OK;

    case GET_SECTOR_COUNT:
        if (buff == RT_NULL)
        {
            return RES_PARERR;
        }
        *(DWORD *)buff = g_sd_geometry.sector_count;
        return RES_OK;

    case GET_SECTOR_SIZE:
        if (buff == RT_NULL)
        {
            return RES_PARERR;
        }
        *(WORD *)buff = (WORD)g_sd_geometry.bytes_per_sector;
        return RES_OK;

    case GET_BLOCK_SIZE:
        if (buff == RT_NULL)
        {
            return RES_PARERR;
        }
        *(DWORD *)buff = 1;
        return RES_OK;

    default:
        return RES_PARERR;
    }
}
