#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/classes/block.h>

#include "ff.h"
#include "diskio.h"

#include "fatfs_sdcard_port.h"

#define DBG_TAG "fatfs.sd"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define FATFS_SDCARD_DRIVE        "0:"
#define FATFS_SDCARD_LOG_PATH     "0:/imu_log.csv"
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
