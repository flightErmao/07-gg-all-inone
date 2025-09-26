#include <fal.h>
#include "rtthread.h"
#include "rtconfig.h"
#include "drv_flash.h"

#define FLASH_SIZE_PAR_128K_OFFSET (7 * 128 * 1024)
#define FLASH_SIZE_PAR_128K_START_ADDRS (STM32_FLASH_START_ADRESS + FLASH_SIZE_PAR_128K_OFFSET)
#define FLASH_SIZE_PAR_128K_SPACE_SIZE (128 * 1024)

static int fal_flash_read_128k(long offset, rt_uint8_t *buf, size_t size);
static int fal_flash_write_128k(long offset, const rt_uint8_t *buf, size_t size);
static int fal_flash_erase_128k(long offset, size_t size);

const struct fal_flash_dev stm32_onchip_flash_last_128k =
{
    "onchip_flash_last_128k",
    FLASH_SIZE_PAR_128K_START_ADDRS,
    FLASH_SIZE_PAR_128K_SPACE_SIZE,
    (128 * 1024),
    {
        NULL,
        fal_flash_read_128k,
        fal_flash_write_128k,
        fal_flash_erase_128k,
    },
    8,
    {},
};

static int fal_flash_read_128k(long offset, rt_uint8_t *buf, size_t size)
{
    return stm32_flash_read(stm32_onchip_flash_last_128k.addr + offset, buf, size);
}

static int fal_flash_write_128k(long offset, const rt_uint8_t *buf, size_t size)
{
    return stm32_flash_write(stm32_onchip_flash_last_128k.addr + offset, buf, size);
}

static int fal_flash_erase_128k(long offset, size_t size)
{
    return stm32_flash_erase(stm32_onchip_flash_last_128k.addr + offset, size);
}

#ifdef L3_PERIPHERAL_USING_ON_CHIP_FLASH_AUTOSTART_FALINIT
INIT_DEVICE_EXPORT(fal_init);
#endif
