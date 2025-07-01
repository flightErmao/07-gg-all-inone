#include <drv_gpio.h>
#include <rtthread.h>

#include "drv_spi.h"

#ifdef BSP_USING_SPI1
#define SPI_NAME_SPI1 "spi1"
#define SPI_CS_SPI10 GET_PIN(A, 4)
#endif

#ifdef BSP_USING_SPI20
#define SPI_NAME_SPI2 "spi2"
#define SPI_CS_SPI20 GET_PIN(B, 0)
#endif

static int rt_hw_spi(void)
{
#ifdef BSP_USING_SPI10
    rt_pin_mode(SPI_CS_SPI10, PIN_MODE_OUTPUT);
    rt_hw_spi_device_attach(SPI_NAME_SPI1, SPI_SLAVE_NAME_SPI10, SPI_CS_SPI10);
#endif

#ifdef BSP_USING_SPI20
    rt_pin_mode(SPI_CS_SPI20, PIN_MODE_OUTPUT);
    rt_hw_spi_device_attach(SPI_NAME_SPI2, SPI_SLAVE_NAME_SPI20, SPI_CS_SPI20);
#endif
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_spi);
