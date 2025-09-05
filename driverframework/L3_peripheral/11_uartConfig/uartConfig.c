#include "uartConfig.h"
#include <string.h>
#include "drv_usart_v2.h"

rt_err_t uart_get_buffer_size(const char* device_name, rt_uint32_t* rx_bufsz, rt_uint32_t* tx_bufsz,
                              rt_uint32_t* dma_ping_bufsz) {
  if (device_name == RT_NULL || rx_bufsz == RT_NULL || tx_bufsz == RT_NULL || dma_ping_bufsz == RT_NULL) {
    return -RT_ERROR;
  }

  if (strcmp(device_name, "uart1") == 0) {
#ifdef BSP_USING_UART1
    *rx_bufsz = BSP_UART1_RX_BUFSIZE;
    *tx_bufsz = BSP_UART1_TX_BUFSIZE;
#if defined(BSP_UART1_RX_USING_DMA) && defined(BSP_UART1_TX_USING_DMA)
    *dma_ping_bufsz = BSP_UART1_DMA_PING_BUFSIZE;
#endif
#else
    rt_kprintf("Error: UART1 not enabled in BSP configuration\n");
    return -RT_ERROR;
#endif
  } else if (strcmp(device_name, "uart2") == 0) {
#ifdef BSP_USING_UART2
    *rx_bufsz = BSP_UART2_RX_BUFSIZE;
    *tx_bufsz = BSP_UART2_TX_BUFSIZE;
#if defined(BSP_UART2_RX_USING_DMA) && defined(BSP_UART2_TX_USING_DMA)
    *dma_ping_bufsz = BSP_UART2_DMA_PING_BUFSIZE;
#endif
#else
    rt_kprintf("Error: UART2 not enabled in BSP configuration\n");
    return -RT_ERROR;
#endif
  } else {
    // 默认值
    *rx_bufsz = 64;
    *tx_bufsz = 64;
    *dma_ping_bufsz = 64;
    rt_kprintf("Warning: Unknown UART device %s, using default buffer size\n", device_name);
    return -RT_ERROR;
  }

  return RT_EOK;
}

rt_err_t uart_config_by_device_name(const char* device_name, rt_uint32_t baud_rate, struct serial_configure* config) {
  if (device_name == RT_NULL || config == RT_NULL) {
    return -RT_ERROR;
  }
  struct serial_configure config_temp = RT_SERIAL_CONFIG_DEFAULT;
  *config = config_temp;
  config->baud_rate = baud_rate;
  rt_uint32_t rx_bufsz, tx_bufsz, dma_ping_bufsz;
  rt_err_t result = uart_get_buffer_size(device_name, &rx_bufsz, &tx_bufsz, &dma_ping_bufsz);
  if (result == RT_EOK) {
    config->rx_bufsz = rx_bufsz;
    config->tx_bufsz = tx_bufsz;
    config->dma_ping_bufsz = dma_ping_bufsz;
    rt_kprintf("UART %s configured: baud=%d, rx_buf=%d, tx_buf=%d, dma_ping=%d\n", device_name, baud_rate, rx_bufsz,
               tx_bufsz, dma_ping_bufsz);
  } else {
    config->rx_bufsz = 64;
    config->tx_bufsz = 64;
    config->dma_ping_bufsz = 64;
    rt_kprintf("UART %s using default buffer size: rx_buf=%d, tx_buf=%d, dma_ping=%d\n", device_name, config->rx_bufsz,
               config->tx_bufsz, config->dma_ping_bufsz);
  }

  return RT_EOK;
}
