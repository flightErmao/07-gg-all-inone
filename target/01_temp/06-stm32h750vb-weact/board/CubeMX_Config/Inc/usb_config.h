/*
 * CherryUSB configuration for STM32H750VBT6
 * Based on cherryusb_config_template.h
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

#ifdef __RTTHREAD__
#include <rtthread.h>
#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)
#else
#define CONFIG_USB_PRINTF(...) printf(__VA_ARGS__)
#endif

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

#define CONFIG_USB_PRINTF_COLOR_ENABLE

/* data align size when use dma or use dcache */
#define CONFIG_USB_ALIGN_SIZE 4

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* ================= USB Device Stack Configuration ================ */

#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

#define CONFIG_USBDEV_ADVANCE_DESC

#ifndef CONFIG_USBDEV_MSC_MAX_BUFSIZE
#define CONFIG_USBDEV_MSC_MAX_BUFSIZE 512
#endif

/* ================ USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1
#endif

/* DWC2: PA11/PA12 does not support DMA mode */
/* #define CONFIG_USB_DWC2_DMA_ENABLE */

/* ================ USB Host Port Configuration ==================*/
#ifndef CONFIG_USBHOST_MAX_BUS
#define CONFIG_USBHOST_MAX_BUS 1
#endif

#define CONFIG_USBHOST_MAX_RHPORTS          1
#define CONFIG_USBHOST_MAX_EXTHUBS          1
#define CONFIG_USBHOST_MAX_EHPORTS          4
#define CONFIG_USBHOST_MAX_INTERFACES       8
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS 2
#define CONFIG_USBHOST_MAX_ENDPOINTS        4

#define CONFIG_USBHOST_MAX_CDC_ACM_CLASS 4
#define CONFIG_USBHOST_MAX_HID_CLASS     4
#define CONFIG_USBHOST_MAX_MSC_CLASS     2
#define CONFIG_USBHOST_MAX_AUDIO_CLASS   1
#define CONFIG_USBHOST_MAX_VIDEO_CLASS   1

#define CONFIG_USBHOST_DEV_NAMELEN 16

#ifndef CONFIG_USBHOST_PSC_PRIO
#define CONFIG_USBHOST_PSC_PRIO 0
#endif
#ifndef CONFIG_USBHOST_PSC_STACKSIZE
#define CONFIG_USBHOST_PSC_STACKSIZE 2048
#endif

#ifndef CONFIG_USBHOST_REQUEST_BUFFER_LEN
#define CONFIG_USBHOST_REQUEST_BUFFER_LEN 512
#endif

#ifndef CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT
#define CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT 500
#endif

#ifndef usb_phyaddr2ramaddr
#define usb_phyaddr2ramaddr(addr) (addr)
#endif
#ifndef usb_ramaddr2phyaddr
#define usb_ramaddr2phyaddr(addr) (addr)
#endif

#endif /* CHERRYUSB_CONFIG_H */
