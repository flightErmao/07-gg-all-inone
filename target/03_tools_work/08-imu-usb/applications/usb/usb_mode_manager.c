#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/usb_device.h>
#include <board.h>

#include "fatfs_sdcard_port.h"
#include "usb_mode_manager.h"
#include <vconsole.h>
#include <finsh.h>

#define DBG_TAG "usb.mode"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define APP_USB_SWITCH_EVENT     (1UL << 0)
#define APP_USB_FLUSH_LINES      16
#define APP_USB_BOOT_MODE        USB_APP_MODE_CDC
#define APP_USB_BOOT_MAGIC       0x554D4F44UL

extern ufunction_t rt_usbd_function_cdc_create(udevice_t device);
extern ufunction_t rt_usbd_function_mstorage_create(udevice_t device);
extern udcd_t stm_usbd_get_udcd(void);
extern int stm_usbd_start(void);
extern int stm_usbd_stop(void);

#ifdef RT_USB_DEVICE_COMPOSITE
static const char *g_usb_cdc_strings[] =
{
    "Language",
    "RT-Thread Team.",
    "IMU CDC Device",
    "320219198301",
    "Configuration",
    "Interface",
    USB_STRING_OS,
};

static const char *g_usb_composite_strings[] =
{
    "Language",
    "RT-Thread Team.",
    "IMU CDC+MSC Device",
    "320219198301",
    "Configuration",
    "Interface",
    USB_STRING_OS,
};

static struct udevice_descriptor g_usb_composite_desc =
{
    USB_DESC_LENGTH_DEVICE,
    USB_DESC_TYPE_DEVICE,
    USB_BCD_VERSION,
    USB_CLASS_MISC,
    0x02,
    0x01,
    0x40,
    USB_VENDOR_ID,
    (USB_PRODUCT_ID + 2),
    USB_BCD_DEVICE,
    USB_STRING_MANU_INDEX,
    USB_STRING_PRODUCT_INDEX,
    USB_STRING_SERIAL_INDEX,
    USB_DYNAMIC,
};

static struct usb_qualifier_descriptor g_usb_composite_qualifier =
{
    sizeof(g_usb_composite_qualifier),
    USB_DESC_TYPE_DEVICEQUALIFIER,
    0x0200,
    USB_CLASS_MISC,
    0x02,
    0x01,
    64,
    0x01,
    0,
};

static struct usb_os_comp_id_descriptor g_usb_comp_id_desc =
{
    {
        USB_DYNAMIC,
        0x0100,
        0x04,
        USB_DYNAMIC,
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    },
};
#endif

typedef struct
{
    usb_app_mode_t active_mode;
    usb_app_mode_t target_mode;
    rt_bool_t usb_ready;
    rt_bool_t fs_ready;
    rt_bool_t logger_running;
    rt_mutex_t lock;
    rt_event_t event;
    rt_thread_t switch_thread;
    rt_thread_t logger_thread;
    udevice_t device;
    ufunction_t cdc_function;
    ufunction_t msc_function;
    int log_fd;
    rt_uint32_t log_line_count;
    int last_result;
    rt_uint32_t last_stage;
} usb_mode_ctx_t;

static usb_mode_ctx_t g_usb_mode = {
    .active_mode = USB_APP_MODE_SWITCHING,
    .target_mode = USB_APP_MODE_CDC,
    .log_fd = -1,
    .last_result = 0,
    .last_stage = 0,
};

static __attribute__((section(".noinit"))) rt_uint32_t g_usb_boot_magic;
static __attribute__((section(".noinit"))) rt_uint32_t g_usb_boot_mode;

static void usb_mode_store_boot_mode(usb_app_mode_t mode)
{
    g_usb_boot_magic = APP_USB_BOOT_MAGIC;
    g_usb_boot_mode = (rt_uint32_t)mode;
}

static usb_app_mode_t usb_mode_take_boot_mode(void)
{
    usb_app_mode_t mode = APP_USB_BOOT_MODE;

    if ((g_usb_boot_magic == APP_USB_BOOT_MAGIC) &&
        ((g_usb_boot_mode == USB_APP_MODE_CDC) || (g_usb_boot_mode == USB_APP_MODE_MSC)))
    {
        mode = (usb_app_mode_t)g_usb_boot_mode;
    }

    g_usb_boot_magic = 0;
    g_usb_boot_mode = APP_USB_BOOT_MODE;

    return mode;
}

static void usb_mode_reset_to(usb_app_mode_t mode)
{
    usb_mode_store_boot_mode(mode);
    rt_thread_mdelay(50);
    NVIC_SystemReset();
}

rt_weak int usb_mode_fill_sensor_line(char *buf, rt_size_t size)
{
    static rt_uint32_t seq = 0;
    rt_tick_t tick = rt_tick_get();
    rt_int32_t ax = (rt_int32_t)((seq * 3U) % 200U) - 100;
    rt_int32_t ay = (rt_int32_t)((seq * 5U) % 200U) - 100;
    rt_int32_t az = (rt_int32_t)((seq * 7U) % 200U) - 100;

    seq++;

    return rt_snprintf(buf,
                       size,
                       "%lu,%lu,%ld,%ld,%ld\r\n",
                       (unsigned long)seq,
                       (unsigned long)tick,
                       (long)ax,
                       (long)ay,
                       (long)az);
}

static void usb_mode_lock(void)
{
    rt_mutex_take(g_usb_mode.lock, RT_WAITING_FOREVER);
}

static void usb_mode_unlock(void)
{
    rt_mutex_release(g_usb_mode.lock);
}

static void usb_mode_attach_shell_to_vcom(void)
{
    rt_device_t vcom = rt_device_find("vcom");

    if (vcom == RT_NULL)
    {
        LOG_W("vcom not found, shell stays on current console");
        return;
    }

    vconsole_switch(vcom);
    finsh_set_device(vcom->parent.name);
    LOG_I("shell attached to %s", vcom->parent.name);
}

static rt_err_t usb_mode_build_device(usb_app_mode_t mode)
{
    uconfig_t cfg;
    udevice_t dev;
    ufunction_t cdc_func = RT_NULL;
    ufunction_t msc_func = RT_NULL;

    g_usb_mode.last_stage = 10;
    dev = rt_usbd_device_new();
    if (dev == RT_NULL)
    {
        LOG_E("rt_usbd_device_new failed");
        return -RT_ENOMEM;
    }

    rt_usbd_device_set_controller(dev, stm_usbd_get_udcd());

    g_usb_mode.last_stage = 11;
    cfg = rt_usbd_config_new();
    if (cfg == RT_NULL)
    {
        LOG_E("rt_usbd_config_new failed");
        rt_usbd_device_unregister(dev);
        return -RT_ENOMEM;
    }

    if (mode == USB_APP_MODE_CDC)
    {
        g_usb_mode.last_stage = 12;
        cdc_func = rt_usbd_function_cdc_create(dev);
        if (cdc_func == RT_NULL)
        {
            LOG_E("create cdc function failed");
            rt_usbd_device_unregister(dev);
            return -RT_ERROR;
        }

        rt_usbd_config_add_function(cfg, cdc_func);
        rt_usbd_device_set_descriptor(dev, cdc_func->dev_desc);
#ifdef RT_USB_DEVICE_COMPOSITE
        rt_usbd_device_set_string(dev, g_usb_cdc_strings);
#endif
    }
    else
    {
        g_usb_mode.last_stage = 13;
        cdc_func = rt_usbd_function_cdc_create(dev);
        msc_func = rt_usbd_function_mstorage_create(dev);
        if ((cdc_func == RT_NULL) || (msc_func == RT_NULL))
        {
            LOG_E("create composite functions failed");
            rt_usbd_device_unregister(dev);
            return -RT_ERROR;
        }

        rt_usbd_config_add_function(cfg, cdc_func);
        rt_usbd_config_add_function(cfg, msc_func);
#ifdef RT_USB_DEVICE_COMPOSITE
        rt_usbd_device_set_descriptor(dev, &g_usb_composite_desc);
        rt_usbd_device_set_string(dev, g_usb_composite_strings);
        rt_usbd_device_set_os_comp_id_desc(dev, &g_usb_comp_id_desc);
        if (dev->dcd->device_is_hs)
        {
            rt_usbd_device_set_qualifier(dev, &g_usb_composite_qualifier);
        }
#else
        rt_usbd_device_set_descriptor(dev, cdc_func->dev_desc);
#endif
    }

    rt_usbd_device_add_config(dev, cfg);

    g_usb_mode.device = dev;
    g_usb_mode.cdc_function = cdc_func;
    g_usb_mode.msc_function = msc_func;
    g_usb_mode.last_stage = 14;

    return RT_EOK;
}

static void usb_mode_destroy_device(void)
{
    if (g_usb_mode.msc_function != RT_NULL)
    {
        FUNC_DISABLE(g_usb_mode.msc_function);
        g_usb_mode.msc_function = RT_NULL;
    }

    if (g_usb_mode.cdc_function != RT_NULL)
    {
        FUNC_DISABLE(g_usb_mode.cdc_function);
        g_usb_mode.cdc_function = RT_NULL;
    }

    if (g_usb_mode.device != RT_NULL)
    {
        rt_usbd_device_unregister(g_usb_mode.device);
        g_usb_mode.device = RT_NULL;
    }
}

static rt_err_t usb_mode_start_usb(usb_app_mode_t mode)
{
    rt_err_t result;

    g_usb_mode.last_stage = 20;
    result = usb_mode_build_device(mode);
    if (result != RT_EOK)
    {
        g_usb_mode.last_result = result;
        return result;
    }

    g_usb_mode.last_stage = 21;
    result = stm_usbd_start();
    if (result != RT_EOK)
    {
        LOG_E("stm_usbd_start failed: %d", result);
        usb_mode_destroy_device();
        g_usb_mode.last_result = result;
        return result;
    }

    rt_thread_mdelay(30);
    g_usb_mode.last_stage = 22;
    result = rt_usbd_set_config(g_usb_mode.device, 1);
    if ((result != RT_EOK) && (result != RT_TRUE))
    {
        LOG_E("rt_usbd_set_config failed: %d", result);
        stm_usbd_stop();
        usb_mode_destroy_device();
        g_usb_mode.last_result = result;
        return result;
    }

    g_usb_mode.usb_ready = RT_TRUE;
    g_usb_mode.active_mode = mode;
    g_usb_mode.last_result = RT_EOK;
    g_usb_mode.last_stage = 23;
    LOG_I("usb enumerated as %s", mode == USB_APP_MODE_CDC ? "CDC" : "CDC+MSC");

    if (g_usb_mode.cdc_function != RT_NULL)
    {
        usb_mode_attach_shell_to_vcom();
    }

    return RT_EOK;
}

static void usb_mode_stop_usb(void)
{
    if (g_usb_mode.usb_ready == RT_FALSE)
    {
        return;
    }

    stm_usbd_stop();
    rt_thread_mdelay(3000);
    usb_mode_destroy_device();
    g_usb_mode.usb_ready = RT_FALSE;
}

static rt_err_t usb_mode_mount_fs(void)
{
    if (g_usb_mode.fs_ready == RT_TRUE)
    {
        return RT_EOK;
    }

    g_usb_mode.last_stage = 30;
    if (fatfs_sdcard_mount() != RT_EOK)
    {
        LOG_E("fatfs mount failed");
        g_usb_mode.last_result = -RT_ERROR;
        return -RT_ERROR;
    }

    g_usb_mode.fs_ready = RT_TRUE;
    return RT_EOK;
}

static void usb_mode_unmount_fs(void)
{
    if (g_usb_mode.fs_ready == RT_FALSE)
    {
        return;
    }

    fatfs_sdcard_unmount();
    g_usb_mode.fs_ready = RT_FALSE;
}

static void usb_mode_close_log(void)
{
    fatfs_sdcard_close_log();
    g_usb_mode.log_fd = -1;
}

static rt_err_t usb_mode_open_log(void)
{
    g_usb_mode.last_stage = 31;
    if (fatfs_sdcard_open_log() != RT_EOK)
    {
        g_usb_mode.last_result = -RT_ERROR;
        return -RT_ERROR;
    }

    g_usb_mode.log_fd = 1;
    g_usb_mode.log_line_count = 0;
    return RT_EOK;
}

static rt_err_t usb_mode_prepare_cdc_side(void)
{
    rt_err_t result;

    g_usb_mode.logger_running = RT_FALSE;
    g_usb_mode.log_fd = -1;
    g_usb_mode.log_line_count = 0;

    result = usb_mode_mount_fs();
    if (result != RT_EOK)
    {
        LOG_W("fatfs not ready at CDC entry, logger will retry in background");
        return RT_EOK;
    }

    result = usb_mode_open_log();
    if (result != RT_EOK)
    {
        LOG_W("log open failed at CDC entry, logger will retry in background");
        usb_mode_unmount_fs();
        return RT_EOK;
    }

    g_usb_mode.logger_running = RT_TRUE;
    return RT_EOK;
}

static void usb_mode_prepare_msc_side(void)
{
    g_usb_mode.logger_running = RT_FALSE;
    usb_mode_close_log();
    usb_mode_unmount_fs();
}

static rt_err_t usb_mode_switch_to(usb_app_mode_t mode)
{
    rt_err_t result;

    g_usb_mode.last_stage = 40;
    usb_mode_lock();
    g_usb_mode.active_mode = USB_APP_MODE_SWITCHING;
    usb_mode_unlock();

    if (mode == USB_APP_MODE_MSC)
    {
        usb_mode_prepare_msc_side();
    }

    usb_mode_stop_usb();

    if (mode == USB_APP_MODE_CDC)
    {
        g_usb_mode.last_stage = 41;
        result = usb_mode_prepare_cdc_side();
        if (result != RT_EOK)
        {
            g_usb_mode.last_result = result;
            return result;
        }
    }

    g_usb_mode.last_stage = 42;
    result = usb_mode_start_usb(mode);
    if (result != RT_EOK)
    {
        if (mode == USB_APP_MODE_CDC)
        {
            g_usb_mode.logger_running = RT_FALSE;
            usb_mode_close_log();
            usb_mode_unmount_fs();
        }
        g_usb_mode.last_result = result;
        return result;
    }

    g_usb_mode.last_result = RT_EOK;
    return RT_EOK;
}

static void usb_mode_request(usb_app_mode_t mode)
{
    usb_mode_lock();
    g_usb_mode.target_mode = mode;
    usb_mode_unlock();
    rt_event_send(g_usb_mode.event, APP_USB_SWITCH_EVENT);
}

static void usb_mode_switch_entry(void *parameter)
{
    rt_uint32_t event;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_event_recv(g_usb_mode.event,
                      APP_USB_SWITCH_EVENT,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER,
                      &event);

        usb_mode_lock();
        usb_app_mode_t current = g_usb_mode.active_mode;
        usb_app_mode_t target = g_usb_mode.target_mode;
        usb_mode_unlock();

        if (target == USB_APP_MODE_CDC || target == USB_APP_MODE_MSC)
        {
            if (current == USB_APP_MODE_SWITCHING)
            {
                usb_mode_switch_to(target);
            }
            else if (current != target)
            {
                if (target == USB_APP_MODE_MSC)
                {
                    usb_mode_prepare_msc_side();
                }

                usb_mode_stop_usb();
                usb_mode_reset_to(target);
            }
        }
    }
}

static void usb_mode_logger_entry(void *parameter)
{
    char line[96];
    rt_tick_t last_retry_tick = 0;

    RT_UNUSED(parameter);

    while (1)
    {
        if ((g_usb_mode.active_mode == USB_APP_MODE_CDC) &&
            (g_usb_mode.logger_running == RT_FALSE))
        {
            rt_tick_t now = rt_tick_get();

            if ((last_retry_tick == 0) ||
                ((now - last_retry_tick) >= rt_tick_from_millisecond(1000)))
            {
                last_retry_tick = now;

                if ((g_usb_mode.fs_ready == RT_FALSE) && (usb_mode_mount_fs() != RT_EOK))
                {
                    rt_thread_mdelay(20);
                    continue;
                }

                if ((g_usb_mode.log_fd < 0) && (usb_mode_open_log() != RT_EOK))
                {
                    usb_mode_unmount_fs();
                    rt_thread_mdelay(20);
                    continue;
                }

                g_usb_mode.logger_running = RT_TRUE;
                LOG_I("CDC logger recovered");
            }
        }

        if ((g_usb_mode.active_mode == USB_APP_MODE_CDC) &&
            (g_usb_mode.logger_running == RT_TRUE) &&
            (g_usb_mode.log_fd >= 0))
        {
            int len = usb_mode_fill_sensor_line(line, sizeof(line));
            if (len > 0)
            {
                rt_bool_t sync_now;

                g_usb_mode.log_line_count++;
                sync_now = ((g_usb_mode.log_line_count % APP_USB_FLUSH_LINES) == 0U) ? RT_TRUE : RT_FALSE;
                fatfs_sdcard_append_line(line, (rt_size_t)len, sync_now);
            }
        }

        rt_thread_mdelay(20);
    }
}



int enter_msc_mode(void)
{
    usb_mode_request(USB_APP_MODE_MSC);
    return RT_EOK;
}
MSH_CMD_EXPORT(enter_msc_mode, switch USB device to CDC+MSC mode);

int enter_cdc_mode(void)
{
    usb_mode_request(APP_USB_BOOT_MODE);
    return RT_EOK;
}
MSH_CMD_EXPORT(enter_cdc_mode, switch USB device to CDC-only mode);

static int usb_mode_status(void)
{
    const char *state = "switching";

    if (g_usb_mode.active_mode == USB_APP_MODE_CDC)
    {
        state = "cdc";
    }
    else if (g_usb_mode.active_mode == USB_APP_MODE_MSC)
    {
        state = "cdc+msc";
    }

    rt_kprintf("mode=%s fs=%d log_fd=%d\r\n",
               state,
               g_usb_mode.fs_ready,
               g_usb_mode.log_fd);
    return RT_EOK;
}
MSH_CMD_EXPORT(usb_mode_status, show USB mode manager state);

usb_app_mode_t usb_mode_manager_current_mode(void)
{
    return g_usb_mode.active_mode;
}

int usb_mode_manager_start(void)
{
    static rt_bool_t started = RT_FALSE;

    if (started == RT_TRUE)
    {
        return RT_EOK;
    }

    g_usb_mode.lock = rt_mutex_create("usbmode", RT_IPC_FLAG_PRIO);
    g_usb_mode.event = rt_event_create("usbevt", RT_IPC_FLAG_PRIO);
    if ((g_usb_mode.lock == RT_NULL) || (g_usb_mode.event == RT_NULL))
    {
        return -RT_ENOMEM;
    }

    rt_usbd_core_init();

    g_usb_mode.switch_thread = rt_thread_create("usb_sw",
                                                usb_mode_switch_entry,
                                                RT_NULL,
                                                4096,
                                                12,
                                                10);
    g_usb_mode.logger_thread = rt_thread_create("usb_log",
                                                usb_mode_logger_entry,
                                                RT_NULL,
                                                4096,
                                                16,
                                                20);
    if ((g_usb_mode.switch_thread == RT_NULL) ||
        (g_usb_mode.logger_thread == RT_NULL))
    {
        return -RT_ENOMEM;
    }

    rt_thread_startup(g_usb_mode.switch_thread);
    rt_thread_startup(g_usb_mode.logger_thread);

    started = RT_TRUE;
    usb_mode_request(usb_mode_take_boot_mode());
    return RT_EOK;
}
