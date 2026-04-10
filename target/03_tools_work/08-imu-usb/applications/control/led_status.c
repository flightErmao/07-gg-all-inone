#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "led_status.h"

#define APP_LED_STATUS_PIN            GET_PIN(E, 3)
#define APP_LED_STATUS_ON_LEVEL       PIN_LOW
#define APP_LED_STATUS_OFF_LEVEL      PIN_HIGH
#define APP_LED_STATUS_IDLE_DELAY_MS  200U
#define APP_LED_STATUS_RUN_DELAY_MS   250U
#define APP_LED_STATUS_ERR_DELAY_MS   100U

static rt_thread_t g_led_status_thread = RT_NULL;
static volatile led_status_mode_t g_led_status_mode = LED_STATUS_IDLE;
static rt_bool_t g_led_status_inited = RT_FALSE;

static void led_status_thread_entry(void *parameter);

static void led_status_write(rt_base_t level)
{
    rt_pin_write(APP_LED_STATUS_PIN, level);
}

static int led_status_ensure_init(void)
{
    if (g_led_status_inited == RT_TRUE)
    {
        return RT_EOK;
    }

    rt_pin_mode(APP_LED_STATUS_PIN, PIN_MODE_OUTPUT);
    led_status_write(APP_LED_STATUS_OFF_LEVEL);

    g_led_status_thread = rt_thread_create("ledstat",
                                           led_status_thread_entry,
                                           RT_NULL,
                                           1024,
                                           24,
                                           10);
    if (g_led_status_thread == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    g_led_status_inited = RT_TRUE;
    rt_thread_startup(g_led_status_thread);
    return RT_EOK;
}

static void led_status_thread_entry(void *parameter)
{
    rt_bool_t led_on = RT_FALSE;

    RT_UNUSED(parameter);

    while (1)
    {
        switch (g_led_status_mode)
        {
        case LED_STATUS_RUNNING:
            led_on = (led_on == RT_TRUE) ? RT_FALSE : RT_TRUE;
            led_status_write(led_on == RT_TRUE ? APP_LED_STATUS_ON_LEVEL : APP_LED_STATUS_OFF_LEVEL);
            rt_thread_mdelay(APP_LED_STATUS_RUN_DELAY_MS);
            break;

        case LED_STATUS_ERROR:
            led_on = (led_on == RT_TRUE) ? RT_FALSE : RT_TRUE;
            led_status_write(led_on == RT_TRUE ? APP_LED_STATUS_ON_LEVEL : APP_LED_STATUS_OFF_LEVEL);
            rt_thread_mdelay(APP_LED_STATUS_ERR_DELAY_MS);
            break;

        case LED_STATUS_IDLE:
        default:
            led_on = RT_FALSE;
            led_status_write(APP_LED_STATUS_OFF_LEVEL);
            rt_thread_mdelay(APP_LED_STATUS_IDLE_DELAY_MS);
            break;
        }
    }
}

int led_status_set_mode(led_status_mode_t mode)
{
    int result = led_status_ensure_init();

    if (result != RT_EOK)
    {
        return result;
    }

    g_led_status_mode = mode;
    if (mode == LED_STATUS_IDLE)
    {
        led_status_write(APP_LED_STATUS_OFF_LEVEL);
    }

    return RT_EOK;
}
