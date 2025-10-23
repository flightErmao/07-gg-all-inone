
#include "task/init/task_init.h"

#include <rtdevice.h>
#include <rtthread.h>

#include "drv_debugPin.h"

#define HWTIMER_DEV_NAME "timer3"

#define EVENT_TIMER_NOTIFY_IMU (1 << 1)
#define EVENT_TIMER_NOTIFY_DSHOT (1 << 2)
// #define EVENT_TIMER_NOTIFY_IMU (1 << 3)
// #define EVENT_TIMER_NOTIFY_DSHOT (1 << 4)

#define EVENT_DAL_GYRO_UPDATE (1 << 1)
#define EVENT_DAL_ACC_UPDATE (1 << 2)
#define EVENT_DAL_RPM_UPDATE (1 << 3)

#define EVENT_FILTER_DOWNSAMPLE_UPDATE (1 << 1)

#define EVENT_RPM_PARSE (1 << 1)

static rt_sem_t core_tick_sem = RT_NULL;
static rt_sem_t imu_xfer_sem = RT_NULL;  // test purpose
static rt_sem_t accgyr_ready_sem = RT_NULL;
static rt_sem_t dshot_tx_cplt_sem = RT_NULL;
static rt_sem_t dshot_rx_cplt_sem = RT_NULL;
static rt_sem_t rpm_data_ready_sem = RT_NULL;
static rt_sem_t rpm_filter_applied_sem = RT_NULL;

static struct rt_event event_hwtimer;  // hwtimer event
static struct rt_event event_dal;      // data access layer event
static struct rt_event event_filter;   // filter event
static struct rt_event event_rpm;      // filter event

static int init_all_sem(void) {
    core_tick_sem = rt_sem_create("coreticksem", 0, RT_IPC_FLAG_PRIO);
    if (core_tick_sem == RT_NULL) {
        rt_kprintf("create coreticksem failed.\n");
        return -1;
    } else {
        rt_kprintf("\ncreate done. coreticksem value = 0.\n");
    }

    imu_xfer_sem = rt_sem_create("imuxfersem", 0, RT_IPC_FLAG_PRIO);
    if (imu_xfer_sem == RT_NULL) {
        rt_kprintf("create imuxfersem failed.\n");
        return -1;
    } else {
        rt_kprintf("create done. imuxfersem value = 0.\n");
    }

    accgyr_ready_sem = rt_sem_create("accgyrreadysem", 0, RT_IPC_FLAG_PRIO);
    if (accgyr_ready_sem == RT_NULL) {
        rt_kprintf("create accgyrreadysem failed.\n");
        return -1;
    } else {
        rt_kprintf("create done. accgyrreadysem value = 0.\n");
    }

    dshot_tx_cplt_sem = rt_sem_create("dshottxcpltsem", 0, RT_IPC_FLAG_PRIO);
    if (dshot_tx_cplt_sem == RT_NULL) {
        rt_kprintf("create dshottxcpltsem failed.\n");
        return -1;
    } else {
        rt_kprintf("create done. dshottxcpltsem value = 0.\n");
    }

    dshot_rx_cplt_sem = rt_sem_create("dshotrxcpltsem", 0, RT_IPC_FLAG_PRIO);
    if (dshot_rx_cplt_sem == RT_NULL) {
        rt_kprintf("create dshotrxcpltsem failed.\n");
        return -1;
    } else {
        rt_kprintf("create done. dshotrxcpltsem value = 0.\n");
    }

    rpm_data_ready_sem = rt_sem_create("rpmdatareadysem", 0, RT_IPC_FLAG_PRIO);
    if (rpm_data_ready_sem == RT_NULL) {
        rt_kprintf("create rpmdatareadysem failed.\n");
        return -1;
    } else {
        rt_kprintf("create done. rpmdatareadysem value = 0.\n");
    }

    rpm_filter_applied_sem = rt_sem_create("rpmfilterappliedsem", 0, RT_IPC_FLAG_PRIO);
    if (rpm_filter_applied_sem == RT_NULL) {
        rt_kprintf("create rpmfilterappliedsem failed.\n");
        return -1;
    } else {
        rt_kprintf("create done. rpmfilterappliedsem value = 0.\n");
    }

    return 0;
}

rt_err_t release_sem(const char *name) {
    if (name == "coreticksem") {
        rt_sem_release(core_tick_sem);
    } else if (name == "imuxfersem") {
        rt_sem_release(imu_xfer_sem);
    } else if (name == "accgyrreadysem") {
        rt_sem_release(accgyr_ready_sem);
    } else if (name == "dshottxcpltsem") {
        rt_sem_release(dshot_tx_cplt_sem);
    } else if (name == "dshotrxcpltsem") {
        rt_sem_release(dshot_rx_cplt_sem);
    } else if (name == "rpmdatareadysem") {
        rt_sem_release(rpm_data_ready_sem);
    } else if (name == "rpmfilterappliedsem") {
        rt_sem_release(rpm_filter_applied_sem);
    } else {
        rt_kprintf("release sem failed: no such sem\n");
    }
    return RT_EOK;
}

rt_err_t take_sem(const char *name, rt_int32_t timeout) {
    if (name == "coreticksem") {
        return rt_sem_take(core_tick_sem, timeout);
    } else if (name == "imuxfersem") {
        return rt_sem_take(imu_xfer_sem, timeout);
    } else if (name == "accgyrreadysem") {
        return rt_sem_take(accgyr_ready_sem, timeout);
    } else if (name == "dshottxcpltsem") {
        return rt_sem_take(dshot_tx_cplt_sem, timeout);
    } else if (name == "dshotrxcpltsem") {
        return rt_sem_take(dshot_rx_cplt_sem, timeout);
    } else if (name == "rpmdatareadysem") {
        return rt_sem_take(rpm_data_ready_sem, timeout);
    } else if (name == "rpmfilterappliedsem") {
        return rt_sem_take(rpm_filter_applied_sem, timeout);
    } else {
        rt_kprintf("take sem failed: no such sem\n");
        return RT_EOK;
    }
}

static int init_all_event(void) {
    rt_err_t result;

    /* initiate the event (statically) */
    result = rt_event_init(&event_hwtimer, "event_hwtimer", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK) {
        rt_kprintf("init event failed.\n");
        return -1;
    }
    result = rt_event_init(&event_dal, "event_dal", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK) {
        rt_kprintf("init event failed.\n");
        return -2;
    }
    result = rt_event_init(&event_filter, "event_filter", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK) {
        rt_kprintf("init event failed.\n");
        return -3;
    }

    result = rt_event_init(&event_rpm, "event_rpm", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK) {
        rt_kprintf("init event failed.\n");
        return -3;
    }

    return 0;
}

rt_err_t recv_event(const char *pubname, const char *subname, rt_uint32_t opt, rt_int32_t timeout) {
    static rt_event_t event;
    static rt_uint32_t set;
    if (pubname == "hwtimer") {
        event = &event_hwtimer;
        if (subname == "imu") {
            set = EVENT_TIMER_NOTIFY_IMU;
            opt = RT_EVENT_FLAG_OR | opt;
        } else if (subname == "dshot") {
            set = EVENT_TIMER_NOTIFY_DSHOT;
            opt = RT_EVENT_FLAG_OR | opt;
        } else {
            return RT_ERROR;
        }
    } else if (pubname == "dal") {
        event = &event_dal;
        if (subname == "rpm_filter") {
            set = EVENT_DAL_RPM_UPDATE | EVENT_DAL_GYRO_UPDATE;
            opt = RT_EVENT_FLAG_AND | opt;
        } else if (subname == "acc_checker") {
            set = EVENT_DAL_ACC_UPDATE;
            opt = RT_EVENT_FLAG_OR | opt;
        } else {
            return RT_ERROR;
        }
    } else if (pubname == "filter") {
        event = &event_filter;
        if (subname == "downsample_update") {
            set = EVENT_FILTER_DOWNSAMPLE_UPDATE;
            opt = RT_EVENT_FLAG_OR | opt;
        } else {
            return RT_ERROR;
        }
    } else if (pubname == "rpm") {
        event = &event_rpm;
        set = EVENT_RPM_PARSE;
        opt = RT_EVENT_FLAG_OR | opt;
    } else {
        return RT_ERROR;
    }
    return rt_event_recv(event, set, opt, timeout, NULL);
}

rt_err_t ssend_event(const char *pubname, const char *eventname) {
    if (pubname == "hwtimer") {
        return rt_event_send(&event_hwtimer, EVENT_TIMER_NOTIFY_IMU | EVENT_TIMER_NOTIFY_DSHOT);
    } else if (pubname == "dal") {
        if (eventname == "gyro_update") {
            return rt_event_send(&event_dal, EVENT_DAL_GYRO_UPDATE);
        } else if (eventname == "acc_update") {
            return rt_event_send(&event_dal, EVENT_DAL_ACC_UPDATE);
        } else if (eventname == "rpm_update") {
            return rt_event_send(&event_dal, EVENT_DAL_RPM_UPDATE);
        } else {
            return RT_ERROR;
        }
    } else if (pubname == "filter") {
        if (eventname == "downsample_update") {
            return rt_event_send(&event_filter, EVENT_FILTER_DOWNSAMPLE_UPDATE);
        } else {
            return RT_ERROR;
        }
    } else if (pubname == "rpm") {
        return rt_event_send(&event_rpm, EVENT_RPM_PARSE);
    } else {
        return RT_ERROR;
    }
}

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size) {
#ifdef TIME_PIN_DEBUG_EN
    test_pinToggle1();
#endif
    return (ssend_event("hwtimer", "event_timer"));
}

static int hwtimer_init(void) {
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;
    rt_device_t hw_dev = RT_NULL;
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq = 10000000;
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL) {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);

    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK) {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    // TODO: pass the timer config from the user profile or board profile when initialize
    timeout_s.sec = 0;
    timeout_s.usec = 312;
    timeout_s.ns = 500;
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s)) {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    return ret;
}

rt_err_t InitCommon(void) {
    /*init sem should at hwtimer init ahead*/
    if (init_all_sem() != RT_EOK) {
        return -1;
    }
    if (init_all_event() != RT_EOK) {
        return -2;
    }
    if (hwtimer_init() != RT_EOK) {
        return -3;
    }
    // debug_pin_init();
    return 0;
}
