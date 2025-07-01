#include <drivers/dev_i2c.h>
#include "barometer.h"
#include "spl06.h"

#define DBG_TAG "spl16_001"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define IIC_MAX_WIRTE_COUNT 512

static rt_device_t i2c_dev;

static rt_err_t i2cdevRead(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data) {
    struct rt_i2c_msg msg[2] = {0};
    RT_ASSERT(bus != RT_NULL);

    msg[0].addr = devAddress;
    msg[0].flags = RT_I2C_WR;
    msg[0].len = 1;
    msg[0].buf = &memAddress;

    msg[1].addr = devAddress;
    msg[1].flags = RT_I2C_RD;
    msg[1].len = len;
    msg[1].buf = data;

    if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msg, 2) != 2) {
      rt_kprintf("I2C read data failed, reg = 0x%02x. \n", memAddress);
      return RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t i2cdevWrite(rt_device_t bus, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t* data) {
    struct rt_i2c_msg msgs[1] = {0};
    if (len > IIC_MAX_WIRTE_COUNT - 1) {
        rt_kprintf("[i2cdevWrite] too large len in once write\n");
        return RT_ERROR;
    }
    rt_uint8_t buff[IIC_MAX_WIRTE_COUNT] = {0};

    RT_ASSERT(bus != RT_NULL);

    buff[0] = memAddress;
    rt_memcpy(&buff[1], data, len);

    msgs[0].addr = devAddress;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buff;
    msgs[0].len = len + 1;

    if (rt_i2c_transfer((struct rt_i2c_bus_device*)bus, msgs, 1) != 1) {
      rt_kprintf("I2C write data failed, reg = 0x%2x. \n", memAddress);
      return RT_ERROR;
    }
    return RT_EOK;
}

static int8_t i2c_read_reg8_pack(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len) {
    int8_t ret_function = i2cdevRead(i2c_dev, device_addr, register_addr, len, data);
    return ret_function;
}

static int8_t i2c_write_reg8_pack(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len) {
    int8_t ret_function = i2cdevWrite(i2c_dev, device_addr, register_addr, len, data);
    return ret_function;
}

static void spl_delay_pack(uint16_t ms) { rt_thread_mdelay(ms); }

static rt_err_t lowlevel_init(void) {
    static struct spl06_t spl16_001_;
    spl16_001_.dev_addr = 0x77;
    spl16_001_.bus_read = i2c_read_reg8_pack;
    spl16_001_.bus_write = i2c_write_reg8_pack;
    spl16_001_.delay_msec = spl_delay_pack;

    int ret_init = spl06_init(&spl16_001_);
    int ret_work_mode = spl06_set_work_mode(SPL06_LOW_POWER_MODE);
    int ret_power_mode = spl06_set_power_mode(SPL06_NORMAL_MODE);

    if (ret_init || ret_work_mode || ret_power_mode) {
        while (1) {
            rt_kprintf("sensor baro spl16_001 init failed\n");
            rt_thread_mdelay(2000);
        }
    }

    return RT_EOK;
}

static rt_err_t baro_control(baro_dev_t baro, int cmd, void* arg) {
    // switch (cmd) {
    //     case BARO_CMD_CHECK_READY: {
    //         DEFINE_TIMETAG(baro_interval, 10);
    //         *(uint8_t*)arg = check_timetag(TIMETAG(baro_interval));
    //     } break;
    //     default:
    //         break;
    // }

    return RT_EOK;
}

static rt_size_t baro_read(baro_dev_t baro, baro_report_t* report) {
    rt_size_t size = 0;

    uint32_t pressure_compensate = spl06_compensate_pressure_int32();

    report->pressure_Pa = (float)(pressure_compensate / 100.0f);
    size = sizeof(baro_report_t);

    return size;
}

static struct baro_ops _baro_ops = {.baro_control = baro_control, .baro_read = baro_read};

rt_err_t drv_spl16_001_init(const char* i2c_device_name, const char* baro_device_name) {
    static struct baro_device baro_dev = {.ops = &_baro_ops};

    i2c_dev = rt_device_find(i2c_device_name);
    RT_ASSERT(i2c_dev != NULL);
    RT_ASSERT(rt_device_open(i2c_dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK);

    /* device low-level initialization */
    RT_ASSERT(lowlevel_init() == RT_EOK);
    /* register barometer device */
    RT_ASSERT(hal_baro_register(&baro_dev, baro_device_name, RT_DEVICE_FLAG_RDWR, RT_NULL) == RT_EOK);

    return RT_EOK;
}

static int drv_spl16_001_reg_id1(void) { return drv_spl16_001_init(SPL16001_I2C_NAME_ID1, SPL16001_DEVICE_NAME_ID1); }

static int drv_spl16_001_reg_id2(void) { return drv_spl16_001_init(SPL16001_I2C_NAME_ID2, SPL16001_DEVICE_NAME_ID2); }

#ifdef SPL16001_ID1
INIT_COMPONENT_EXPORT(drv_spl16_001_reg_id1);
#endif

#ifdef SPL16001_ID2
INIT_COMPONENT_EXPORT(drv_spl16_001_reg_id2);
#endif
