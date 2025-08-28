
#include "dps368_reg.h"

#include "Dps3xx.h"
#include "barometer.h"
// #include "systime.h"
#define DBG_TAG "dps368_reg"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_device_t i2c_dev;
// static Dps3xx dps368_;

#define IIC_MAX_WIRTE_COUNT 512

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

static void delay_pack(uint16_t ms) { rt_thread_mdelay(ms); }

static rt_err_t lowlevel_init(void) {
    Dps3xx::getInstance().i2c_bus_ops.bus_read = i2c_read_reg8_pack;
    Dps3xx::getInstance().i2c_bus_ops.bus_write = i2c_write_reg8_pack;
    Dps3xx::getInstance().delay_msec = delay_pack;
    Dps3xx::getInstance().begin(SENSOR_DPS368_I2C_ADDR);
    Dps3xx::getInstance().init();

    /*
     * temperature measure rate (value from 0 to 7)
     * 2^temp_mr temperature measurement results per second
     */
    int16_t temp_mr = 2;

    /*
     * temperature oversampling rate (value from 0 to 7)
     * 2^temp_osr internal temperature measurements per result
     * A higher value increases precision
     */
    int16_t temp_osr = 2;

    /*
     * pressure measure rate (value from 0 to 7)
     * 2^prs_mr pressure measurement results per second
     */
    int16_t prs_mr = 2;

    /*
     * pressure oversampling rate (value from 0 to 7)
     * 2^prs_osr internal pressure measurements per result
     * A higher value increases precision
     */
    int16_t prs_osr = 2;

    /*
     * startMeasureBothCont enables background mode
     * temperature and pressure ar measured automatically
     * High precision and hgh measure rates at the same time are not available.
     * Consult Datasheet (or trial and error) for more information
     */
    int16_t ret = Dps3xx::getInstance().startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
    /*
     * Use one of the commented lines below instead to measure only temperature or pressure
     * int16_t ret = Dps3xxPressureSensor.startMeasureTempCont(temp_mr, temp_osr);
     * int16_t ret = Dps3xxPressureSensor.startMeasurePressureCont(prs_mr, prs_osr);
     */

    if (ret != 0) {
        while (1) {
            rt_kprintf("sensor baro Dps3xx init failed\n");
            rt_thread_mdelay(2000);
        }
    }

    return RT_EOK;
}

static rt_err_t baro_control(baro_dev_t baro, int cmd, void* arg) { return RT_EOK; }

static rt_size_t baro_read(baro_dev_t baro, baro_report_t* report) {
    rt_size_t size = 0;

    uint8_t pressureCount = 20;
    float pressure[pressureCount] = {0};
    uint8_t temperatureCount = 20;
    float temperature[temperatureCount] = {0};
    static float temperature_stash = 0.0f;
    static float pressure_stash = 0.0f;
    // report->timestamp_ms = systime_now_ms();
    if (Dps3xx::getInstance().getContResults(temperature, temperatureCount, pressure, pressureCount) == 0) {
        if (temperatureCount) {
            report->temperature_deg = temperature[temperatureCount - 1];
            temperature_stash = temperature[temperatureCount - 1];
        }
        else
        {
            report->temperature_deg = temperature_stash;
        }

        if (pressureCount) {
            report->pressure_Pa = pressure[pressureCount - 1] / 100.0f;
            pressure_stash = pressure[pressureCount - 1] / 100.0f;
        }
        else
        {
            report->pressure_Pa = pressure_stash;
        }

        size = sizeof(baro_report_t);
        return size;
    } else {
        return 0;
    }
}

static struct baro_ops _baro_ops = {.baro_control = baro_control, .baro_read = baro_read};

rt_err_t drv_dps368_init(const char* i2c_device_name, const char* baro_device_name) {
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

static int drv_dps368_reg(void)
{
    return drv_dps368_init(SENSOR_I2C_NAME_DPS368, SENSOR_NAME_DPS368);
}

#ifdef BSP_USING_DPS368
INIT_COMPONENT_EXPORT(drv_dps368_reg);
#endif
