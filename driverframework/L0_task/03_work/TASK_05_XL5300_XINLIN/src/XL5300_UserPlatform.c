#include "XL5300_UserPlatform.h"
#include "XL5300_API.h"
#include "I2cInterface.h"

static I2cInterface_t g_xl5300_i2c;
static rt_bool_t g_xl5300_i2c_inited = RT_FALSE;

static inline XL5300_Status i2c_lazy_init(void)
{
    if (g_xl5300_i2c_inited)
    {
        return XL5300_OK;
    }
    /* 在任务启动阶段已通过 get_i2c_interface 完成，这里只做保护 */
    if (g_xl5300_i2c.i2c_dev != RT_NULL)
    {
        g_xl5300_i2c_inited = RT_TRUE;
        return XL5300_OK;
    }
    return XL5300_ERROR;
}

/* 由任务在初始化时调用，传入 I2cInterface_t */
void XL5300_Bind_I2C_Interface(I2cInterface_t *iface)
{
    if (iface)
    {
        g_xl5300_i2c = *iface;
        g_xl5300_i2c_inited = RT_TRUE;
    }
}

static inline XL5300_Status i2c_write_reg(uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (i2c_lazy_init() != XL5300_OK) return XL5300_ERROR;
    return i2c_write_reg8_mult_pack(g_xl5300_i2c, reg, (uint8_t *)buf, len) == 0 ? XL5300_OK : XL5300_ERROR;
}

static inline XL5300_Status i2c_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (i2c_lazy_init() != XL5300_OK) return XL5300_ERROR;
    return i2c_read_reg8_mult_pack(g_xl5300_i2c, reg, buf, len) == 0 ? XL5300_OK : XL5300_ERROR;
}

XL5300_Status WriteOneReg(uint8_t addr, uint8_t value)
{
    return i2c_write_reg(addr, &value, 1);
}

XL5300_Status ReadOneReg(uint8_t addr, uint8_t *value)
{
    return i2c_read_reg(addr, value, 1);
}

XL5300_Status WriteCommand(uint8_t cmd)
{
    return WriteOneReg(XL5300_REG_CMD, cmd);
}

XL5300_Status I2C_WriteXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    return i2c_write_reg(startaddr, buf, len);
}

XL5300_Status I2C_ReadXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    return i2c_read_reg(startaddr, buf, len);
}






