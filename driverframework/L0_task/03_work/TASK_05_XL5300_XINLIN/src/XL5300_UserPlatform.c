#include "XL5300_UserPlatform.h"
#include "XL5300_API.h"
#include "xl_sw_i2c.h"

#define XL5300_ECO_2V1 1

#ifdef XL5300_ECO_2V1

XL5300_Status I2C_2V1_WriteOneReg(uint8_t addr, uint8_t value)
{
		XL5300_Status ret;
		ret = (XL5300_Status)vi_sw_writereg(gSalve,addr,value);
		if(ret == 1)
		ret = XL5300_OK;
		else if(ret == 0)
		ret = XL5300_ERROR;
		
		return ret;
}

XL5300_Status I2C_2V1_ReadOneReg(uint8_t addr, uint8_t *value)
{
	XL5300_Status ret;
	ret = (XL5300_Status)vi_sw_readreg(gSalve,addr,value,1);
	if(ret == 1)
	ret = XL5300_OK;
	else if(ret == 0)
	ret = XL5300_ERROR;

	return ret;
}

XL5300_Status I2C_WriteXByteWraper(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    XL5300_Status ret = XL5300_OK;
		
		ret = (XL5300_Status)vi_sw_writeRegs(gSalve,startaddr,buf,len);
		if(ret == 1)
		ret = XL5300_OK;
		else if(ret == 0)
		ret = XL5300_ERROR;

    return ret;
}

XL5300_Status I2C_ReadXByteWraper(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    XL5300_Status ret = XL5300_OK;
	
		ret = (XL5300_Status)vi_sw_readRegs(gSalve,startaddr,buf,len);
		if(ret == 1)
		ret = XL5300_OK;
		else if(ret == 0)
		ret = XL5300_ERROR;
		
    return ret;
}

#endif

XL5300_Status WriteOneReg(uint8_t addr, uint8_t value)
{
		return I2C_2V1_WriteOneReg(addr, value);
}

XL5300_Status ReadOneReg(uint8_t addr, uint8_t *value)
{
	
		return I2C_2V1_ReadOneReg(addr, value);
}

XL5300_Status WriteCommand(uint8_t cmd)
{
    return WriteOneReg(XL5300_REG_CMD, cmd);
}

XL5300_Status I2C_WriteXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    return I2C_WriteXByteWraper(startaddr, buf, len);
}

XL5300_Status I2C_ReadXBytes(uint8_t startaddr, uint8_t *buf, uint8_t len)
{
    return I2C_ReadXByteWraper(startaddr, buf, len);
}






