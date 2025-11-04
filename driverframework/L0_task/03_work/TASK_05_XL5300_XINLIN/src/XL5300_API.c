#include "XL5300_API.h"
#include "XL5300_Config.h"
#include "XL5300_Firmware_8.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "XL5300_UserPlatform.h"
#include "das.h"
#include "XL5300_API.h"
/* RT-Thread 延时封装（在 xl5300_port.c 实现） */
extern void XL5300_Delay_Ms(uint16_t nMs);
extern XL5300_Calibration_TypeDef XL5300_Cali_Data;

uint8_t gSalve = XL5300_DEVICE_ADDR, chip_reg = 0, chip_reg38, chip_reg3a;

uint8_t XL5300_GPIO_Interrupt_status = 0;
uint8_t VI530x_Chip_Version = 0x31;

void XL5300_Chip_PowerON(uint16_t pin)
{
	HAL_GPIO_WritePin(XSHUT_GPIO_Port, pin, GPIO_PIN_RESET); // set 0
	HAL_Delay(10);											 // delay 30ms
	HAL_GPIO_WritePin(XSHUT_GPIO_Port, pin, GPIO_PIN_SET);	 // set 1
	HAL_Delay(10);
}
uint8_t XL5300_Device_Check(void)
{
	uint8_t Chip_id = 0;
	ReadOneReg(0x06, &Chip_id);
	return Chip_id;
}
uint8_t XL5300_Wait_For_CPU_Ready(void)
{
	XL5300_Status Status = XL5300_OK;
	uint8_t stat;
	int retry = 0;
	do
	{
		HAL_Delay(1); // delay 1ms
		Status = ReadOneReg(0x02, &stat);
	} while ((retry++ < 20) && (stat & 0x01));
	if (retry >= 20)
	{
	
		return 1;
	}
	return Status;
}

XL5300_Status XL5300_Set_Digital_Clock_Dutycycle(void)
{
	XL5300_Status ret = XL5300_OK;

	ret |= WriteOneReg(XL5300_REG_PW_CTRL, 0x0F);
	ret |= WriteOneReg(XL5300_REG_PW_CTRL, 0x0E);
	HAL_Delay(5); //默认4ms ，延时不准可以增加

	return ret;
}

uint8_t reg_sys_cfg = 0;
/*****************
download firmware
******************/
/***********add the function of setting integral counts and frame counts *********/
XL5300_Status XL5300_Integral_Counts_Write(uint32_t inte_counts)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t buf[4] = {0};
	//小端模式，从小到大保存
	buf[0] = (inte_counts)&0xFF;
	buf[1] = (inte_counts >> 8) & 0xFF;
	buf[2] = (inte_counts >> 16) & 0xFF;
	/*******************FOR XL5300 **********************/
	/********************************************************/
	XL5300_Set_Digital_Clock_Dutycycle(); //  
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x03);
	ret |= WriteOneReg(0x0E, 0x01);
	ret |= WriteOneReg(0x0F, buf[0]);
	ret |= WriteOneReg(0x10, buf[1]);
	ret |= WriteOneReg(0x11, buf[2]);
	ret |= WriteOneReg(0x0A, 0x09);
	XL5300_CHECK_RET(ret);

	return ret;
}
XL5300_Status XL5300_Delay_Count_Write(uint16_t delay_count)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t buf[2] = {0};

	//大端模式
	buf[0] = (delay_count >> 8) & 0xFF;
	buf[1] = (delay_count)&0xFF;
	XL5300_Set_Digital_Clock_Dutycycle(); 
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x02);
	ret |= WriteOneReg(0x0E, 0x04);
	//大端模式
	ret |= WriteOneReg(0x0F, buf[0]);
	ret |= WriteOneReg(0x10, buf[1]);
	ret |= WriteOneReg(0x0A, 0x09);
	XL5300_CHECK_RET(ret);

	return ret;
}
XL5300_Status XL5300_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns)
{
	XL5300_Status ret = XL5300_OK;
	uint32_t inte_time;
	uint32_t fps_time;
	uint32_t delay_time;
	uint16_t delay_counts;
	inte_time = intecoutns * 1463 / 10;
	fps_time = 1000000000 / fps;
	delay_time = fps_time - inte_time - 1600000; // 1600000  3000000
	delay_counts = (uint16_t)(delay_time / 40900);
	ret |= XL5300_Integral_Counts_Write(intecoutns);
	ret |= XL5300_Delay_Count_Write(delay_counts);
	XL5300_CHECK_RET(ret);

	return ret;
}
XL5300_Status XL5300_Temp_Enable(uint8_t enable)
{
	XL5300_Status ret = XL5300_OK;

    XL5300_Set_Digital_Clock_Dutycycle();
	ret |= WriteOneReg(0x0C, 0x01);
	ret |= WriteOneReg(0x0D, 0x01);
	ret |= WriteOneReg(0x0E, 0x0E);
	ret |= WriteOneReg(0x0F, enable);
	ret |= WriteOneReg(0x0A, 0x09);
	HAL_Delay(5);
	return ret;
}
uint8_t XL5300_Stop_Continuous_Measure(void)
{
	XL5300_Status ret = XL5300_OK;

	ret = WriteCommand(0x1F);
	HAL_Delay(10);
	XL5300_CHECK_RET(ret);

	return ret;
}
XL5300_Status XL5300_Clear_Interrupt(void)
{
	uint8_t ret = 0;
	//硬件中断
	XL5300_GPIO_Interrupt_status = 0;
	//寄存器中断（软件中断）
	return ReadOneReg(XL5300_RET_INT_STATUS, &ret);
}
XL5300_Status XL5300_Chip_Register_Init(uint8_t *chip_version)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t version_31_cnt = 5;
	uint8_t version_30_cnt = 5;

	//读取芯片版本
	do
	{
		ret |= ReadOneReg(0x38, chip_version);
		if (*chip_version == 0x30)
		{
			version_31_cnt--;
		}
		else
		{
			version_30_cnt--;
		}
	} while (version_31_cnt && version_30_cnt); // 0x30 v3.1, 0x00 v3.0

	if (version_31_cnt == 0)
	{
		*chip_version = 0x31;
	}
	else
	{
		*chip_version = 0x30;
	}

	//寄存器初始化
	/*****************************************************************/
	/***********************for v3.1 test ********************/
	//#define VI530x_POWER_MANAGE       1

	if(XL5300_Cali_Data.XL5300_Power_Manage_Status)
	{
		//开启电源管理模式
		ret |= WriteOneReg(XL5300_REG_SYS_CFG, 0x0C);
	}
	else
	{
		//关闭启电源管理模式
		ret |= WriteOneReg(XL5300_REG_SYS_CFG, 0x08);
	}

	ret |= WriteOneReg(0x07, 0x00); //閿熸枻鎷蜂綅PD閿熸枻鎷锋媷閿熸枻鎷烽敓锟?
	ret |= WriteOneReg(0x07, 0x01);
	ret |= WriteOneReg(0x07, 0x00); //閿熻?闈╂嫹A0閿熸枻鎷锋媷閿熸枻鎷烽敓琛楋拷
	ret |= WriteOneReg(0x04, 0x21);
	ret |= WriteOneReg(0x05, 0x0e);

	ret |= WriteOneReg(0x08, 0x00);
	ret |= WriteOneReg(0x37, 0x80);
	ret |= WriteOneReg(0x38, 0x30); // v3.0 0x00
	ret |= WriteOneReg(0x39, 0x00);
	ret |= WriteOneReg(0x3a, 0x30); // v3.0 0x00
	ret |= WriteOneReg(0x3b, 0x80);
	ret |= WriteOneReg(0x3c, 0x80);
	ret |= WriteOneReg(0x3d, 0x80);
	ret |= WriteOneReg(0x3e, 0x00);
	ret |= WriteOneReg(0x3f, 0x00);
	ret |= WriteOneReg(0x07, 0x0e);
	ret |= WriteOneReg(0x07, 0x0f);
	
	if (XL5300_Cali_Data.XL5300_Crystal_Mode_Status)
	{
		ret |= WriteOneReg(0x01, 0x08); //关闭电源管理，闭合otp switch
		ret |= WriteOneReg(0x04, 0x01); //屏蔽内部中断
		ret |= WriteOneReg(0x3d, 0x88); //进入trim mode
	}

	return ret;
}
/**
 * @brief 	初始化VI530X
 * @param 	[none] 
 * @return 	[uint8_t]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
XL5300_Status VI530x_Chip_Init(void)
{
	XL5300_Status Status = XL5300_OK;;
	uint8_t IIC_ID = 0;

	//配置电源管理模式：0----关闭电源管理模式，其他值----开启电源管理
	//开启后VI530x待机进入低功耗
	XL5300_Cali_Data.XL5300_Power_Manage_Status = 0x01;//开启电源管理
	XL5300_Cali_Data.XL5300_Crystal_Mode_Status = 0; //内部晶振
	//Xshut使能
	XL5300_Chip_PowerON(XSHUT_Pin1); // powerON
	Status |= ReadOneReg(VAN_REG_IIC_DEV_ADDR, &IIC_ID);
	if(IIC_ID != XL5300_DEVICE_ADDR)  
	{
		Status = XL5300_ERROR;
		//Debug
	#ifdef Debug_Mode
		/* VI530x Device ID 为 0xD8, ID不对时不可通讯，请检测	*/
		printf("Check device ID is 0x%2x!\r\n",IIC_ID);
	#endif
	}

#ifdef Change_IIC_Dev_Addr	
	//更改IIC地址为新地址，调用前需要保证IIC总线上地址为VI530x_IIC_DEV_ADDR的器件释放IIC
	//多颗VI530x共IIC总线时需要分别使能对应Xshut引脚配置
	Status |= VI530x_Set_ModelChangeAddr(VI530x_IIC_DEV_ADDR2);			
#endif

	Status |= XL5300_Chip_Register_Init(&VI530x_Chip_Version);

#ifdef Debug_Mode
	printf("VI530x chip version is %x\r\n",VI530x_Chip_Version);
#endif

	Status |= XL5300_Wait_For_CPU_Ready();
	return Status;
}
/**
 * @brief 	VI530X 下固件前的配置
 * @param 	[none] 
 * @return 	[uint8_t]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
XL5300_Status VI530x_Write_Firmware_PreConfig(void)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t reg_sys_cfg = 0;
	ret |= XL5300_Set_Digital_Clock_Dutycycle();

	ret |= WriteOneReg(XL5300_REG_PW_CTRL, 0x08);

	ret |= WriteOneReg(XL5300_REG_PW_CTRL, 0x0a);

	ret |= WriteOneReg(VAN_REG_MCU_CFG, 0x06); // 0x02

	ret |= ReadOneReg(XL5300_REG_SYS_CFG, &reg_sys_cfg);

	ret |= WriteOneReg(XL5300_REG_SYS_CFG, reg_sys_cfg | (0x01 << 0));

	ret |= WriteOneReg(VAN_REG_CMD, 0x01);

	ret |= WriteOneReg(XL5300_REG_SIZE, 0x02);

	ret |= WriteOneReg(XL5300_REG_SCRATCH_PAD_BASE + 0x00, 0x00);

	ret |= WriteOneReg(XL5300_REG_SCRATCH_PAD_BASE + 0x01, 0x00);

	return ret;
}

/**
 * @brief 	VI530X 写完固件后的配置
 * @param 	[none] 
 * @return 	[uint8_t]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
XL5300_Status XL5300_Write_Firmware_Post_Config(void)
{
	XL5300_Status ret = XL5300_OK;
	//#define VI530x_POWER_MANAGE       1
	if(XL5300_Cali_Data.XL5300_Power_Manage_Status)
	{
		//开启电源管理模式
		ret |= WriteOneReg(XL5300_REG_SYS_CFG, 0x0C);
	}
	else
	{
		//关闭启电源管理模式
		ret |= WriteOneReg(XL5300_REG_SYS_CFG, 0x08);
	}
	ret |= WriteOneReg(VAN_REG_MCU_CFG, 0x06);

	ret |= WriteOneReg(VAN_REG_AO_DOMAIN, 0xA0);

	ret |= WriteOneReg(VAN_REG_AO_DOMAIN, 0x80);

	ret |= WriteOneReg(VAN_REG_MCU_CFG, 0x07); // 0x03

	ret |= WriteOneReg(XL5300_REG_PW_CTRL, 0x02);

	ret |= WriteOneReg(XL5300_REG_PW_CTRL, 0x00);

	return ret;
}
XL5300_Status Get_XL5300_Download_Firmware_Status(void)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t value = 0;

	ret |= ReadOneReg(0x08, &value);

	if(0x66 == value || 0xAA == value)
	{
			ret &= ~XL5300_ERROR_FW_FAILURE;
	}
	else
	{
			ret |= XL5300_ERROR_FW_FAILURE;
	}
	return ret;
}
XL5300_Status VI530x_Get_Sys_Histogram_MA_Window_Data(uint8_t *getting_buff)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t i = 0;
	ret |= XL5300_Set_Digital_Clock_Dutycycle();

	ret |= WriteOneReg(0x0C, 0x00);
	ret |= WriteOneReg(0x0D, 0x08);
	ret |= WriteOneReg(0x0E, 0x06);
	ret |= WriteOneReg(0x0A, 0x09);
	XL5300_Delay_Ms(5);
	ret |= I2C_ReadXBytes(0x0C, getting_buff, 8);

	getting_buff[8] = 0;

	for(i = 0;i < 8; i++)
	{
		//MA系数之和
		getting_buff[8] += ((getting_buff[i] & 0x0F)+((getting_buff[i] >> 4) & 0x0F));
	}	

	return ret;
}
/**
 * @brief 	VI530X 写固件
 * @param 	[uint8_t] *Firmware_buff：固件数据地址
 * @param 	[uint16_t] size：固件长度
 * @return 	[uint8_t]	ret：0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */

XL5300_Status VI530x_Download_Firmware(uint8_t *Firmware_buff, uint16_t size)
{
	XL5300_Status ret = XL5300_OK;
	uint8_t page = 0;
//	uint8_t timeout_cnt = 0;

	uint8_t page_buf[32] = {0x00};
	uint8_t temp_buff[2] = {0x00};

	ret |= VI530x_Write_Firmware_PreConfig();

	while (size >= 32)
	{
		temp_buff[0] = XL5300_RET_INT_STATUS;
		temp_buff[1] = 32;

		ret |= I2C_WriteXBytes(VAN_REG_CMD, temp_buff, 2);

		ret |= I2C_WriteXBytes(XL5300_REG_SCRATCH_PAD_BASE, Firmware_buff + page * 32, 32);
		if (ret != XL5300_OK)
		{
		#ifdef Debug_Mode
			printf("VI530x download failed! ret is %x, page is %x\r\n", ret,page);
		#endif
			return XL5300_ERROR_FW_FAILURE; //返回错误状态
		}
		size -= 32;
		page++;
		//写入帧与帧之间必须间隔10us以上
	//	XL5300_Delay_Ms(1);
		XL5300_Delay_Ms(1);
	}

	if (size > 0)
	{
		temp_buff[0] = XL5300_RET_INT_STATUS;
		temp_buff[1] = size;

		ret |= I2C_WriteXBytes(VAN_REG_CMD, temp_buff, 2);

		ret |= I2C_WriteXBytes(XL5300_REG_SCRATCH_PAD_BASE, Firmware_buff + page * 32, size);
		if (ret != XL5300_OK)
		{
		#ifdef Debug_Mode
			printf("VI530x download failed! ret is %x, page is %x\r\n", ret,page);
		#endif
			return XL5300_ERROR_FW_FAILURE; //返回错误状态
		}
	}

	ret |= XL5300_Write_Firmware_Post_Config();
	//此延时是必须的，具体可以改
	XL5300_Delay_Ms(100);
	ret |= Get_XL5300_Download_Firmware_Status();		
	
	//根据·方案写入
	//ret |= VI530x_Bvd_Add(3);
	ret |= VI530x_Get_Sys_Histogram_MA_Window_Data(page_buf);
	XL5300_Cali_Data.MA_Sum  = page_buf[8];
	return ret;
}

