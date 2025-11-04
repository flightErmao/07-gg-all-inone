#include "VI530x_User_Handle.h"
#include "VI530x_API.h"
#include "VI530x_Firmware.h"
/* Start user code for adding. */

/* End user code.  */


/***********************************************************
 * 此文档需要客户移植完成
 * 1、IIC读写函数
 * 2、实现延时函数：void VI530x_Delay_Ms(uint16_t nMs)
 * 3、如果使用硬件中断,则在触发下降沿中断中调用：void VI530x_GPIO_Interrupt_Handle(void)
 * 4、XSHUT高低电位的控制：void VI530x_XSHUT_Enable(uint8_t state)
 * "Start user code for adding"--"End user code"间请根据应用完善
 * ********************************************************/
 
/* Start user code for adding. */
//Xshut引脚配置

//IIC读/写函数
/**dev_addr：设备地址，默认0xD8
***addr：寄存器地址
***value/pValue：读/写数据缓存区地址
***tlen：长度
***return：[uint8_t]0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
**/
// 这些函数在taskTofXl530x.c中实现
extern uint8_t IIC_Write_X_Bytes(uint8_t dev_addr, uint8_t addr, uint8_t *pValue, uint16_t tlen);
extern uint8_t IIC_Read_X_Bytes(uint8_t dev_addr, uint8_t addr, uint8_t *value, uint16_t tlen);

/* End user code.  */

/**
 * @brief 	VI530X I2C 1Byte 读
 * @param 	[uint8_t] addr：读地址
 * @param 	[uint8_t] *value：读到的值
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Read_One_Byte(uint8_t addr, uint8_t *value)
{		
	
	uint8_t ret = 0;
	
	ret = IIC_Read_X_Bytes(VI530x_IIC_Dev_Addr_Now,addr,value,1);

	if(ret != 0)
		return VI530x_IIC_ERROR;
	else
		return VI530x_OK;

}

/**
 * @brief 	VI530X I2C x Byte 读
 * @param 	[uint8_t] addr：读地址
 * @param 	[uint8_t] *value：读到的值
 * @param 	[uint16_t] tlen：读取的长度
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Read_X_Bytes(uint8_t addr, uint8_t *value, uint16_t tlen)
{
	uint8_t ret = 0;
	
	ret = IIC_Read_X_Bytes(VI530x_IIC_Dev_Addr_Now,addr,value,tlen);
	
	if(ret != 0)
		return VI530x_IIC_ERROR;
	else
		return VI530x_OK;

}

/**
 * @brief 	VI530X I2C 1 Byte 写
 * @param 	[uint8_t] addr：写的地址
 * @param 	[uint8_t] value：写入的值
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Write_One_Byte(uint8_t addr, uint8_t value)
{	
	uint8_t ret = 0;
	
	ret = IIC_Write_X_Bytes(VI530x_IIC_Dev_Addr_Now,addr,&value,1);
	
	if(ret != 0)
		return VI530x_IIC_ERROR;
	else
		return VI530x_OK;

}

/**
 * @brief 	VI530X I2C 1 Byte 写
 * @param 	[uint8_t] addr：写的地址
 * @param 	[uint8_t] *pValue：写入的值
 * @param 	[uint16_t] tlen：写入的长度
 * @return 	[VI530x_Status]	ret:0-操作成功（I2C读写无异常）;other-异常（I2C读写有异常）
 */
VI530x_Status VI530x_IIC_Write_X_Bytes(uint8_t addr, uint8_t *pValue, uint16_t tlen)
{	
	uint8_t ret = 0;
	
	ret = IIC_Write_X_Bytes(VI530x_IIC_Dev_Addr_Now,addr,pValue,tlen);
	
	if(ret != 0)
		return VI530x_IIC_ERROR;
	else
		return VI530x_OK;

}

/**
 * @brief 	VI530X 延时 单位：ms
 * @param 	[none]
 * @return 	[none]
 */
// VI530x_Delay_Ms 函数在 taskTofXl530x.c 中实现
// 这里不再定义，避免重复定义错误

//添加于输入中断函数内调用
/**
 * @brief 	VI530X 延时 硬件中断处理
 * @param 	[none]
 * @return 	[none]
 */
// VI530x_GPIO_Interrupt_Handle 函数在 taskTofXl530x.c 中实现
// 这里不再定义，避免重复定义错误

/**
 * @brief 	VI530X XSHUS引脚控制
 * @param 	[uint8_t] state：0-拉低，1-拉高
 * @return 	[none]
 */
// VI530x_XSHUT_Enable 函数在 taskTofXl530x.c 中实现
// 这里不再定义，避免重复定义错误

VI530x_MEASURE_TypeDef result;
//移植流程
void VI530x_main(void)
{
	VI530x_Status ret = VI530x_OK;
	//VI530x_MEASURE_TypeDef result;
	
	//1、IIC 初始化
	//支持1M
	/* Start user code for adding. */

	/* End user code.  */

	//2、GPIO 初始化
	//（a）驱动配置xshut管脚----上拉输出
	//（b）如果使用硬件GPIO中断，则驱动配置GPIO管脚----下降沿输入中断
	/* Start user code for adding. */

	/* End user code.  */

	//3、选择中断方式：0x00----寄存器0x03查询，其他值----GPIO硬件中断
#if 1
	VI530x_Cali_Data.VI530x_Interrupt_Mode_Status = 0x88;		//GPIO引脚启用，硬件中断
#else	
	VI530x_Cali_Data.VI530x_Interrupt_Mode_Status = 0x00;		//GPIO引脚不启用，软件中断
#endif

	//4、VI530x初始化，选择复位方式
#if 1
	VI530x_Chip_PowerON();							//Xshut引脚启用，硬件复位/使能，**建议方式**
#else	
	ret |= VI530x_Chip_SWReset();			//Xshut引脚不启用，软复位，Xshut引脚需上拉
#endif
	ret |= VI530x_Chip_Init();

	//5、VI530x固件写入，系统参数配置
	ret |= VI530x_Download_Firmware((uint8_t *)VI5301_M40_firmware_buff, FirmwareSize());
	/*******请根据应用需求评估测试使用**************/
	ret |= VI530x_Set_Integralcounts_Frame(20,321000);	//帧率，积分次数
	
	//6、标定配置
#if 0
	/************************* 标定后正常测距流程 *****************************/
	/***********从主控端NVM读取标定值，并赋值到标定全局变量**********************/
	/* Start user code for adding. */
	//6.1 从主控端NVM（Flash/EEROM..）读取标定值
	ret |= VI530x_Calibration_Read();
	//6.2 赋值到标定全局变量
	/*******没有标定前用于测试的缺省标定值**************/
	/*
	VI530x_Cali_Data.VI530x_Cali_Offset = 0;
	VI530x_Cali_Data.VI530x_Cali_CG_Pos = -5;
	VI530x_Cali_Data.VI530x_Cali_CG_Bin = 0;
	VI530x_Cali_Data.VI530x_Cali_CG_Maxratio = 6;
	VI530x_Cali_Data.VI530x_Cali_CG_Peak = 1000;
	*/
	/* End user code.  */
	//6.3 配置标定值
	//	VI530x_Cali_Data.VI530x_Cali_CG_Maxratio += 10;	//增加Xtalk处理上限（正常不需要启用）
	ret |= VI530x_Set_Californiation_Data(VI530x_Cali_Data.VI530x_Cali_Offset);
	
#else
	/************************* 生产标定流程 *****************************/
	/*******标定前如果在测距状态，请先停止测距再开始标定*****************/
	
	//Xtalk标定，环境要求60cm内无目标物
	ret |= VI530x_Xtalk_Calibration();
	if(ret == 0)
	{
		printf("VI530x_Calibration_CG_Pos = %d\r\n",VI530x_Cali_Data.VI530x_Cali_CG_Pos);
		printf("VI530x_Calibration_CG_Bin = %d\r\n",VI530x_Cali_Data.VI530x_Cali_CG_Bin);
		printf("VI530x_Calibration_CG_Maxratio = %d\r\n",VI530x_Cali_Data.VI530x_Cali_CG_Maxratio);
		printf("VI530x_Calibration_CG_Peak = %d\r\n",VI530x_Cali_Data.VI530x_Cali_CG_Peak);
		//建议该标定卡控在工装上卡控，便于根据结构情况调整。
		if(VI530x_Cali_Data.VI530x_Cali_CG_Maxratio > 15)
		{
			//检测结构Xtalk影响，盖板为主要影响，可以根据实际情况微调
			printf("Xtalk = %d is too large, Fail!\r\n",VI530x_Cali_Data.VI530x_Cali_CG_Maxratio);
		}
	}
	else
	{
		printf("Xtalk Calibration Fail!\r\n");
	}

	//Offset标定,可以根据应用需求标定关键距离和目标。
	//在固定距离做offset标定,如2cm，则把参数VI530x_OFFSET_DISTANCE改成20（mm）
	ret |= VI530x_Offset_Calibration(VI530x_OFFSET_DISTANCE);
	if(ret == 0)
	{
		printf("VI530x_Calibration_Offset = %f\r\n",VI530x_Cali_Data.VI530x_Cali_Offset);
	}
	else
	{
		printf("Offset Calibration Fail!\r\n");
	}
	/*******************保存标定值在主控NVM*************************/
	/*注意：模组只需做一次标定即可，之后可将标定的值直接写入固件
	VI530x_Cali_Data.VI530x_Cali_CG_Pos					//与结构相关，一般值在区域-10~+10
	VI530x_Cali_Data.VI530x_Cali_CG_Bin					//与结构相关
	VI530x_Cali_Data.VI530x_Cali_CG_Maxratio			//与结构相关，一般值在区域 0~+10
	VI530x_Cali_Data.VI530x_Cali_CG_Peak				//与结构相关
	VI530x_Cali_Data.VI530x_Cali_Offset 				//与电特性和结构相关，一般值在区域 -50~+50
	********************************************/
	/* Start user code for adding. */

	/* End user code.  */
	/** 注：标定结束，芯片自动进入待机状态并关闭温补，需要发送测距命令才会再测距  **/
	
#endif

	//7、其它配置
	//开启温度校准:0x00-关，0x01-开；建议开启
	ret |= VI530x_Set_Sys_Temperature_Enable(0x01);

	//8、开启测距
	//ret |= VI530x_Stop_Continue_Ranging_Cmd();	//关闭命令，给芯片下发命令前需要芯片在停止状态
	ret |= VI530x_Start_Continue_Ranging_Cmd();		//连续模式
	//ret = VI530x_Start_Single_Ranging_Cmd();		//单次模式,功耗低


	if(ret)
	{
		printf("VI530x Configer Error! ret = %x\r\n", ret);
	}
	else
	{
		printf("VI530x Configer Ok!\r\n");
	}

	while(1)
	{
		/* Start user code for adding. */

		/* End user code.  */
		ret = VI530x_Get_Measure_Data(&result, 1);
		//wait_mode:1-在一定时间内等待中断信号，0-没有中断信号则直接退出
		if(!ret)
		{
			//！！建议confidece大于70，ToF值为可信
			printf("tof = %4d, confidece = %3u, peak = %4lu, noise = %4lu, intecounts = %4lu\r\n", result.correction_tof, (unsigned int)result.confidence, (unsigned long)result.peak, (unsigned long)result.noise, (unsigned long)result.intecounts);
			/* 参数说明：
			result.correction_tof：距离值，毫米为单位；
			result.confidence：表示当前 TOF 值的可信度，建议大于70可信，具体可以根据应用调整；
			result.peak：表征接收到光信号强度；
			result.intecounts：积分次数；
			******************/
		}
		//开启单次测距，单次测距调用只会输出1次距离值
		//ret = VI530x_Start_Single_Ranging_Cmd();
			
	}

}












