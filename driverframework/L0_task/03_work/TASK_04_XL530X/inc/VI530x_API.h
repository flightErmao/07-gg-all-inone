
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_API_H
#define __VI530x_API_H			  	 

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"


//API 版本
//VI5301_MCU_ShortRange1_M40_V201


//Debug时开启
//#define Debug_Mode

//VI530x IIC的设备地址
#define VI530x_IIC_DEV_ADDR     0xD8			//上电默认IIC地址，8位地址
#ifdef Change_IIC_Dev_Addr	
#define VI530x_IIC_DEV_ADDR2    0xD0			//配置IIC地址，8位地址，注意不是7位地址，最低位为0
#endif
extern uint8_t VI530x_IIC_Dev_Addr_Now;



//---------------------------------------------------------------------
//API 版本
//---------------------------------------------------------------------
#define APIversion               0x0202

#define VI530x_DEVICE_ADDR          0xD8
#define VI530x_REG_MCU_CFG          0x00
#define VI530x_RET_INT_STATUS       0x03
#define VI530x_REG_SYS_CFG          0x01
#define VI530x_REG_PW_CTRL          0x07
#define VI530x_REG_CMD              0x0a
#define VI530x_REG_SIZE             0x0b
#define VI530x_REG_SCRATCH_PAD_BASE 0x0c 
#define VI530x_REG_CHIPID_BASE      0x2C
#define VI530x_REG_AO_DOMAIN				0x3B
#define VI530x_REG_IIC_DEV_ADDR     0x06

#define VI530x_WRITEFW_CMD          0x03
#define VI530x_USER_CFG_CMD         0x09
#define VI530x_SINGLE_RANGE_CMD     0x0E
#define VI530x_CONTINOUS_RANGE_CMD  0x0F
#define VI530x_STOP_RANGE_CMD       0x1F


#define VI530x_CFG_SUBCMD           0x01
#define VI530x_OTPW_SUBCMD          0x02
#define VI530x_OTPR_SUBCMD          0x03

// VI530x GPIO中断信号
// 0-清零/没有中断信号
// 1-有中断信号
extern uint8_t VI530x_GPIO_Interrupt_status;

// VI530x system data
typedef struct
{
	uint8_t sys_status;//0x88--APP run,BOOTLOAD_UPDATA--check IAP

	uint8_t calibration_state; // 0x88 calibrate OK
	uint16_t VI530x_current_id;

	//VI530x电源管理设置
	//0x00：不开启电源管理，其它：开启电源管理
	uint8_t VI530x_Power_Manage_Status;
	//VI530x中断状态设置
	//0x88：使用硬件中断,不使用寄存器中断，0x00：不使用硬件中断,使用寄存器查询
	uint8_t VI530x_Interrupt_Mode_Status;	

	//VI530x CG_Pos
	int8_t  VI530x_Cali_CG_Pos;
	//VI530x CG_Bin
	uint8_t VI530x_Cali_CG_Bin;
	//VI530x CG_Maxratio
	uint8_t VI530x_Cali_CG_Maxratio;
	//VI530x CG_Peak
	uint16_t VI530x_Cali_CG_Peak;
	// VI530x Xtalk Ratio计算值
	uint8_t VI530x_Xtalk_Ratio;	
	//VI530x CK
	uint8_t VI530x_Cali_CK;
	//VI530x MP
	uint8_t VI530x_Cali_MP;
	//VI530x BVD
	uint8_t VI530x_Cali_BVD;

	// VI530x reftof标定值
	uint16_t VI530x_Cali_Reftof;
	// VI530x 距离限制值
	uint16_t VI530x_Set_Limit;
	// VI530x offset
	float VI530x_Cali_Offset;
	//VI530x Offset斜率
	float VI530x_Cali_GradientK;
	//VI530x MA系数之和
	uint16_t VI530X_MA_Sum;
	uint16_t VI530x_FW_Len;
	//VI530x SPAD通道数量
	uint8_t VI530x_SPAD_CHANNEL;

	int32_t MaxPeak;
} VI530x_Params_T;

extern VI530x_Params_T VI530x_Cali_Data;

typedef union _SYSTEM_PARAMS_UNION
{
    VI530x_Params_T user_params;
    uint32_t data[60];
} SYSTEM_PARAMS_UNION;



typedef struct
{
	//校正的tof
	int16_t correction_tof;	
	//置信度
	uint8_t confidence;		
	//积分次数
	uint32_t intecounts;
	//Peak
	uint32_t peak;
	//Noise
	uint32_t noise;
	//xtalk_count
	uint16_t xtalk_count;
	//ts
	int8_t ts;

}VI530x_MEASURE_TypeDef;

/**************************************************************************************************
******************************************  Init API *********************************************
**************************************************************************************************/
/**
 * @brief 	通过VI530X的XSHUT引脚拉低再拉高进行复位，保持常拉高---使能
 * @param 	[none]  
 * @return 	[none]
 */
void VI530x_Chip_PowerON(void);
/**
 * @brief 	通过VI530X的XSHUT引脚拉低，保持常拉低---失能
 * @param 	[VI530X_DEV] dev: VI530x data  
 * @return 	[none]
 */
void VI530x_Chip_PowerOFF(void);
/**
 * @brief 	初始化VI530X
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Chip_Init(void);

/**
 * @brief 	VI530X芯片软复位
 * @param 	[none]
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Chip_SWReset(void);

/**
 * @brief 清除中断
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Clear_Interrupt(void);
/**
 * @brief 获取并清除中断信号
 * @param [uint8_t] *interrupt_status	1-有中断，0-无中断
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_And_Clear_Interrupt(uint8_t *interrupt_status);
/**
 * @brief 改IIC设备地址
* @param [uint8_t] addr_val	8位地址，注意不是7位地址，最低位为0
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_ModelChangeAddr(uint8_t addr_val);

/**************************************************************************************************
******************************************  Range API *********************************************
**************************************************************************************************/
/**
 * @brief 	VI530X开始单次测距命令
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Start_Single_Ranging_Cmd(void);
/**
 * @brief 开启连续测距
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Start_Continue_Ranging_Cmd(void);
/**
 * @brief 停止连续测距
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Stop_Continue_Ranging_Cmd(void);
/**
 * @brief 获取测距值
 * @param *data_buff
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Measure_Data(VI530x_MEASURE_TypeDef *result, uint8_t wait_mode);


/**************************************************************************************************
******************************************  Calibration *******************************************
**************************************************************************************************/
/**
 * @brief Xtalk 标定
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Xtalk_Calibration(void);

/**
 * @brief offset标定
 * @param [float] mili   标定距离
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Offset_Calibration(float mili);

/**
 * @brief Gradient标定
 * @param mili_offset	Offset标定的位置
 * @param mili_k	gradient标定的位置
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_GradientK_Calibration(float mili_offset, float mili_k);

/**
 * @brief 设置所有标定参数
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Californiation_Data(float cali_offset);
/**
 * @brief 设置xtalk_Pos
 * @param [uint8_t] xtalk_position
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Xtalk_Position(uint8_t xtalk_position);
/**
 * @brief 获取xtalk_Pos
 * @param [uint8_t] *xtalk_position
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Xtalk_Position(uint8_t *xtalk_position);
/**
 * @brief 设置CG_Maxratio
 * @param [uint8_t] maxratio
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Xtalk_Maxratio(uint8_t maxratio);
/**
 * @brief 获取CG_Maxratio
 * @param [uint8_t] *maxratio
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Xtalk_Maxratio(uint8_t *maxratio);


/**************************************************************************************************
******************************************  Other *******************************************
**************************************************************************************************/

/**
 * @brief 唤醒模组
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_Digital_Clock_Dutycycle(void);
VI530x_Status VI530x_Close_Digital_Clock_Dutycycle(void);
/**
 * @brief 	VI530X 设置系统参数
 * @param 	[uint8_t] offset_addr：起始地址
 * @param 	[uint8_t] *buff：缓存空间
 * @param 	[uint8_t] len：设置的长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Write_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len);

/**
 * @brief 	VI530X 读取系统参数
 * @param 	[uint8_t] offset_addr：起始地址
 * @param 	[uint8_t] *buff：缓存空间
 * @param 	[uint8_t] len：读取的长
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告；
 */
VI530x_Status VI530x_Read_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len);

/**
 * @brief 设置积分次数
 * @param [uint32_t] inte_counts
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Integral_Time(uint32_t inte_counts);
/**
 * @brief 读取积分次数
 * @param [uint32_t] *inte_counts  
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Integral_Time(uint32_t *inte_counts);
/**
 * @brief 设置帧率延时时间
 * @param [uint16_t] delay_count  
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Delay_Count_Write(uint16_t delay_count);
VI530x_Status VI530x_Delay_Count_Read(uint16_t *delay_count);
/**
 * @brief 设置帧率和积分次数
 * @param [uint8_t] fps  帧率
 * @param [uint32_t] intecoutns  积分次数
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns);
/**
 * @brief 设置帧率
 * @param [uint32_t] period 测距后延时时间
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_FPS(uint32_t period);
/**
 * @brief 设置温度校准模式
 * @param [uint8_t] status  0x00-关，0x01-开
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_Temperature_Enable(uint8_t status);
/**
 * @brief 获取温度校准模式
 * @param [uint8_t] *status
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Temperature_Enable(uint8_t *status);
/**
 * @brief 读取 MA窗
 * @param [uint8_t] *getting_buff	8 Byte
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_Histogram_MA_Window_Data(uint8_t *getting_buff);

/**
 * @brief 	VI530X 设置SPAD通道
 * @param 	[uint8_t] channel: 1~9
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Set_Sys_SAPD_Channel(uint8_t channel);

/**
 * @brief 	VI530X 读取SPAD通道
 * @param 	[uint8_t] *channel: 1~9
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_Sys_SAPD_Channel(uint8_t *channel);

VI530x_Status VI530x_Get_FW_Version(uint8_t *rVersion);
/**
 * @brief 	VI530X SN打印
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_Get_SN_Number(void);
/**
 * @brief 	VI530X Debug打印
 * @param 	[none] 
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status  VI530x_Print_All_Reg(void);



/************************************  OTP  Read  *******************************************/

/**
 * @brief 读OTP（少于29Bytes）
 * @param [uint8_t] base		地址
 * @param [uint8_t] *write_buff	数据
 * @param [uint8_t] len			数据长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_OTP_Read_Less_Than_29Bytes(uint8_t base, uint8_t *read_buff, uint8_t len);
/**
 * @brief 读OTP（超过29Bytes）
 * @param [uint8_t] base		地址
 * @param [uint8_t] *read_buff	数据
 * @param [uint8_t] len			数据长度
 * @return 	[VI530x_Status]	ret:0-操作成功;other-异常/警告
 */
VI530x_Status VI530x_OTP_Read_More_Than_29Bytes(uint8_t base,uint8_t *read_buff,uint8_t len);


#endif  
	 
