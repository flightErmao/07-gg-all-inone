
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_API_H
#define __VI530x_API_H			  	 

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"


//API 版本
//VI5300_MCU_Gemeral_M31_V206

//VI530x IIC的设备地址
#define VI530x_IIC_DEV_ADDR     0xD8			//上电默认IIC地址，8位地址
#ifdef Change_IIC_Dev_Addr	
#define VI530x_IIC_DEV_ADDR2    0xD0			//配置IIC地址，8位地址，注意不是7位地址，最低位为0
#endif
extern uint8_t VI530x_IIC_Dev_Addr_Now;



//---------------------------------------------------------------------
//API 版本
//---------------------------------------------------------------------
#define APIversion               0x0206
#define VAN_REG_CMD              0x0A
#define VAN_RET_INT_STATUS       0x03
#define VAN_REG_PW_CTRL          0x07

#define VAN_REG_MCU_CFG          0x00
#define VAN_REG_SCRATCH_PAD_BASE 0x0c
#define VAN_REG_SYS_CFG          0x01
#define VAN_REG_AO_DOMAIN        0x3b
#define VAN_REG_SIZE             0x0b
#define VAN_REG_IIC_DEV_ADDR     0x06

#define REG_MCU_CFG          0x00
#define REG_SYS_CFG          0x01
#define REG_PW_CTRL          0x07
#define REG_CMD              0x0a
#define REG_SIZE             0x0b
#define REG_SCRATCH_PAD_BASE 0x0c 

//#define VAN_START_RANG_CMD       0x0e
//2022-3-25测试
#define VAN_START_RANG_CMD       0x0e
#define VAN_WRITEFW_CMD          0x03

// VI530x GPIO中断信号
// 0-清零/没有中断信号
// 1-有中断信号
extern uint8_t VI530x_GPIO_Interrupt_status;

//VI530x状态
typedef enum Result_Status
{
	Result_OK       = 0x00,
	Result_ERROR    = 0x90
}Result_Status;

typedef struct
{
	//530xID值
	uint16_t VI530x_Calibration_ID;	
	//VI530x电源管理设置
	//0x00：不开启电源管理，其它：开启电源管理
	uint8_t VI530x_Power_Manage_Status;
	//VI530x中断状态设置
	//0x88：使用硬件中断,不使用寄存器中断，0x00：不使用硬件中断,使用寄存器查询
	uint8_t VI530x_Interrupt_Mode_Status;	
	//VI530x CG_Pos
	int8_t VI530x_Calibration_CG_Pos;		
	//VI530x CG_Peak
	uint16_t VI530x_Calibration_CG_peak;		
	//VI530x CG_Maxratio
	uint8_t VI530x_Calibration_CG_Maxratio;		
	//VI530x CG_Pos_Min
	int8_t VI530x_Calibration_CG_Pos_Min;	
	//VI530x CG_Pos_Max
	int8_t VI530x_Calibration_CG_Pos_Max;			
	// VI530x CG Xtalk Ratio计算值
	uint8_t VI530x_Xtalk_Ratio;	
	//VI530x CK
	uint8_t VI530x_Calibration_CK;		
	//VI530x MP
	uint8_t VI530x_Calibration_MP;		
	//VI530x reftof标定值
	uint16_t VI530x_Calibration_Reftof;		
	//VI530x 距离限制值
	uint16_t VI530x_Set_Limit;	
	//VI530x offset标定值
	float VI530x_Calibration_Offset;
	//VI530x MA系数之和
	uint16_t VI530x_MA_Sum;
	//斜率
	float VI530x_Calibration_GradientK;
	
}VI530x_Calibration_TypeDef;

extern VI530x_Calibration_TypeDef VI530x_Cali_Data;

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
	uint16_t noise;	
	//xtalk_count
	uint16_t xtalk_count;

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
 * @param 	[none]  
 * @return 	[none]
 */
void VI530x_Chip_PowerOFF(void);

/**
 * @brief 	VI530X芯片软复位
 * @param 	[none]
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Chip_SWReset(void);

/**
 * @brief 	查询VI530X芯片busy等状态
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Wait_For_CPU_Ready(void);

/**
 * @brief 	VI530X中断清除
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Clear_Interrupt(void);

/**
 * @brief 	VI530X中断 读取 & 清除
 * @param 	[uint8_t] *interrupt_status：获取中断状态
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Get_And_Clear_Interrupt(uint8_t *interrupt_status);

/**
 * @brief 改IIC设备地址
 * @param [uint8_t] addr_val	8位地址，注意不是7位地址，最低位为0
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_ModelChangeAddr(uint8_t addr_val);

/**
 * @brief 	VI530X写命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530xWriteCommand(uint8_t cmd);

/**
 * @brief 	初始化VI530X寄存器
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Chip_Register_Init(uint8_t *chip_version);

/**
 * @brief 	初始化VI530X
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Chip_Init(void);


/**************************************************************************************************
******************************************  Range API *********************************************
**************************************************************************************************/

/**
 * @brief 	VI530X开始单次测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Start_Single_Ranging_Cmd(void);

/**
 * @brief 	VI530X开始连续测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Start_Continue_Ranging_Cmd(void);

/**
 * @brief 	VI530X停止连续测距命令
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Stop_Continue_Ranging_Cmd(void);

/**
 * @brief 	VI530X获取测距测距数据
 * @param 	[uint8_t] *data_buff:获取测距数据（包括校准的TOF、confidence）
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Get_Measure_Data(VI530x_MEASURE_TypeDef *measure_data, uint8_t wait_mode);


/**************************************************************************************************
******************************************  Calibration *******************************************
**************************************************************************************************/

/**
 * @brief xTalk 标定
 * @param *pbuff xtalk_pos(1 Byte) + xtalk_peak(2 Bytes) + xtalk_maxratio(1 Bytes)
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
*/
VI530x_Status VI530x_Xtalk_Calibration(void);


/**
 * @brief RefToF 标定
 * @param 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
*/
VI530x_Status VI530x_Reftof_Calibration(void);


/**
 * @brief Offset标定
 * @param mili	标定的位置
 * @param *pbuff	标定结果：(int32) 单位0.1mm
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Offset_Calibration(uint16_t mili);

/**
 * @brief Gradient标定
 * @param mili_offset	Offset标定的位置
 * @param mili_k	gradient标定的位置
 * @return
 */
VI530x_Status VI530x_GradientK_Calibration(uint16_t mili_offset, uint16_t mili_k);

/**
 * @brief 	VI530X 下载完固件后设置标定数据
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_Californiation_Data(VI530x_Calibration_TypeDef calidata);

/**************************************************************************************************
******************************************  Other *******************************************
**************************************************************************************************/
/**
 * @brief 	VI530X 唤醒
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Set_Digital_Clock_Dutycycle(void);

/**
 * @brief 	VI530X 休眠（VI530x使用，vincent一般不使用）
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status VI530x_Close_Digital_Clock_Dutycycle(void);

/**
 * @brief 	VI530X Debug打印
 * @param 	[none] 
 * @return 	[VI530x_Status ]	ret:0-操作成功;other-异常
 */
VI530x_Status  VI530x_Print_All_Reg(void);

#endif  
	 
