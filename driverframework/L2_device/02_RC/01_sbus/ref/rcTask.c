#include <stdbool.h>
#include <string.h>
#include "rcTask.h"
#include "stm32f10x.h"
#include "main_ui.h"
/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly_Remotor
 * SBUS遥控器任务驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/6/1
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

// SBUS解码矩阵 - 用于从25字节帧中解析16个11位通道数据
static const struct {
    u8 byte;
    u8 rshift;
    u8 mask;
    u8 lshift;
} sbus_decoder[SBUS_CHANNEL_NUM][3] = {
    /*  0 */ { { 0, 0, 0xff, 0 }, { 1, 0, 0x07, 8 }, { 0, 0, 0x00, 0 } },
    /*  1 */ { { 1, 3, 0x1f, 0 }, { 2, 0, 0x3f, 5 }, { 0, 0, 0x00, 0 } },
    /*  2 */ { { 2, 6, 0x03, 0 }, { 3, 0, 0xff, 2 }, { 4, 0, 0x01, 10 } },
    /*  3 */ { { 4, 1, 0x7f, 0 }, { 5, 0, 0x0f, 7 }, { 0, 0, 0x00, 0 } },
    /*  4 */ { { 5, 4, 0x0f, 0 }, { 6, 0, 0x7f, 4 }, { 0, 0, 0x00, 0 } },
    /*  5 */ { { 6, 7, 0x01, 0 }, { 7, 0, 0xff, 1 }, { 8, 0, 0x03, 9 } },
    /*  6 */ { { 8, 2, 0x3f, 0 }, { 9, 0, 0x1f, 6 }, { 0, 0, 0x00, 0 } },
    /*  7 */ { { 9, 5, 0x07, 0 }, { 10, 0, 0xff, 3 }, { 0, 0, 0x00, 0 } },
    /*  8 */ { { 11, 0, 0xff, 0 }, { 12, 0, 0x07, 8 }, { 0, 0, 0x00, 0 } },
    /*  9 */ { { 12, 3, 0x1f, 0 }, { 13, 0, 0x3f, 5 }, { 0, 0, 0x00, 0 } },
    /* 10 */ { { 13, 6, 0x03, 0 }, { 14, 0, 0xff, 2 }, { 15, 0, 0x01, 10 } },
    /* 11 */ { { 15, 1, 0x7f, 0 }, { 16, 0, 0x0f, 7 }, { 0, 0, 0x00, 0 } },
    /* 12 */ { { 16, 4, 0x0f, 0 }, { 17, 0, 0x7f, 4 }, { 0, 0, 0x00, 0 } },
    /* 13 */ { { 17, 7, 0x01, 0 }, { 18, 0, 0xff, 1 }, { 19, 0, 0x03, 9 } },
    /* 14 */ { { 19, 2, 0x3f, 0 }, { 20, 0, 0x1f, 6 }, { 0, 0, 0x00, 0 } },
    /* 15 */ { { 20, 5, 0x07, 0 }, { 21, 0, 0xff, 3 }, { 0, 0, 0x00, 0 } }
};

// 静态变量
static sbus_data_t sbus_data;					// SBUS数据
static u8 sbus_frame[SBUS_FRAME_SIZE];			// SBUS帧缓冲区
static u8 sbus_buffer[SBUS_BUFFER_SIZE];		// SBUS接收缓冲区
static volatile u16 sbus_rx_index = 0;			// SBUS接收索引
static volatile bool sbus_frame_received = false;	// SBUS帧接收完成标志

// RC通道6解锁控制相关变量
static u8 rc_ch6_last_state = 1;            // 上次通道6状态 (1=上锁, 0=解锁)
static bool rc_lock_state_changed = false;  // 解锁状态变化标志

// RC摇杆方向控制相关变量
static u8 rc_joystick1_last_state = 0;        // 摇杆1上次状态 (用于边沿检测)
static u8 rc_joystick2_last_state = 0;        // 摇杆2上次状态 (用于边沿检测)
static bool rc_joy1_havebackToCenter = true;  // 摇杆1是否回中
static bool rc_joy2_havebackToCenter = true;  // 摇杆2是否回中

// RC按键状态控制相关变量
static u8 rc_ch5_last_state = 1;      // 通道5上次状态 (3段开关)
static u8 rc_ch6_key_last_state = 1;  // 通道6按键功能上次状态 (2段开关)
static u8 rc_ch7_last_state = 1;      // 通道7上次状态 (2段开关)
static u8 rc_ch8_last_state = 1;      // 通道8上次状态 (2段开关)
static u8 rc_key_state = 0;           // RC按键状态

static u8 current_ch5_state, current_ch6_key_state, current_ch7_state, current_ch8_state;
// SBUS串口初始化 (USART1: PA9-TX, PA10-RX)
static void sbusInit(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// 使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	// 配置PA9 (TX)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		// 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 配置PA10 (RX)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 配置NVIC中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	// 抢占优先级2 (高优先级)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		// 子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// 配置串口参数 (SBUS: 100000bps, 8E2 - 8位数据，偶校验，2停止位)
	USART_InitStructure.USART_BaudRate = baudrate;			// 波特率100000
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;	// 9位字长(含校验位)
	USART_InitStructure.USART_StopBits = USART_StopBits_2;		// 2停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;		// 偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART1, &USART_InitStructure);
	
	// 使能接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	// 使能空闲中断
	
	// 使能串口
	USART_Cmd(USART1, ENABLE);
	
	// 初始化数据结构
	memset(&sbus_data, 0, sizeof(sbus_data));
	memset(sbus_frame, 0, sizeof(sbus_frame));
	memset(sbus_buffer, 0, sizeof(sbus_buffer));
	sbus_rx_index = 0;
	sbus_frame_received = false;
}

// SBUS数据解析
static bool sbus_decode(void)
{
	// 检查帧头和帧尾
	if (sbus_frame[0] != SBUS_START_BYTE || sbus_frame[24] != SBUS_END_BYTE) {
		return false;
	}
	
	// 解析16个通道数据 (每个通道11位)
	for (u8 channel = 0; channel < SBUS_CHANNEL_NUM; channel++) {
		u16 value = 0;
		
		// 使用解码矩阵提取通道数据
		for (u8 pick = 0; pick < 3; pick++) {
			if (sbus_decoder[channel][pick].mask != 0) {
				u16 piece = sbus_frame[1 + sbus_decoder[channel][pick].byte];
				piece >>= sbus_decoder[channel][pick].rshift;
				piece &= sbus_decoder[channel][pick].mask;
				piece <<= sbus_decoder[channel][pick].lshift;
				value |= piece;
			}
		}
		
		// 将0-2047的原始值转换为1000-2000的PWM值
		sbus_data.channels[channel] = (u16)((value * 0.625f) + 880);
	}
	
	// 解析标志位 (第23字节)
	sbus_data.failsafe = (sbus_frame[23] & 0x08) ? 1 : 0;	// 失控保护
	sbus_data.frameLost = (sbus_frame[23] & 0x04) ? 1 : 0;	// 丢帧标志
	sbus_data.dataReady = 1;	// 数据就绪
	
	return true;
}

// 处理接收缓冲区中的SBUS数据
static void sbus_process_buffer(void)
{
	u16 i;
	
	// 确保有足够的数据进行处理
	if (sbus_rx_index < SBUS_FRAME_SIZE) {
		sbus_rx_index = 0;
		sbus_frame_received = false;
		return;
	}
	
	// 查找SBUS帧起始字节
	for (i = 0; i <= sbus_rx_index - SBUS_FRAME_SIZE; i++) {
		if (sbus_buffer[i] == SBUS_START_BYTE) {
			// 检查是否是完整的SBUS帧
			if (i + SBUS_FRAME_SIZE <= sbus_rx_index) {
				// 复制完整帧到解析缓冲区
				memcpy(sbus_frame, &sbus_buffer[i], SBUS_FRAME_SIZE);
				
				// 尝试解析SBUS数据
				if (sbus_decode()) {
					// 解析成功，跳出循环
					break;
				}
			}
		}
	}
	
	// 重置接收缓冲区
	sbus_rx_index = 0;
	sbus_frame_received = false;
}

// USART1中断服务程序 - 新的实现
void USART1_IRQHandler(void)
{
	u8 ch;
	
	// 接收中断处理
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		// 读取第一个字节
		ch = USART_ReceiveData(USART1);
		
		// 防止缓冲区溢出
		if (sbus_rx_index < SBUS_BUFFER_SIZE) {
			sbus_buffer[sbus_rx_index++] = ch;
		} else {
			// 缓冲区满了，重新开始
			sbus_rx_index = 0;
			sbus_buffer[sbus_rx_index++] = ch;
		}
		
		// 持续读取，直到没有更多数据
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
			ch = USART_ReceiveData(USART1);
			
			// 防止缓冲区溢出
			if (sbus_rx_index < SBUS_BUFFER_SIZE) {
				sbus_buffer[sbus_rx_index++] = ch;
			} else {
				// 缓冲区满了，重新开始
				sbus_rx_index = 0;
				sbus_buffer[sbus_rx_index++] = ch;
			}
		}
	}
	
	// 空闲中断处理 - 数据接收完成，开始解析
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
		// 读SR和DR寄存器清除中断标志
		(void)USART1->SR;
		(void)USART1->DR;
		
		// 设置帧接收完成标志，在主任务中处理
		if (sbus_rx_index > 0) {
			sbus_frame_received = true;
		}
	}
	
	// 清除其他错误标志
	if (USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET) {
		USART_ClearFlag(USART1, USART_FLAG_FE);
	}
	if (USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET) {
		USART_ClearFlag(USART1, USART_FLAG_PE);
	}
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
}

// 遥控器任务初始化
void rcTaskInit(void)
{
	// 初始化SBUS串口 (100000 bps)
	sbusInit(100000);
}

// 遥控器任务主循环
void rcTask(void* param)
{
	static u32 lastDataTime = 0;
	
	while (1) {
		u32 currentTime = xTaskGetTickCount();
		
		// 检查是否有新的SBUS帧需要处理
		if (sbus_frame_received) {
			// 禁用中断，确保数据一致性
			__disable_irq();
			sbus_process_buffer();
			__enable_irq();
			
			// 更新最后接收时间
			lastDataTime = currentTime;
		}
		
		// 检查SBUS数据超时
		if (sbus_data.dataReady) {
			if (currentTime - lastDataTime > 100) {	// 100ms超时
				// SBUS数据超时，清除数据就绪标志
				sbus_data.dataReady = 0;
				sbus_data.failsafe = 1;
			}
		}
                // 更新RC按键状态
                updateRCKeyState();

                // 任务延时
                vTaskDelay(5);  // 5ms，提高响应速度
        }
}

// 获取SBUS数据
sbus_data_t* getSbusData(void)
{
	return &sbus_data;
}

// 检查SBUS数据是否就绪
u8 isSbusDataReady(void)
{
	return sbus_data.dataReady;
}

// RC数据转换为飞控百分比数据
void getJoystickFromRc(joystickFlyf_t *percent)
{
	// 如果SBUS数据未就绪，返回中性值
	if (!sbus_data.dataReady || sbus_data.failsafe) {
		percent->thrust = 0.0f;
		percent->roll = 0.0f;
		percent->pitch = 0.0f;
		percent->yaw = 0.0f;
		return;
	}
	
	// SBUS通道映射 (根据实际遥控器通道顺序调整)
	// 通常: CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw
	u16 ch_roll = sbus_data.channels[0];		// 横滚 (副翼)
	u16 ch_pitch = sbus_data.channels[1];		// 俯仰 (升降)
	u16 ch_throttle = sbus_data.channels[2];	// 油门
	u16 ch_yaw = sbus_data.channels[3];			// 偏航 (方向)
	
	// 将PWM值转换为百分比值，使用宏定义便于调整
	
	// 油门通道 (0到1.0)
	if (ch_throttle <= RC_THROTTLE_MIN) {
		percent->thrust = 0.0f;
	} else if (ch_throttle >= RC_THROTTLE_MAX) {
		percent->thrust = 1.0f;
	} else {
		percent->thrust = (float)(ch_throttle - RC_THROTTLE_MIN) / RC_THROTTLE_RANGE;
	}
	
	// 横滚通道 (-1.0到1.0) - 对称范围
	if (ch_roll <= RC_ROLL_CHANNEL_MIN) {
		percent->roll = -1.0f;
	} else if (ch_roll >= RC_ROLL_CHANNEL_MAX) {
		percent->roll = 1.0f;
	} else if (ch_roll >= RC_ROLL_CHANNEL_CENTER) {
		// 向上范围 (0到1.0)
		percent->roll = (float)(ch_roll - RC_ROLL_CHANNEL_CENTER) / RC_ROLL_CHANNEL_RANGE_UP;
	} else {
		// 向下范围 (-1.0到0)
		percent->roll = -(float)(RC_ROLL_CHANNEL_CENTER - ch_roll) / RC_ROLL_CHANNEL_RANGE_DOWN;
	}
	
	// 俯仰通道 (-1.0到1.0) - 非对称范围，需要分别计算
	if (ch_pitch <= RC_PITCH_CHANNEL_MIN) {
		percent->pitch = -1.0f;
	} else if (ch_pitch >= RC_PITCH_CHANNEL_MAX) {
		percent->pitch = 1.0f;
	} else if (ch_pitch >= RC_PITCH_CHANNEL_CENTER) {
		// 向上范围 (0到1.0)
		percent->pitch = (float)(ch_pitch - RC_PITCH_CHANNEL_CENTER) / RC_PITCH_CHANNEL_RANGE_UP;
	} else {
		// 向下范围 (-1.0到0)
		percent->pitch = -(float)(RC_PITCH_CHANNEL_CENTER - ch_pitch) / RC_PITCH_CHANNEL_RANGE_DOWN;
	}
	
	// 偏航通道 (-1.0到1.0) - 对称范围
	if (ch_yaw <= RC_YAW_CHANNEL_MIN) {
		percent->yaw = -1.0f;
	} else if (ch_yaw >= RC_YAW_CHANNEL_MAX) {
		percent->yaw = 1.0f;
	} else if (ch_yaw >= RC_YAW_CHANNEL_CENTER) {
		// 向上范围 (0到1.0)
		percent->yaw = (float)(ch_yaw - RC_YAW_CHANNEL_CENTER) / RC_YAW_CHANNEL_RANGE_UP;
	} else {
		// 向下范围 (-1.0到0)
		percent->yaw = -(float)(RC_YAW_CHANNEL_CENTER - ch_yaw) / RC_YAW_CHANNEL_RANGE_DOWN;
	}
	
	// 添加死区处理，使用宏定义
	const float deadzone = RC_DEADZONE_PERCENT;
	
	if (percent->roll > -deadzone && percent->roll < deadzone) {
		percent->roll = 0.0f;
	}
	if (percent->pitch > -deadzone && percent->pitch < deadzone) {
		percent->pitch = 0.0f;
	}
	if (percent->yaw > -deadzone && percent->yaw < deadzone) {
		percent->yaw = 0.0f;
	}
	
	// 限制范围到[-1.0, 1.0] (对于roll/pitch/yaw) 和 [0.0, 1.0] (对于thrust)
	if (percent->roll < -1.0f) percent->roll = -1.0f;
	if (percent->roll > 1.0f) percent->roll = 1.0f;
	if (percent->pitch < -1.0f) percent->pitch = -1.0f;
	if (percent->pitch > 1.0f) percent->pitch = 1.0f;
	if (percent->yaw < -1.0f) percent->yaw = -1.0f;
	if (percent->yaw > 1.0f) percent->yaw = 1.0f;
	if (percent->thrust < 0.0f) percent->thrust = 0.0f;
	if (percent->thrust > 1.0f) percent->thrust = 1.0f;
}

// RC按键状态检测 (使用通道5=3段开关, 通道6/7=2段开关) 替代getKeyState
u8 getRcPaddle(void) {
  u8 temp;
  temp = rc_key_state;
  rc_key_state = 0;  // 读取清除，避免重复触发
  return temp;
}

// RC按键状态更新 (在任务中调用)
void updateRCKeyState(void) {
  // 如果SBUS数据未就绪，不更新按键状态
  if (!sbus_data.dataReady || sbus_data.failsafe) {
    return;
  }

  // 通道5 - 3段开关 (通道4，数组从0开始)
  if (sbus_data.channels[4] < 1200) {
    current_ch5_state = 0;  // 下位
  } else if (sbus_data.channels[4] > 1800) {
    current_ch5_state = 2;  // 上位
  } else {
    current_ch5_state = 1;  // 中位
  }

  // 通道6 - 2段开关 (用于按键功能，不是解锁)
  if (sbus_data.channels[5] < 1500) {
    current_ch6_key_state = 0;  // 下位
  } else {
    current_ch6_key_state = 1;  // 上位
  }

  // 通道7 - 2段开关 (通道6，数组从0开始)
  if (sbus_data.channels[6] < 1500) {
    current_ch7_state = 0;  // 下位
  } else {
    current_ch7_state = 1;  // 上位
  }

  // 通道8 - 2段开关 (通道7，数组从0开始)
  if (sbus_data.channels[7] < 1500) {
    current_ch8_state = 0;  // 下位
  } else {
    current_ch8_state = 1;  // 上位
  }

  // 检测通道5状态变化
  if (current_ch5_state != rc_ch5_last_state) {
    switch (current_ch5_state) {
      case 0:
        rc_key_state = KEY_CH5_DOWN;
        break;
      case 1:
        rc_key_state = KEY_CH5_MID;
        break;
      case 2:
        rc_key_state = KEY_CH5_UP;
        break;
    }
    rc_ch5_last_state = current_ch5_state;
  }

  // 检测通道6按键状态变化
  if (current_ch6_key_state != rc_ch6_key_last_state) {
    if (current_ch6_key_state == 0) {
      rc_key_state = KEY_CH6_DOWN;
    } else {
      rc_key_state = KEY_CH6_UP;
    }
    rc_ch6_key_last_state = current_ch6_key_state;
  }

  // 检测通道7状态变化
  if (current_ch7_state != rc_ch7_last_state) {
    if (current_ch7_state == 0) {
      rc_key_state = KEY_CH7_DOWN;
    } else {
      rc_key_state = KEY_CH7_UP;
    }
    rc_ch7_last_state = current_ch7_state;
  }

  // 检测通道8状态变化
  if (current_ch8_state != rc_ch8_last_state) {
    if (current_ch8_state == 0) {
      rc_key_state = KEY_CH8_DOWN;
    } else {
      rc_key_state = KEY_CH8_UP;
    }
    rc_ch8_last_state = current_ch8_state;
  }
}

rc_channel_position_e getThreeChannelState(u8 ch)
{
	rc_channel_position_e state = UNDEFINED_POSITION;
	if (ch == 5) {
		if (rc_ch5_last_state == LOW_POSITION) {
			state = LOW_POSITION;
		} else if (rc_ch5_last_state == MID_POSITION) {
			state = MID_POSITION;
		} else {
			state = HIGH_POSITION;
		}
  	}
  return state;
}

// 根据RC通道数据获取摇杆2方向
enum dir_e getRCJoystick2Dir(void)
{
	enum dir_e ret = CENTER;
	static bool havebackToCenter = true;
	
	// 如果SBUS数据未就绪，返回中心位置
	if (!sbus_data.dataReady || sbus_data.failsafe) {
		return CENTER;
	}
	
	// 获取摇杆2的pitch和roll通道值
	u16 ch_pitch = sbus_data.channels[1];  // 俯仰通道 (升降)
	u16 ch_roll = sbus_data.channels[0];   // 横滚通道 (副翼)
	
	// 计算相对于中心位置的偏移
	s16 pitch_offset = ch_pitch - RC_PITCH_CHANNEL_CENTER;
	s16 roll_offset = ch_roll - RC_ROLL_CHANNEL_CENTER;
	
	// 死区阈值 - 使用通道范围的10%作为死区
	// Roll通道范围500，Pitch通道范围约430，取较小值确保一致性
	const s16 deadzone = 50;  // 50对应通道范围的10%左右
	
	if(havebackToCenter == true) // 摇杆回到过中间位置
	{	
		if(pitch_offset > deadzone)
			ret = FORWARD;
		else if(pitch_offset < -deadzone)
			ret = BACK;
		
		if(ret==BACK && roll_offset > deadzone)
			ret = BACK_RIGHT;
		else if(ret==BACK && roll_offset < -deadzone)
			ret = BACK_LEFT;
		else if(roll_offset > deadzone)
			ret = RIGHT;
		else if(roll_offset < -deadzone)
			ret = LEFT;

		havebackToCenter = false; // 摇杆离开了中间位置
		if(ret == CENTER) // 摇杆依然在中间位置
			havebackToCenter = true;
	}
	else if( pitch_offset >= -deadzone && pitch_offset <= deadzone &&
			 roll_offset >= -deadzone && roll_offset <= deadzone 
		   ) // 摇杆离开了中间位置，现在查询摇杆是否回中
	{
		havebackToCenter = true;
		ret = CENTER;
	}
	
	return ret;
}