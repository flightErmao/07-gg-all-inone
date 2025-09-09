#ifndef __RC_TASK_H
#define __RC_TASK_H	 
#include "sys.h"
#include "joystick.h"

#define SBUS_FRAME_SIZE		25		// SBUS帧长度
#define SBUS_CHANNEL_NUM	16		// SBUS通道数
#define SBUS_START_BYTE		0x0F	// SBUS起始字节
#define SBUS_END_BYTE		0x00	// SBUS结束字节
#define SBUS_BUFFER_SIZE	32		// SBUS接收缓冲区大小

// RC通道数值范围宏定义 - 可根据实际遥控器调整
// 横滚通道 (Roll)
#define RC_ROLL_CHANNEL_MIN		1000	// 通道最小值 (PWM 1000us)
#define RC_ROLL_CHANNEL_MAX		2000	// 通道最大值 (PWM 2000us)
#define RC_ROLL_CHANNEL_CENTER	1500	// 通道中位值 (PWM 1500us)
#define RC_ROLL_CHANNEL_RANGE_UP		(RC_ROLL_CHANNEL_MAX - RC_ROLL_CHANNEL_CENTER)	// 向上范围 (500)
#define RC_ROLL_CHANNEL_RANGE_DOWN	(RC_ROLL_CHANNEL_CENTER - RC_ROLL_CHANNEL_MIN)	// 向下范围 (500)

// 俯仰通道 (Pitch) - 中值不在正中间，需要分别计算上下范围
#define RC_PITCH_CHANNEL_MIN		1071	// 通道最小值 (PWM 1071us)
#define RC_PITCH_CHANNEL_MAX		1938	// 通道最大值 (PWM 1938us)
#define RC_PITCH_CHANNEL_CENTER	1512	// 通道中位值 (PWM 1512us)
#define RC_PITCH_CHANNEL_RANGE_UP		(RC_PITCH_CHANNEL_MAX - RC_PITCH_CHANNEL_CENTER)	// 向上范围 (426)
#define RC_PITCH_CHANNEL_RANGE_DOWN	(RC_PITCH_CHANNEL_CENTER - RC_PITCH_CHANNEL_MIN)	// 向下范围 (441)

// 偏航通道 (Yaw)
#define RC_YAW_CHANNEL_MIN		1071	// 通道最小值 (PWM 1000us)
#define RC_YAW_CHANNEL_MAX		1938	// 通道最大值 (PWM 2000us)
#define RC_YAW_CHANNEL_CENTER	1511	// 通道中位值 (PWM 1500us)
#define RC_YAW_CHANNEL_RANGE_UP		(RC_YAW_CHANNEL_MAX - RC_YAW_CHANNEL_CENTER)	// 向上范围 (500)
#define RC_YAW_CHANNEL_RANGE_DOWN	(RC_YAW_CHANNEL_CENTER - RC_YAW_CHANNEL_MIN)	// 向下范围 (500)

// 油门通道特殊范围 (0-100%)
#define RC_THROTTLE_MIN		1067	// 油门最小值 (0%)
#define RC_THROTTLE_MAX		1933	// 油门最大值 (100%)
#define RC_THROTTLE_RANGE		(RC_THROTTLE_MAX - RC_THROTTLE_MIN)	// 油门范围 (1000)

// 死区设置
#define RC_DEADZONE_PERCENT	0.05f	// 死区百分比 (5%)

// 按键检测阈值设置
#define RC_KEY_3WAY_DOWN_THRESHOLD	1200	// 3段开关下位阈值
#define RC_KEY_3WAY_UP_THRESHOLD		1800	// 3段开关上位阈值
#define RC_KEY_2WAY_THRESHOLD		1500	// 2段开关阈值

#define KEY_CH5_UP		10		// 通道5上位
#define KEY_CH5_MID		11		// 通道5中位  
#define KEY_CH5_DOWN		12		// 通道5下位
#define KEY_CH6_UP		13		// 通道6上位
#define KEY_CH6_DOWN		14		// 通道6下位
#define KEY_CH7_UP		15		// 通道7上位
#define KEY_CH7_DOWN		16		// 通道7下位
#define KEY_CH8_UP		17		// 通道8上位
#define KEY_CH8_DOWN		18		// 通道8下位

typedef struct {
	u16 channels[SBUS_CHANNEL_NUM];	// 16个通道数据
	u8 failsafe;					// 失控保护标志
	u8 frameLost;					// 丢帧标志
	u8 dataReady;					// 数据就绪标志
} sbus_data_t;

typedef enum {
	SBUS_STATE_WAIT_START = 0,		// 等待起始字节
	SBUS_STATE_RECV_DATA,			// 接收数据
	SBUS_STATE_RECV_END				// 接收结束字节
} sbus_state_e;

typedef enum {
	LOW_POSITION = 0,
	MID_POSITION,
	HIGH_POSITION,
	UNDEFINED_POSITION
} rc_channel_position_e;

void rcTaskInit(void);					// 遥控器任务初始化
void rcTask(void* param);				// 遥控器任务
sbus_data_t* getSbusData(void);			// 获取SBUS数据
u8 isSbusDataReady(void);				// 检查SBUS数据是否就绪
void getJoystickFromRc(joystickFlyf_t *percent);	// RC数据转飞控百分比
u8 getRCChannel6State(void);					// 获取RC通道6状态 (0=解锁, 1=上锁)
void checkRCLockControl(void);					// 检查RC解锁控制 (在任务中调用)
u8 getRcPaddle(void);							// RC通道5/6/7 替代按键状态
void updateRCKeyState(void);					// 更新RC按键状态 (在任务中调用)
bool getDisarmRcStateChanged(void);				// 获取RC解锁状态变化 (返回true表示状态已变更)
rc_channel_position_e getThreeChannelState(u8 ch);
enum dir_e getRCJoystick2Dir(void);			// 根据RC通道数据获取摇杆2方向
#endif 