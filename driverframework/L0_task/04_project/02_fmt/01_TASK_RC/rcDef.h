#ifndef __RC_DEF_H
#define __RC_DEF_H

#define  LOW_SPEED_THRUST   (95.0)
#define  LOW_SPEED_PITCH    (10.0)
#define  LOW_SPEED_ROLL     (10.0)

#define  MID_SPEED_THRUST   (95.0)
#define  MID_SPEED_PITCH    (18.0)
#define  MID_SPEED_ROLL     (18.0)

#define  HIGH_SPEED_THRUST  (95.0)
#define  HIGH_SPEED_PITCH   (30.0)
#define  HIGH_SPEED_ROLL    (30.0)

#define  MIN_THRUST			(25.0)
#define  ALT_THRUST		    (100.0)
#define  MAX_YAW			(200.0)


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

enum flightSpeed
{
	LOW_SPEED,
	MID_SPEED,
	HIGH_SPEED,
};

#endif 