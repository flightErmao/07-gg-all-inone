#ifndef __STABALIZER_H
#define __STABALIZER_H
#include <stdbool.h>
#include <stdint.h>
#include "stabilizerTypes.h"

#define MAIN_LOOP_RATE RATE_1000_HZ
#define MAIN_LOOP_DT (uint32_t)(1000 / MAIN_LOOP_RATE) /*单位ms*/

#define ATTITUDE_ESTIMAT_RATE RATE_250_HZ  // 姿态解算速率
#define ATTITUDE_ESTIMAT_DT (1.0 / RATE_250_HZ)

#define POSITION_ESTIMAT_RATE RATE_250_HZ  // 位置预估速率
#define POSITION_ESTIMAT_DT (1.0 / RATE_250_HZ)

#define RATE_PID_RATE RATE_500_HZ  // 角速度环（内环）PID速率
#define RATE_PID_DT (1.0 / RATE_500_HZ)

#define ANGEL_PID_RATE ATTITUDE_ESTIMAT_RATE  // 角度环（外环）PID速率
#define ANGEL_PID_DT (1.0 / ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE POSITION_ESTIMAT_RATE  // 速度环（内环）PID速率
#define VELOCITY_PID_DT (1.0 / POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE POSITION_ESTIMAT_RATE  // 位置环（外环）PID速率
#define POSITION_PID_DT (1.0 / POSITION_ESTIMAT_RATE)

void stabilizerGetState(state_t* state);

#endif /* __STABALIZER_H */
