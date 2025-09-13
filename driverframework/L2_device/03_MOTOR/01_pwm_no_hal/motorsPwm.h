#ifndef __MOTORS_H
#define __MOTORS_H

#include <stdint.h>
#include <stdbool.h>

/* 96M主频下 8位精度输出375K PWM */
#define TIM_CLOCK_HZ 96000000
#define MOTORS_PWM_BITS 8
#define MOTORS_PWM_PERIOD ((1 << MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0

// #define ENABLE_THRUST_BAT_COMPENSATED /*使能电池油门补偿*/

#define NBR_OF_MOTORS 4
#define MOTOR_M1 0
#define MOTOR_M2 1
#define MOTOR_M3 2
#define MOTOR_M4 3

#define MOTORS_TEST_RATIO 10
#define MOTORS_TEST_ON_TIME_MS 1000
#define MOTORS_TEST_DELAY_TIME_MS 1000

void motorOrder(void);
void motorsSetRatio(uint32_t id, uint16_t ithrust);

#endif /* __MOTORS_H */
