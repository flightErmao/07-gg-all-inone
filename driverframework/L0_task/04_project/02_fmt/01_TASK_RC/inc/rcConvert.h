#ifndef __RC_CONVERT_H
#define __RC_CONVERT_H

#include "stdint.h"

typedef struct 
{
	float roll;
	float pitch;
	float yaw;
	float thrust;
}joystickPercent;

// flightLimit 定义
typedef struct{
	float max_thrust;
	float max_pitch;
	float max_roll;
} flightLimit_t;

typedef joystickPercent rcValue_t;

void normalizationRcChannels(joystickPercent *percent,uint16_t *channels);
void getLimitFromConfig(flightLimit_t *limit);
void remapRcThrottlePitchRoll(const joystickPercent percent, const flightLimit_t limit_temp, rcValue_t *flydata_temp);

#endif


