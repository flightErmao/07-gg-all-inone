#include "rcConvert.h"
#include "rcDef.h"
#include <stddef.h>


static float limit(float value,float min, float max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}
	return value;
}


void getLimitFromConfig(flightLimit_t *limit)
{
  enum flightSpeed speed = HIGH_SPEED;

  switch (speed) {
    case LOW_SPEED:
      limit->max_thrust = LOW_SPEED_THRUST;
      limit->max_pitch = LOW_SPEED_PITCH;
      limit->max_roll = LOW_SPEED_ROLL;
      break;
    case MID_SPEED:
      limit->max_thrust = MID_SPEED_THRUST;
      limit->max_pitch = MID_SPEED_PITCH;
      limit->max_roll = MID_SPEED_ROLL;
      break;
    case HIGH_SPEED:
      limit->max_thrust = HIGH_SPEED_THRUST;
      limit->max_pitch = HIGH_SPEED_PITCH;
      limit->max_roll = HIGH_SPEED_ROLL;
      break;
  }
}

void normalizationRcChannels(joystickPercent *percent,uint16_t *channels)
{
	if (percent == NULL) {
		return;
	}

	if(channels[0] == 1500 && channels[1] == 1500 && channels[2] == 1500 && channels[3] == 1500)
	{
		percent->thrust = 0.0f;
		percent->roll = 0.0f;
		percent->pitch = 0.0f;
		percent->yaw = 0.0f;
		return;
	}

	uint16_t ch_roll = channels[0];
	uint16_t ch_pitch = channels[1];
	uint16_t ch_throttle = channels[2];
	uint16_t ch_yaw = channels[3];

	if (ch_throttle <= RC_THROTTLE_MIN) {
		percent->thrust = 0.0f;
	} else if (ch_throttle >= RC_THROTTLE_MAX) {
		percent->thrust = 1.0f;
	} else {
		percent->thrust = (float)(ch_throttle - RC_THROTTLE_MIN) / RC_THROTTLE_RANGE;
	}

	if (ch_roll <= RC_ROLL_CHANNEL_MIN) {
		percent->roll = -1.0f;
	} else if (ch_roll >= RC_ROLL_CHANNEL_MAX) {
		percent->roll = 1.0f;
	} else if (ch_roll >= RC_ROLL_CHANNEL_CENTER) {
		percent->roll = (float)(ch_roll - RC_ROLL_CHANNEL_CENTER) / RC_ROLL_CHANNEL_RANGE_UP;
	} else {
		percent->roll = -(float)(RC_ROLL_CHANNEL_CENTER - ch_roll) / RC_ROLL_CHANNEL_RANGE_DOWN;
	}

	if (ch_pitch <= RC_PITCH_CHANNEL_MIN) {
		percent->pitch = -1.0f;
	} else if (ch_pitch >= RC_PITCH_CHANNEL_MAX) {
		percent->pitch = 1.0f;
	} else if (ch_pitch >= RC_PITCH_CHANNEL_CENTER) {
		percent->pitch = (float)(ch_pitch - RC_PITCH_CHANNEL_CENTER) / RC_PITCH_CHANNEL_RANGE_UP;
	} else {
		percent->pitch = -(float)(RC_PITCH_CHANNEL_CENTER - ch_pitch) / RC_PITCH_CHANNEL_RANGE_DOWN;
	}

	if (ch_yaw <= RC_YAW_CHANNEL_MIN) {
		percent->yaw = -1.0f;
	} else if (ch_yaw >= RC_YAW_CHANNEL_MAX) {
		percent->yaw = 1.0f;
	} else if (ch_yaw >= RC_YAW_CHANNEL_CENTER) {
		percent->yaw = (float)(ch_yaw - RC_YAW_CHANNEL_CENTER) / RC_YAW_CHANNEL_RANGE_UP;
	} else {
		percent->yaw = -(float)(RC_YAW_CHANNEL_CENTER - ch_yaw) / RC_YAW_CHANNEL_RANGE_DOWN;
	}

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

	if (percent->roll < -1.0f) percent->roll = -1.0f;
	if (percent->roll > 1.0f) percent->roll = 1.0f;

	if (percent->pitch < -1.0f) percent->pitch = -1.0f;
	if (percent->pitch > 1.0f) percent->pitch = 1.0f;

	if (percent->yaw < -1.0f) percent->yaw = -1.0f;
	if (percent->yaw > 1.0f) percent->yaw = 1.0f;

	if (percent->thrust < 0.0f) percent->thrust = 0.0f;
	if (percent->thrust > 1.0f) percent->thrust = 1.0f;
}


void remapRcThrottlePitchRoll(const joystickPercent percent, const flightLimit_t limit_temp, rcValue_t *flydata_temp)
{
	//THRUST
	flydata_temp->thrust = percent.thrust * (limit_temp.max_thrust - MIN_THRUST);
	flydata_temp->thrust += MIN_THRUST;
	flydata_temp->thrust = limit(flydata_temp->thrust, MIN_THRUST, limit_temp.max_thrust);
	
	//ROLL
	flydata_temp->roll = percent.roll * limit_temp.max_roll;
	flydata_temp->roll = limit(flydata_temp->roll, -limit_temp.max_roll, limit_temp.max_roll);

	//PITCH
	flydata_temp->pitch = percent.pitch * limit_temp.max_pitch;
	flydata_temp->pitch = limit(flydata_temp->pitch, -limit_temp.max_pitch, limit_temp.max_pitch);

	//YAW
	flydata_temp->yaw = percent.yaw * MAX_YAW;
	flydata_temp->yaw = limit(flydata_temp->yaw, -MAX_YAW, MAX_YAW);
}
