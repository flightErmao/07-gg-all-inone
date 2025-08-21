#ifndef __STABILIZER_TYPES_H
#define __STABILIZER_TYPES_H
#include <stdbool.h>
#include "sensors_types.h"

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef struct  
{
	u32 timestamp;

	float roll;
	float pitch;
	float yaw;
} attitude_t;

struct  vec3_s 
{
	u32 timestamp;

	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct quaternion_s 
{
	uint32_t timestamp;

	union 
	{
		struct 
		{
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct 
		{
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

typedef struct toaMeasurement_s 
{
	int8_t senderId;
	float x, y, z;
	int64_t rx, tx;
} toaMeasurement_t;

typedef struct tdoaMeasurement_s {
  point_t anchorPosition[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct positionMeasurement_s 
{
	union 
	{
		struct 
		{
			float x;
			float y;
			float z;
		};
		float pos[3];
	};
	float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s 
{
	union 
	{
		struct 
		{
			float x;
			float y;
			float z;
		};
		float pos[3];
	};
	float distance;
	float stdDev;
} distanceMeasurement_t;

typedef struct zRange_s 
{
	uint32_t timestamp;
	float distance;
	float quality;
} zRange_t;

typedef struct flowMeasurement_s 
{
	uint32_t timestamp;
	union 
	{
		struct 
		{
			float dpixelx;
			float dpixely;
		};
		float dpixel[2];
	};
	float stdDevX;
	float stdDevY;
	float dt;
} flowMeasurement_t;


typedef struct tofMeasurement_s 
{
	uint32_t timestamp;
	float distance;
	float stdDev;
} tofMeasurement_t;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;

typedef struct
{
	Axis3f acc;
	Axis3f gyro;
	Axis3f mag;
	baro_t baro;
	point_t position;
	zRange_t zrange;
} sensorData_t;

typedef struct
{
	attitude_t attitude;
	quaternion_t attitudeQuaternion;
	point_t position;
	velocity_t velocity;
	acc_t acc;
	bool isRCLocked;
} state_t;

enum dir_e
{
	CENTER=0,
	FORWARD,
	BACK,
	LEFT,
	RIGHT,
};

typedef struct
{
	s16 roll;
	s16 pitch;
	s16 yaw;
	float thrust;
	enum dir_e flipDir;
} control_t;

typedef enum
{
	modeDisable = 0,
	modeVelocity,
	modeAbs
} mode_e;

typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_t;

typedef struct
{
	attitude_t attitude;
	attitude_t attitudeRate;
	point_t position;
	velocity_t velocity;
	mode_t mode;
	float thrust;
} setpoint_t;


#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#endif


