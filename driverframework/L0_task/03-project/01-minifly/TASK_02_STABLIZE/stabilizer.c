#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "sensfusion6.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_control.h"
#include "position_pid.h"
#include "flip.h"
#include "optical_flow.h"
#include "vl53lxx.h"
#include "maths.h"
#include <rtthread.h>
#include "task_sensor_minifly.h"

static bool isInit;

static setpoint_t 	setpoint;
static sensorData_t sensorData;
static state_t 		state;
static control_t 	control;

static u16 velModeTimes = 0;
static u16 absModeTimes = 0;
static float setHeight = 0.f;
static float baroLast = 0.f;
static float baroVelLpf = 0.f;


void stabilizerTask(void* param);

/* thread & event config */
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5
#define STABI_EVT_TICK 0x01

static struct rt_thread stabi_tid;
static rt_uint8_t stabi_stack[THREAD_STACK_SIZE];
static struct rt_event stabi_evt;
static rt_timer_t stabi_timer = RT_NULL;

static void stabi_timer_cb(void *parameter)
{
	RT_UNUSED(parameter);
	rt_event_send(&stabi_evt, STABI_EVT_TICK);
}

void stabilizerInit(void)
{
	if(isInit) return;

	stateControlInit();
	powerControlInit();

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= stateControlTest();
	pass &= powerControlTest();

	return pass;
}


void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height)
{
	if(velTimes != 0 && velModeTimes == 0)
	{
		baroLast = sensorData.baro.asl;
		baroVelLpf = 0.f;

		velModeTimes = velTimes;
	}
	if(absTimes != 0 && absModeTimes ==0)
	{
		setHeight = height;
		absModeTimes = absTimes;
	}		
}

static void fastAdjustPosZ(void)
{	
	if(velModeTimes > 0)
	{
		velModeTimes--;
		estRstHeight();
		
		float baroVel = (sensorData.baro.asl - baroLast) / 0.004f;
		baroLast = sensorData.baro.asl;
		baroVelLpf += (baroVel - baroVelLpf) * 0.35f;

		setpoint.mode.z = modeVelocity;
		state.velocity.z = baroVelLpf;
		setpoint.velocity.z = -1.0f * baroVelLpf;
		
		if(velModeTimes == 0)
		{
			if(getModuleID() == OPTICAL_FLOW)
				setHeight = getFusedHeight();
			else
				setHeight = state.position.z;
		}		
	}
	else if(absModeTimes > 0)
	{
		absModeTimes--;
		estRstAll();
		setpoint.mode.z = modeAbs;		
		setpoint.position.z = setHeight;
	}	
}



void getAttitudeData(attitude_t* get)
{
	get->pitch = -state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = -state.attitude.yaw;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

void getSensorData(sensorData_t* get)
{
	*get = sensorData;
}

void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos)
{
	acc->x = 1.0f * state.acc.x;
	acc->y = 1.0f * state.acc.y;
	acc->z = 1.0f * state.acc.z;
	vel->x = 1.0f * state.velocity.x;
	vel->y = 1.0f * state.velocity.y;
	vel->z = 1.0f * state.velocity.z;
	pos->x = 1.0f * state.position.x;
	pos->y = 1.0f * state.position.y;
	pos->z = 1.0f * state.position.z;
}

void stabilizerTask(void* param)
{
	u32 tick = 0;
	RT_UNUSED(param);
	while(1) 
	{
		rt_uint32_t recvd = 0;
		rt_event_recv(&stabi_evt, STABI_EVT_TICK, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recvd);

		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			Axis3f acc_tmp, gyro_tmp;
			rt_err_t ra = sensor_minifly_read_acc(&acc_tmp);
			rt_err_t rg = sensor_minifly_read_gyro(&gyro_tmp);
			if (ra == RT_EOK && rg == RT_EOK)
			{
				sensorData.acc = acc_tmp;
				sensorData.gyro = gyro_tmp;
			}
			else
			{
				sensorsAcquire(&sensorData, tick);
			}
		}

		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
		}

		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
			
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated()==true)
		{
			commanderGetSetpoint(&setpoint, &state);	
		}
		
		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			fastAdjustPosZ();
		}		
		
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			getOpFlowData(&state, 0.01f);	
		}
		
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick) && (getCommanderCtrlMode() != 0x03))
		{
			flyerFlipCheck(&setpoint, &control, &state);	
		}
		
		anomalDetec(&sensorData, &state, &control);			
		
		stateControl(&control, &sensorData, &state, &setpoint, tick);
				
		
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerControl(&control);	
		}
		
		tick++;
	}
}


static void stabi_thread_init(void)
{
	rt_event_init(&stabi_evt, "stabi_evt", RT_IPC_FLAG_PRIO);
	rt_thread_init(&stabi_tid, "t_stabi", stabilizerTask, RT_NULL, stabi_stack,
				THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
	rt_thread_startup(&stabi_tid);
	stabi_timer = rt_timer_create("stabi_tmr", stabi_timer_cb, RT_NULL, MAIN_LOOP_DT,
				RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	if (stabi_timer)
	{
		rt_timer_start(stabi_timer);
	}
}

#ifdef TASK_02_STABLIZE
INIT_APP_EXPORT(stabi_thread_init);
#endif
