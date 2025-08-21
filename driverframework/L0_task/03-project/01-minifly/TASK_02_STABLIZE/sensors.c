#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "config.h"
#include "config_param.h"
#include "ledseq.h"
#include "mpu6500.h"
#include "sensors.h"
#include "ak8963.h"
#include "bmp280.h"
#include "spl06.h"
#include "filters.h"
#include "FreeRTOS.h"
#include "task.h"


#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024
#define GYRO_VARIANCE_BASE				4000
#define SENSORS_ACC_SCALE_SAMPLES  		200

#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN       	8
#define SENSORS_BARO_STATUS_LEN		1
#define SENSORS_BARO_DATA_LEN		6
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)

static Axis3i16 gyroBiasBuffer[SENSORS_NBR_OF_BIAS_SAMPLES];
BiasObj	gyroBiasRunning;
static Axis3f  gyroBias;

static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static bool isInit = false;
static sensorData_t sensors;
static Axis3i16	gyroRaw;
static Axis3i16	accRaw;
static Axis3i16 magRaw;

/*��ͨ�˲�����*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static bool isMPUPresent=false;
static bool isMagPresent=false;
static bool isBaroPresent=false;

enum {IDLE, BMP280, SPL06}baroType = IDLE;

static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;

static imuDataFrame_t imuDataFrame;

static void sensorsBiasObjInit(BiasObj* bias);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);


/*�Ӷ��ж�ȡ��������*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*�Ӷ��ж�ȡ���ټ�����*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}


static void sensorsBiasObjInit(BiasObj* bias)
{
	bias_init(bias, gyroBiasBuffer, SENSORS_NBR_OF_BIAS_SAMPLES);
}

/* moved to filters module */
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		bias_calc_var_mean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}

static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

/**
 * �������������������ٶ���������
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

/**
 * �������ݷ���
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/*�������ټƺ�����������*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	accRaw.x = ax;/*�����ϴ�����λ��*/
	accRaw.y = ay;
	accRaw.z = az;
	gyroRaw.x = gx - gyroBias.x;
	gyroRaw.y = gy - gyroBias.y;
	gyroRaw.z = gz - gyroBias.z;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	
	}
	
	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	apply_axis3f_lpf(gyroLpf, &sensors.gyro);	

	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

	apply_axis3f_lpf(accLpf, &sensors.acc);
}

static void storeOneFrameImuData(u8 *data, u32 time_stamp)
{
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];
	imuDataFrame.gyroRaw.x = gx;
	imuDataFrame.gyroRaw.y = gy;
	imuDataFrame.gyroRaw.z = gz;
	imuDataFrame.accRaw.x = ax;
	imuDataFrame.accRaw.y = ay;
	imuDataFrame.accRaw.z = az;
	imuDataFrame.time_stamp = time_stamp;
}

void getOneFrameImuData(imuDataFrame_t *data)
{
	if (data != NULL)
	{
		data->gyroRaw = imuDataFrame.gyroRaw;
		data->accRaw = imuDataFrame.accRaw;
		data->time_stamp = imuDataFrame.time_stamp;
	}
}

void sensorsTask(void *param)
{
	while (1)
	{
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
			u8 dataLen = (u8) (SENSORS_MPU6500_BUFF_LEN +
				(isMagPresent ? SENSORS_MAG_BUFF_LEN : 0) +
				(isBaroPresent ? SENSORS_BARO_BUFF_LEN : 0));

			u32 time_stamp = getSysTickCnt();
			i2cdevRead(I2C1_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
			storeOneFrameImuData(buffer, time_stamp);
			
			processAccGyroMeasurements(&(buffer[0]));

			if (isMagPresent)
			{
				processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			}
			if (isBaroPresent)
			{
				processBarometerMeasurements(&(buffer[isMagPresent ?
					SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6500_BUFF_LEN]));
			}
			
			vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
			xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
			xQueueOverwrite(gyroDataQueue, &sensors.gyro);
			if (isMagPresent)
			{
				xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
			}
			if (isBaroPresent)
			{
				xQueueOverwrite(barometerDataQueue, &sensors.baro);
			}
			xTaskResumeAll();
		}
	}	
}

void sensorsAcquire(sensorData_t *sensors, const u32 tick)	
{	
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
	sensorsReadBaro(&sensors->baro);
}

void __attribute__((used)) EXTI4_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}
/*���׵�ͨ�˲�*/
// removed obsolete function after migration to filters module
/*У׼*/
bool sensorsAreCalibrated()	
{
	return gyroBiasFound;
}
/*��λ����ȡ��ȡԭʼ����*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag)
{
	*acc = accRaw;
	*gyro = gyroRaw;
	*mag = magRaw;
}

bool getIsMPU9250Present(void)
{
	bool value = isMPUPresent;
#ifdef SENSORS_ENABLE_MAG_AK8963
	value &= isMagPresent;
#endif
	return value;
}


bool getIsBaroPresent(void)
{
	return isBaroPresent;
}



