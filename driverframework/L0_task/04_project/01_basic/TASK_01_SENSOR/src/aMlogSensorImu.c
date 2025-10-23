#include "aMlogSensorImu.h"

/* Mlog IMU data structure - redesigned */
typedef struct {
    uint32_t timestamp;
    float acc_filter_before[3];  // acceleration data before filtering
    float acc_filter_after[3];   // acceleration data after filtering
    float gyro_filter_before[3]; // gyro data before filtering
    float gyro_filter_after[3];  // gyro data after filtering
} mlogImuData_t;

void mlogImuSetEnable(uint8_t enable);

#ifdef PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_EN
static void mlogImuStartCb(void);

/* Mlog bus definition for IMU data - redefined */
static mlog_elem_t Sensor_IMU_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(acc_filter_before, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(acc_filter_after, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filter_before, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filter_after, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(Sensor_IMU, Sensor_IMU_Elems);

/* Static variables */
static mlogImuData_t mlog_imu_data = {0};
static int Sensor_IMU_ID = -1;
static uint8_t mlog_push_en = 0;
#ifdef PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_FREQ
static rt_tick_t mlog_last_tick = 0;      // Last log timestamp in ticks
static rt_tick_t mlog_min_interval = 0;   // Minimum interval in ticks
#endif

/**
 * @brief Initialize mlog IMU functionality
 */
void mlogImuInit(void) {
    /* Initialize mlog bus ID for sensor data */
    Sensor_IMU_ID = mlog_get_bus_id("Sensor_IMU");
    if (Sensor_IMU_ID < 0) {
        rt_kprintf("Failed to get mlog bus ID for Sensor_IMU\n");
    } else {
        rt_kprintf("Sensor_IMU mlog bus ID: %d\n", Sensor_IMU_ID);
    }
    
#ifdef PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_FREQ
    /* Calculate minimum interval in ticks for the configured frequency */
    rt_tick_t ticks_per_second = RT_TICK_PER_SECOND;
    mlog_min_interval = ticks_per_second / PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_FREQ;
    rt_kprintf("Sensor_IMU mlog frequency: %d Hz, min interval: %d ticks\n", 
               PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_FREQ, mlog_min_interval);
#endif
    
    /* Register mlog start callback */
    mlog_register_callback(MLOG_CB_START, mlogImuStartCb);
}

/**
 * @brief Mlog start callback - internal function
 */
static void mlogImuStartCb(void) {
    mlogImuSetEnable(1);
}

/**
 * @brief Set mlog enable status
 * @param enable 1-enable, 0-disable
 */
void mlogImuSetEnable(uint8_t enable) {
    mlog_push_en = enable;
}

/**
 * @brief Copy acceleration data to mlog buffer
 * @param acc_before acceleration data before filtering
 * @param acc_after acceleration data after filtering
 */
void mlogImuCopyAccData(const Axis3f* acc_before, const Axis3f* acc_after) {
    if (acc_before != RT_NULL) {
        mlog_imu_data.acc_filter_before[0] = acc_before->x;
        mlog_imu_data.acc_filter_before[1] = acc_before->y;
        mlog_imu_data.acc_filter_before[2] = acc_before->z;
    }
    
    if (acc_after != RT_NULL) {
        mlog_imu_data.acc_filter_after[0] = acc_after->x;
        mlog_imu_data.acc_filter_after[1] = acc_after->y;
        mlog_imu_data.acc_filter_after[2] = acc_after->z;
    }
}

/**
 * @brief Copy gyro data to mlog buffer
 * @param gyro_before gyro data before filtering
 * @param gyro_after gyro data after filtering
 */
void mlogImuCopyGyroData(const Axis3f* gyro_before, const Axis3f* gyro_after) {
    if (gyro_before != RT_NULL) {
        mlog_imu_data.gyro_filter_before[0] = gyro_before->x;
        mlog_imu_data.gyro_filter_before[1] = gyro_before->y;
        mlog_imu_data.gyro_filter_before[2] = gyro_before->z;
    }
    
    if (gyro_after != RT_NULL) {
        mlog_imu_data.gyro_filter_after[0] = gyro_after->x;
        mlog_imu_data.gyro_filter_after[1] = gyro_after->y;
        mlog_imu_data.gyro_filter_after[2] = gyro_after->z;
    }
}

/**
 * @brief Push data to mlog
 * @param timestamp timestamp
 */
void mlogImuPushData(uint32_t timestamp) {
    mlog_imu_data.timestamp = timestamp;
    
    if (Sensor_IMU_ID >= 0 && mlog_push_en) {
#ifdef PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_FREQ
        /* Check if enough time has passed since last log */
        rt_tick_t current_tick = rt_tick_get();
        rt_tick_t elapsed = current_tick - mlog_last_tick;
        
        if (elapsed >= mlog_min_interval) {
            mlog_push_msg((uint8_t*)&mlog_imu_data, Sensor_IMU_ID, sizeof(mlogImuData_t));
            mlog_last_tick = current_tick;
        }
#else
        /* No frequency control, push immediately */
        mlog_push_msg((uint8_t*)&mlog_imu_data, Sensor_IMU_ID, sizeof(mlogImuData_t));
#endif
    }
}

#else /* PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_EN not defined */

/* Empty implementations when mlog is not enabled */
void mlogImuInit(void) {
    /* Do nothing when mlog is disabled */
}

void mlogImuSetEnable(uint8_t enable) {
    RT_UNUSED(enable);
    /* Do nothing when mlog is disabled */
}

void mlogImuCopyAccData(const Axis3f* acc_before, const Axis3f* acc_after) {
    RT_UNUSED(acc_before);
    RT_UNUSED(acc_after);
    /* Do nothing when mlog is disabled */
}

void mlogImuCopyGyroData(const Axis3f* gyro_before, const Axis3f* gyro_after) {
    RT_UNUSED(gyro_before);
    RT_UNUSED(gyro_after);
    /* Do nothing when mlog is disabled */
}

void mlogImuPushData(uint32_t timestamp) {
    RT_UNUSED(timestamp);
    /* Do nothing when mlog is disabled */
}

#endif /* PROJECT_MINIFLY_TASK_SENSOR_MLOG_IMU_EN */
