#include "mlogImu.h"

/* Mlog IMU data structure - redesigned */
typedef struct {
    uint32_t timestamp;
    float acc_filter_before[3];  // acceleration data before filtering
    float acc_filter_after[3];   // acceleration data after filtering
    float gyro_filter_before[3]; // gyro data before filtering
    float gyro_filter_after[3];  // gyro data after filtering
} mlogImuData_t;

void mlogImuSetEnable(uint8_t enable);
static void mlogImuStartCb(void);

#ifdef TASK_TOOL_02_SD_MLOG
/* Mlog bus definition for IMU data - redefined */
static mlog_elem_t Minifly_Sensor_IMU_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(acc_filter_before, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(acc_filter_after, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filter_before, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyro_filter_after, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(Minifly_Sensor_IMU, Minifly_Sensor_IMU_Elems);

/* Static variables */
static mlogImuData_t mlog_imu_data = {0};
static int Minifly_Sensor_IMU_ID = -1;
static uint8_t mlog_push_en = 0;

/**
 * @brief Initialize mlog IMU functionality
 */
void mlogImuInit(void) {
    /* Initialize mlog bus ID for sensor data */
    Minifly_Sensor_IMU_ID = mlog_get_bus_id("Minifly_Sensor_IMU");
    if (Minifly_Sensor_IMU_ID < 0) {
        rt_kprintf("Failed to get mlog bus ID for Minifly_Sensor_IMU\n");
    } else {
        rt_kprintf("Minifly_Sensor_IMU mlog bus ID: %d\n", Minifly_Sensor_IMU_ID);
    }
    
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
    
    if (Minifly_Sensor_IMU_ID >= 0 && mlog_push_en) {
        mlog_push_msg((uint8_t*)&mlog_imu_data, Minifly_Sensor_IMU_ID, sizeof(mlogImuData_t));
    }
}

#else /* TASK_TOOL_02_SD_MLOG not defined */

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

#endif /* TASK_TOOL_02_SD_MLOG */
