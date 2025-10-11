#include "aMlogStabilize.h"

/* Mlog stabilizer angle rate data structure */
typedef struct {
    uint32_t timestamp;
    float rate_desired[3];  // desired rate: roll, pitch, yaw (deg/s)
    float rate_current[3];  // current rate: roll, pitch, yaw (deg/s)
} mlogStabilizerAngleRateData_t;

void mlogStabilizerSetEnable(uint8_t enable);
static void mlogStabilizerStartCb(void);

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_EN
/* Mlog bus definition for stabilizer angle rate data */
static mlog_elem_t MF_Stab_AngleRate_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(rate_desired, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(rate_current, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(MF_Stab_AngleRate, MF_Stab_AngleRate_Elems);

/* Static variables */
static mlogStabilizerAngleRateData_t mlog_stabilizer_angle_rate_data = {0};
static int MF_Stab_AngleRate_ID = -1;
static uint8_t mlog_stabilizer_push_en = 0;

/**
 * @brief Initialize mlog stabilizer functionality
 */
void mlogStabilizerInit(void) {
    /* Initialize mlog bus ID for stabilizer angle rate data */
    MF_Stab_AngleRate_ID = mlog_get_bus_id("MF_Stab_AngleRate");
    if (MF_Stab_AngleRate_ID < 0) {
        rt_kprintf("Failed to get mlog bus ID for MF_Stab_AngleRate\n");
    } else {
        rt_kprintf("MF_Stab_AngleRate mlog bus ID: %d\n", MF_Stab_AngleRate_ID);
    }
    
    /* Register mlog start callback */
    mlog_register_callback(MLOG_CB_START, mlogStabilizerStartCb);
}

/**
 * @brief Mlog start callback - internal function
 */
static void mlogStabilizerStartCb(void) {
    mlogStabilizerSetEnable(1);
}

/**
 * @brief Set mlog enable status
 * @param enable 1-enable, 0-disable
 */
void mlogStabilizerSetEnable(uint8_t enable) {
    mlog_stabilizer_push_en = enable;
}

/**
 * @brief Copy angle rate data to mlog buffer
 * @param rate_desired desired rate data (roll, pitch, yaw)
 * @param rate_current current rate data (roll, pitch, yaw)
 */
void mlogStabilizerCopyAngleRateData(const attitude_t* rate_desired, const attitude_t* rate_current) {
    if (rate_desired != RT_NULL) {
        mlog_stabilizer_angle_rate_data.rate_desired[0] = rate_desired->roll;
        mlog_stabilizer_angle_rate_data.rate_desired[1] = rate_desired->pitch;
        mlog_stabilizer_angle_rate_data.rate_desired[2] = rate_desired->yaw;
    }
    
    if (rate_current != RT_NULL) {
        mlog_stabilizer_angle_rate_data.rate_current[0] = rate_current->roll;
        mlog_stabilizer_angle_rate_data.rate_current[1] = rate_current->pitch;
        mlog_stabilizer_angle_rate_data.rate_current[2] = rate_current->yaw;
    }
}

/**
 * @brief Push data to mlog
 * @param timestamp timestamp
 */
void mlogStabilizerPushAngleRateData(uint32_t timestamp) {
    mlog_stabilizer_angle_rate_data.timestamp = timestamp;
    
    if (MF_Stab_AngleRate_ID >= 0 && mlog_stabilizer_push_en) {
        mlog_push_msg((uint8_t*)&mlog_stabilizer_angle_rate_data, MF_Stab_AngleRate_ID, sizeof(mlogStabilizerAngleRateData_t));
    }
}

#else /* PROJECT_MINIFLY_TASK_STABLIZE_MLOG_EN not defined */

/* Empty implementations when mlog is not enabled */
void mlogStabilizerInit(void) {
    /* Do nothing when mlog is disabled */
}

void mlogStabilizerSetEnable(uint8_t enable) {
    RT_UNUSED(enable);
    /* Do nothing when mlog is disabled */
}

void mlogStabilizerCopyAngleRateData(const attitude_t* rate_desired, const attitude_t* rate_current) {
    RT_UNUSED(rate_desired);
    RT_UNUSED(rate_current);
    /* Do nothing when mlog is disabled */
}

void mlogStabilizerPushAngleRateData(uint32_t timestamp) {
    RT_UNUSED(timestamp);
    /* Do nothing when mlog is disabled */
}

#endif /* PROJECT_MINIFLY_TASK_STABLIZE_MLOG_EN */
