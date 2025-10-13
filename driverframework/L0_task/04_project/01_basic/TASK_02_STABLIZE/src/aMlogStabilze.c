#include "aMlogStabilize.h"
#include "stateControl.h"
#include "commandMinifly.h"
#include "aMcnStabilize.h"

/* Mlog data structures */
typedef struct {
    uint32_t timestamp;
    float rate_desired[3];  // desired rate: roll, pitch, yaw (deg/s)
    float rate_current[3];  // current rate: roll, pitch, yaw (deg/s)
} mlogStabilizerRateData_t;

typedef struct {
  uint32_t timestamp;
  float angle_desired[3];  // desired angle: roll, pitch, yaw (deg)
  float angle_current[3];  // current angle: roll, pitch, yaw (deg)
} mlogStabilizerAngleData_t;

typedef struct {
  uint32_t timestamp;
  float rc_roll;
  float rc_pitch;
  float rc_yaw;
  float rc_throttle;
  uint8_t armed;
} mlogStabilizerRcData_t;

#if defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN) || defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN) || \
    defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN)
static void mlogStabilizerStartCb(void);
#endif

/* Rate data logging */
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN
static mlog_elem_t StabilizeRate_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(rate_desired, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(rate_current, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(StabilizeRate, StabilizeRate_Elems);

static int StabilizeRate_ID = -1;
static uint8_t mlog_stabilizer_rate_push_en = 0;
#endif

/* Angle data logging */
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN
static mlog_elem_t StabilizeAngle_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(angle_desired, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(angle_current, MLOG_FLOAT, 3),
};
MLOG_BUS_DEFINE(StabilizeAngle, StabilizeAngle_Elems);

static int StabilizeAngle_ID = -1;
static uint8_t mlog_stabilizer_angle_push_en = 0;
#endif

/* RC data logging */
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN
static mlog_elem_t StabilizeRc_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32), MLOG_ELEMENT(rc_roll, MLOG_FLOAT),     MLOG_ELEMENT(rc_pitch, MLOG_FLOAT),
    MLOG_ELEMENT(rc_yaw, MLOG_FLOAT),     MLOG_ELEMENT(rc_throttle, MLOG_FLOAT), MLOG_ELEMENT(armed, MLOG_UINT8),
};
MLOG_BUS_DEFINE(StabilizeRc, StabilizeRc_Elems);

static int StabilizeRc_ID = -1;
static uint8_t mlog_stabilizer_rc_push_en = 0;
#endif

/**
 * @brief Initialize mlog stabilizer functionality
 */
void mlogStabilizerInit(void) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN
  /* Initialize mlog bus ID for stabilizer rate data */
  StabilizeRate_ID = mlog_get_bus_id("StabilizeRate");
  if (StabilizeRate_ID < 0) {
    rt_kprintf("Failed to get mlog bus ID for StabilizeRate\n");
  } else {
    rt_kprintf("StabilizeRate mlog bus ID: %d\n", StabilizeRate_ID);
  }
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN
  /* Initialize mlog bus ID for stabilizer angle data */
  StabilizeAngle_ID = mlog_get_bus_id("StabilizeAngle");
  if (StabilizeAngle_ID < 0) {
    rt_kprintf("Failed to get mlog bus ID for StabilizeAngle\n");
  } else {
    rt_kprintf("StabilizeAngle mlog bus ID: %d\n", StabilizeAngle_ID);
  }
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN
  /* Initialize mlog bus ID for stabilizer RC data */
  StabilizeRc_ID = mlog_get_bus_id("StabilizeRc");
  if (StabilizeRc_ID < 0) {
    rt_kprintf("Failed to get mlog bus ID for StabilizeRc\n");
  } else {
    rt_kprintf("StabilizeRc mlog bus ID: %d\n", StabilizeRc_ID);
  }
#endif

#if defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN) || defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN) || \
    defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN)
  /* Register mlog start callback */
  mlog_register_callback(MLOG_CB_START, mlogStabilizerStartCb);
#endif
}

#if defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN) || defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN) || \
    defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN)
/**
 * @brief Mlog start callback - internal function
 */
static void mlogStabilizerStartCb(void) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN
  mlog_stabilizer_rate_push_en = 1;
#endif
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN
  mlog_stabilizer_angle_push_en = 1;
#endif
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN
  mlog_stabilizer_rc_push_en = 1;
#endif
}
#endif

/**
 * @brief Push rate data to mlog
 * @param rate_data rate data structure
 */
void mlogStabilizerPushRateData(const mlogStabilizerRateData_t* rate_data) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN
  if (rate_data != RT_NULL && StabilizeRate_ID >= 0 && mlog_stabilizer_rate_push_en) {
    mlog_push_msg((uint8_t*)rate_data, StabilizeRate_ID, sizeof(mlogStabilizerRateData_t));
  }
#else
  RT_UNUSED(rate_data);
#endif
}

/**
 * @brief Push angle data to mlog
 * @param angle_data angle data structure
 */
void mlogStabilizerPushAngleData(const mlogStabilizerAngleData_t* angle_data) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN
  if (angle_data != RT_NULL && StabilizeAngle_ID >= 0 && mlog_stabilizer_angle_push_en) {
    mlog_push_msg((uint8_t*)angle_data, StabilizeAngle_ID, sizeof(mlogStabilizerAngleData_t));
  }
#else
  RT_UNUSED(angle_data);
#endif
}

/**
 * @brief Push RC data to mlog
 * @param rc_data RC data structure
 */
void mlogStabilizerPushRcData(const mlogStabilizerRcData_t* rc_data) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN
  if (rc_data != RT_NULL && StabilizeRc_ID >= 0 && mlog_stabilizer_rc_push_en) {
    mlog_push_msg((uint8_t*)rc_data, StabilizeRc_ID, sizeof(mlogStabilizerRcData_t));
  }
#else
  RT_UNUSED(rc_data);
#endif
}

/**
 * @brief Unified mlog push function for all stabilizer data
 * @param tick current tick counter
 */
void mlogStabilizerPush(uint32_t tick) {
#if !defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN) && !defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN) && \
    !defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN)
  RT_UNUSED(tick);
  return;
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RATE_EN
  // Push rate data at 500Hz
  if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
    uint32_t timestamp = rt_tick_get();
    mlogStabilizerRateData_t rate_data = {0};
    rate_data.timestamp = timestamp;

    // Get rate desired and current data
    attitude_t rate_desired = {0};
    state_t current_state = {0};

    getRateDesired(&rate_desired);
    mcnStateAcquire(&current_state);

    // Fill rate data
    rate_data.rate_desired[0] = rate_desired.roll;
    rate_data.rate_desired[1] = rate_desired.pitch;
    rate_data.rate_desired[2] = rate_desired.yaw;

    rate_data.rate_current[0] = current_state.gyro_filter.x;
    rate_data.rate_current[1] = current_state.gyro_filter.y;
    rate_data.rate_current[2] = current_state.gyro_filter.z;

    mlogStabilizerPushRateData(&rate_data);
  }
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_ANGLE_EN
  if (RATE_DO_EXECUTE(ANGLE_PID_RATE, tick)) {
    uint32_t timestamp = rt_tick_get();
    mlogStabilizerAngleData_t angle_data = {0};
    angle_data.timestamp = timestamp;

    // Get angle desired and current data
    attitude_t angle_desired = {0};
    state_t current_state = {0};

    getAngleDesired(&angle_desired);
    mcnStateAcquire(&current_state);

    // Fill angle data
    angle_data.angle_desired[0] = angle_desired.roll;
    angle_data.angle_desired[1] = angle_desired.pitch;
    angle_data.angle_desired[2] = angle_desired.yaw;

    angle_data.angle_current[0] = current_state.attitude.roll;
    angle_data.angle_current[1] = current_state.attitude.pitch;
    angle_data.angle_current[2] = current_state.attitude.yaw;

    mlogStabilizerPushAngleData(&angle_data);
  }
#endif

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_MLOG_RC_EN
  // Push RC data at 100Hz
  if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    uint32_t timestamp = rt_tick_get();
    mlogStabilizerRcData_t rc_data = {0};
    rc_data.timestamp = timestamp;

    // Get RC data
    setpoint_t* setpoint = RT_NULL;
    commanderGetCurrentSetpoint(setpoint);

    // Fill RC data
    rc_data.rc_roll = setpoint->attitude.roll;
    rc_data.rc_pitch = setpoint->attitude.pitch;
    rc_data.rc_yaw = setpoint->attitude.yaw;
    rc_data.rc_throttle = setpoint->thrust;
    rc_data.armed = setpoint->armed;

    mlogStabilizerPushRcData(&rc_data);
  }
#endif
}