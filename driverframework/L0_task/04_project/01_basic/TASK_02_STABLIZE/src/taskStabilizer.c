#include "rtthread.h"
#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>
#include "stabilizerTypes.h"
#include "sensorsTypes.h"
#include "sensfusion6.h"
#include "aMcnSensorImu.h"
#include "aMcnStabilize.h"
#include "aMlogStabilize.h"
#include "biasGyro.h"
#include "command.h"
#include "stateControl.h"
#include "mixerControl.h"
#include "taskStabilizer.h"

static state_t state_;
static setpoint_t setpoint_;
static control_t contorl_;

/**
 * @brief Get current setpoint
 * @param setpoint Pointer to setpoint structure to be filled with current values
 */
void commanderGetCurrentSetpoint(setpoint_t* setpoint) {
  if (!setpoint) return;
  *setpoint = setpoint_;
}

static void taskStabilizerInit(void) {
  stateControlInit();
  motorInit();
  mlogStabilizerInit();
}

static void rcAndCmdGenerate(setpoint_t* setpoint, uint32_t tick) {
#if defined(PROJECT_MINIFLY_TASK06_RC_EN) || defined(PROJECT_FMT_TASK01_RC_EN)
  if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
    pilot_cmd_bus_t rc_data = {0};
    rcPilotCmdAcquire(&rc_data);
    commanderGetSetpoint(&rc_data, setpoint);
  }
#endif
}

static void flyerStateUpdate(state_t* state, uint32_t tick) {
  if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick)) {
    sensorData_t sensorData = {0};
    mcnSensorImuAcquire(&sensorData);
    imuUpdate(sensorData.acc_filter, sensorData.gyro_filter, state, ATTITUDE_ESTIMAT_DT);
    state->attitude.timestamp = rt_tick_get();
    state->armed = setpoint_.armed;
    mcnStatePub(state);
  }
}

static void mixerControlExcute(control_t* control, uint32_t tick) {
  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
#if defined(L2_DEVICE_03_MOTOR_01_PWM_EN) || defined(L2_DEVICE_03_MOTOR_03_PWM_EN) || \
    defined(PROJECT_MINIFLY_TASK_DSHOT_EN)
    mixerControl(control);
#endif
  }
}

static void stabilizer_thread_entry(void* parameter) {
  uint32_t tick = 0;
  taskStabilizerInit();

  /* Wait for gyro bias calibration */
  while (!outputGyroBiasFound()) {
    rt_thread_mdelay(100);
  }

  while (1) {
    mcnWaitImuPub();
    DEBUG_PIN_DEBUG1_HIGH();
    rcAndCmdGenerate(&setpoint_, tick);
    flyerStateUpdate(&state_, tick);
    stateControl(&state_, &setpoint_, &contorl_, tick);
    mixerControlExcute(&contorl_, tick);
    mlogStabilizerPush(tick);
    DEBUG_PIN_DEBUG1_LOW();
    tick++;
  }
}

static int taskStabilizerThreadAutoStart(void) {
#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

  static struct rt_thread task_tid_stabilizer;
  static rt_uint8_t task_stack_stabilizer[THREAD_STACK_SIZE];

  rt_thread_init(&task_tid_stabilizer, "stabilizer", stabilizer_thread_entry, RT_NULL, task_stack_stabilizer,
                 THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_stabilizer);
  return RT_EOK;
}

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
INIT_APP_EXPORT(taskStabilizerThreadAutoStart);
#endif
