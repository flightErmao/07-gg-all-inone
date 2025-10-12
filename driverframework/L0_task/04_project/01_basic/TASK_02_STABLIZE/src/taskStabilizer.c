#include "rtthread.h"
#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>
#include "stabilizerTypes.h"
#include "sensorsTypes.h"
#include "sensfusion6.h"
#include "aMcnSensorImu.h"
#include "aMcnStabilize.h"
#include "biasGyro.h"
#include "commandMinifly.h"
#include "stateControl.h"
#include "mixerControl.h"
#include "aMlogStabilize.h"
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define STABILIZER_EVENT_FLAG (1u << 0)

static rt_timer_t stabilizer_timer = RT_NULL;
static rt_event_t stabilizer_event = RT_NULL;
static state_t state_;
static setpoint_t setpoint_;
static control_t contorl_;

static bool sensorsAreCalibrated(void) { return outputGyroBiasFound(); }
static void stabilizer_timer_callback(void* parameter) { rt_event_send(stabilizer_event, STABILIZER_EVENT_FLAG); }

static int eventInit() {
  stabilizer_event = rt_event_create("stabilizer_event", RT_IPC_FLAG_FIFO);
  if (stabilizer_event == RT_NULL) {
    rt_kprintf("Failed to create stabilizer event\n");
    return -1;
  }
  return 0;
}

static int timerInit() {
  stabilizer_timer =
      rt_timer_create("stabilizer_timer", stabilizer_timer_callback, RT_NULL, MAIN_LOOP_DT, RT_TIMER_FLAG_PERIODIC);
  if (stabilizer_timer == RT_NULL) {
    rt_kprintf("Failed to create stabilizer timer\n");
    rt_event_delete(stabilizer_event);
    return -1;
  }
  return 0;
}

static void taskStabilizerInit(void) {
  eventInit();
  timerInit();
  stateControlInit();
  motorInit();
  mlogStabilizerInit();
}

static void rcAndCmdGenerate(setpoint_t* setpoint, uint32_t tick) {
#if defined(PROJECT_MINIFLY_TASK06_RC_EN) || defined(PROJECT_FMT_TASK01_RC_EN)
  if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    pilot_cmd_bus_t rc_data = {0};
    rcPilotCmdAcquire(&rc_data);
    commanderGetSetpoint(&rc_data, setpoint);
  }
#endif
}

static void flyerStateUpdate(state_t* state, uint32_t tick) {
  if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick)) {
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_DEBUGPIN_EN
      DEBUG_PIN_DEBUG3_HIGH();
#endif
      sensorData_t sensorData = {0};
      mcnSensorImuAcquire(&sensorData);
      imuUpdate(sensorData.acc_filter, sensorData.gyro_filter, state, ATTITUDE_ESTIMAT_DT);
      state->attitude.timestamp = rt_tick_get();
      state->armed = setpoint_.armed;
      mcnStatePub(state);
#ifdef PROJECT_MINIFLY_TASK_STABLIZE_DEBUGPIN_EN
      DEBUG_PIN_DEBUG3_LOW();
#endif
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

static void mlogAngleRateLog(state_t* state, uint32_t tick) {
#if defined(PROJECT_MINIFLY_TASK_STABLIZE_MLOG_EN)
  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    attitude_t rate_desired = {0};
    attitude_t rate_current = {0};

    getRateDesired(&rate_desired);
    rate_current.roll = state->gyro_filter.x;
    rate_current.pitch = state->gyro_filter.y;
    rate_current.yaw = state->gyro_filter.z;

    mlogStabilizerCopyAngleRateData(&rate_desired, &rate_current);
    mlogStabilizerPushAngleRateData(rt_tick_get());
  }
#else
  RT_UNUSED(state);
  RT_UNUSED(tick);
#endif
}

static void stabilizer_minifly_thread_entry(void* parameter) {
  uint32_t tick = 0;
  taskStabilizerInit();

  while (!sensorsAreCalibrated()) {
    rt_thread_mdelay(100);
  }
  rt_timer_start(stabilizer_timer);

  while (1) {
    rt_event_recv(stabilizer_event, STABILIZER_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
                  RT_NULL);
    rcAndCmdGenerate(&setpoint_, tick);
    flyerStateUpdate(&state_, tick);
    stateControl(&state_, &setpoint_, &contorl_, tick);
    mixerControlExcute(&contorl_, tick);
    mlogAngleRateLog(&state_, tick);
    tick++;
  }
}

static int taskStabilizerThreadAutoStart(void) {
#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

  static struct rt_thread task_tid_stabilizer_minifly;
  static rt_uint8_t task_stack_stabilizer_minifly[THREAD_STACK_SIZE];

  rt_thread_init(&task_tid_stabilizer_minifly, "stabilizer", stabilizer_minifly_thread_entry, RT_NULL,
                 task_stack_stabilizer_minifly, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_stabilizer_minifly);
  return RT_EOK;
}

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
INIT_APP_EXPORT(taskStabilizerThreadAutoStart);
#endif
