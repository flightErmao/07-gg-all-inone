#include "rtthread.h"
#include <rtdevice.h>
#include "sensorsTypes.h"
#include "taskMiniflyStabilizer.h"
#include "sensfusion6.h"
#include "taskMiniflySensor.h"
#include "uMCN.h"
#ifdef PROJECT_MINIFLY_TASK_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE 4096
#define THREAD_TIMESLICE 5

#define STABILIZER_EVENT_FLAG (1u << 0)

static rt_uint8_t task_stack_stabilizer_minifly[THREAD_STACK_SIZE];
static rt_timer_t stabilizer_timer = RT_NULL;
static rt_event_t stabilizer_event = RT_NULL;
static struct rt_thread task_tid_stabilizer_minifly;

static state_t state_minifly_;

MCN_DECLARE(minifly_stabilizer_state);
MCN_DEFINE(minifly_stabilizer_state, sizeof(state_t));
static McnNode_t state_sub_node = RT_NULL;

static bool sensorsAreCalibrated(void) { return true; }

static void mcnTopicInit(void) {
  rt_err_t result = mcn_advertise(MCN_HUB(minifly_stabilizer_state), RT_NULL);
  if (result != RT_EOK) {
    rt_kprintf("Failed to advertise minifly_stabilizer_state topic: %d\n", result);
  }

  state_sub_node = mcn_subscribe(MCN_HUB(minifly_stabilizer_state), RT_NULL, RT_NULL);
  if (state_sub_node == RT_NULL) {
    rt_kprintf("Failed to subscribe to minifly_stabilizer_state topic\n");
  }
}

static void stabilizer_timer_callback(void* parameter) { rt_event_send(stabilizer_event, STABILIZER_EVENT_FLAG); }

void taskStabilizerInit(void) {
  mcnTopicInit();
  stabilizer_event = rt_event_create("stabilizer_event", RT_IPC_FLAG_FIFO);
  if (stabilizer_event == RT_NULL) {
    rt_kprintf("Failed to create stabilizer event\n");
    return;
  }

  stabilizer_timer =
      rt_timer_create("stabilizer_timer", stabilizer_timer_callback, RT_NULL, MAIN_LOOP_DT, RT_TIMER_FLAG_PERIODIC);

  if (stabilizer_timer == RT_NULL) {
    rt_kprintf("Failed to create stabilizer timer\n");
    rt_event_delete(stabilizer_event);
    return;
  }
}

static void stabilizer_minifly_thread_entry(void* parameter) {
  uint32_t tick = 0;
  sensorData_t sensorData = {0};

  while (!sensorsAreCalibrated()) {
    rt_thread_mdelay(1000);
  }
  rt_timer_start(stabilizer_timer);

  while (1) {
    rt_event_recv(stabilizer_event, STABILIZER_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
                  RT_NULL);

    if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick)) {
#ifdef PROJECT_MINIFLY_TASK_DEBUGPIN_EN
      DEBUG_PIN_DEBUG3_HIGH();
#endif
      sensorsAcquire(&sensorData);
      imuUpdate(sensorData.acc_filter, sensorData.gyro_filter, &state_minifly_, ATTITUDE_ESTIMAT_DT);
      mcn_publish(MCN_HUB(minifly_stabilizer_state), &state_minifly_);
#ifdef PROJECT_MINIFLY_TASK_DEBUGPIN_EN
      DEBUG_PIN_DEBUG3_LOW();
#endif
    }

    tick++;
  }
}

void stabilizerGetState(state_t* state) {
  if (!state) return;
  if (mcn_poll(state_sub_node)) {
    mcn_copy(MCN_HUB(minifly_stabilizer_state), state_sub_node, state);
  }
}

static int taskStabilizerThreadAutoStart(void) {
  taskStabilizerInit();
  rt_thread_init(&task_tid_stabilizer_minifly, "L0_minifly_stabilizer", stabilizer_minifly_thread_entry, RT_NULL,
                 task_stack_stabilizer_minifly, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  rt_thread_startup(&task_tid_stabilizer_minifly);
  return RT_EOK;
}

#ifdef PROJECT_MINIFLY_TASK_STABLIZE_EN
INIT_APP_EXPORT(taskStabilizerThreadAutoStart);
#endif
