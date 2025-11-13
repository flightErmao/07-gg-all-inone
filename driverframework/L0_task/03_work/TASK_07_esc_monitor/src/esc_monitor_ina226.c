#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <rtthread.h>

#define LOG_TAG "esc_ina"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#include "INA226.h"
#include "esc_monitor_ina226.h"

#define ESC_MONITOR_INA226_THREAD_STACK_SIZE 2048
#define ESC_MONITOR_INA226_THREAD_PRIORITY   20
#define ESC_MONITOR_INA226_EVENT_SAMPLE      (1u << 0)

typedef struct {
  rt_bool_t sampling;
  rt_uint16_t index;
  float bus_voltage[ESC_MONITOR_INA226_SAMPLE_COUNT];
  float shunt_current[ESC_MONITOR_INA226_SAMPLE_COUNT];
  esc_monitor_ina_stage_result_t result;
} esc_monitor_ina_stage_context_t;

typedef struct {
  rt_timer_t timer;
  rt_thread_t worker_thread;
  rt_event_t event;
  esc_monitor_ina_stage_e active_stage;
  esc_monitor_ina_stage_context_t stages[ESC_MONITOR_INA_STAGE_MAX];
} esc_monitor_ina_sampler_t;

static esc_monitor_ina_sampler_t g_sampler = {0};
static void esc_monitor_ina226_stop_stage(esc_monitor_ina_stage_context_t* ctx, esc_monitor_ina_stage_e stage);
static void esc_monitor_ina226_worker_entry(void* parameter);

static rt_err_t esc_monitor_ina226_worker_init(void) {
  if (g_sampler.event == RT_NULL) {
    g_sampler.event = rt_event_create("ina_evt", RT_IPC_FLAG_PRIO);
    if (g_sampler.event == RT_NULL) {
      LOG_E("create event failed");
      return -RT_ENOSYS;
    }
  }

  if (g_sampler.worker_thread == RT_NULL) {
    g_sampler.worker_thread =
        rt_thread_create("ina_wk",
                         esc_monitor_ina226_worker_entry,
                         RT_NULL,
                         ESC_MONITOR_INA226_THREAD_STACK_SIZE,
                         ESC_MONITOR_INA226_THREAD_PRIORITY,
                         10);
    if (g_sampler.worker_thread == RT_NULL) {
      LOG_E("create worker thread failed");
      return -RT_ENOMEM;
    }

    rt_err_t ret = rt_thread_startup(g_sampler.worker_thread);
    if (ret != RT_EOK) {
      LOG_E("start worker thread failed(%d)", ret);
      rt_thread_delete(g_sampler.worker_thread);
      g_sampler.worker_thread = RT_NULL;
      return ret;
    }
  }

  return RT_EOK;
}

static rt_tick_t esc_monitor_ina226_period_ticks(void) {
  return rt_tick_from_millisecond(ESC_MONITOR_INA226_SAMPLE_PERIOD_MS);
}

static float esc_monitor_ina226_mean(const float* values, rt_uint16_t count) {
  if (count == 0) {
    return 0.0f;
  }
  double sum = 0.0;
  for (rt_uint16_t i = 0; i < count; ++i) {
    sum += values[i];
  }
  return (float)(sum / count);
}

static int esc_monitor_ina226_float_compare(const void* a, const void* b) {
  float diff = (*(const float*)a) - (*(const float*)b);
  if (diff > 0.0f) {
    return 1;
  }
  if (diff < 0.0f) {
    return -1;
  }
  return 0;
}

static float esc_monitor_ina226_median(const float* values, rt_uint16_t count) {
  if (count == 0) {
    return 0.0f;
  }
  float buffer[ESC_MONITOR_INA226_SAMPLE_COUNT] = {0};
  memcpy(buffer, values, count * sizeof(float));
  qsort(buffer, count, sizeof(float), esc_monitor_ina226_float_compare);
  if ((count & 0x01) != 0) {
    return buffer[count / 2];
  }
  return (buffer[(count / 2) - 1] + buffer[count / 2]) * 0.5f;
}

static void esc_monitor_ina226_finalize_stage(esc_monitor_ina_stage_context_t* ctx, esc_monitor_ina_stage_e stage) {
  esc_monitor_ina_stage_result_t* result = &ctx->result;
  rt_uint16_t count = ctx->index;

  result->sample_count = count;
  result->bus_voltage_mean = esc_monitor_ina226_mean(ctx->bus_voltage, count);
  result->shunt_current_mean = esc_monitor_ina226_mean(ctx->shunt_current, count);
  result->bus_voltage_median = esc_monitor_ina226_median(ctx->bus_voltage, count);
  result->shunt_current_median = esc_monitor_ina226_median(ctx->shunt_current, count);
  result->valid = (count == ESC_MONITOR_INA226_SAMPLE_COUNT);
  result->threshold_pass = RT_FALSE;

  if (result->valid) {
    switch (stage) {
      case ESC_MONITOR_INA_STAGE_1:
        if (result->shunt_current_median >= ESC_MONITOR_STAGE1_CURRENT_MIN &&
            result->shunt_current_median <= ESC_MONITOR_STAGE1_CURRENT_MAX) {
          result->threshold_pass = RT_TRUE;
        }
        break;
      case ESC_MONITOR_INA_STAGE_2:
        if (result->shunt_current_median >= ESC_MONITOR_STAGE2_CURRENT_MIN &&
            result->shunt_current_median <= ESC_MONITOR_STAGE2_CURRENT_MAX) {
          result->threshold_pass = RT_TRUE;
        }
        break;
      default:
        break;
    }
  }

  float power = result->bus_voltage_median * result->shunt_current_median;
  LOG_I("stage %d done, samples=%d, Vmed=%.3f, Imed=%.3f, Pmed=%.3f",
        stage, count, result->bus_voltage_median, result->shunt_current_median, power);
}

static void esc_monitor_ina226_stop_stage(esc_monitor_ina_stage_context_t* ctx, esc_monitor_ina_stage_e stage) {
  if (g_sampler.timer != RT_NULL) {
    rt_timer_stop(g_sampler.timer);
  }
  ctx->sampling = RT_FALSE;
  g_sampler.active_stage = ESC_MONITOR_INA_STAGE_MAX;
  esc_monitor_ina226_finalize_stage(ctx, stage);
}

static void esc_monitor_ina226_timer_callback(void* parameter) {
  RT_UNUSED(parameter);

  esc_monitor_ina_stage_e stage = g_sampler.active_stage;
  if (stage >= ESC_MONITOR_INA_STAGE_MAX) {
    return;
  }

  esc_monitor_ina_stage_context_t* ctx = &g_sampler.stages[stage];
  if (!ctx->sampling) {
    return;
  }

  if (g_sampler.event != RT_NULL) {
    rt_event_send(g_sampler.event, ESC_MONITOR_INA226_EVENT_SAMPLE);
  }
}

static void esc_monitor_ina226_worker_entry(void* parameter) {
  RT_UNUSED(parameter);

  for (;;) {
    rt_uint32_t received = 0;
    if (rt_event_recv(g_sampler.event,
                      ESC_MONITOR_INA226_EVENT_SAMPLE,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER,
                      &received) != RT_EOK) {
      continue;
    }

    esc_monitor_ina_stage_e stage = g_sampler.active_stage;
    if (stage >= ESC_MONITOR_INA_STAGE_MAX) {
      continue;
    }

    esc_monitor_ina_stage_context_t* ctx = &g_sampler.stages[stage];
    if (!ctx->sampling) {
      continue;
    }

    if (ctx->index >= ESC_MONITOR_INA226_SAMPLE_COUNT) {
      esc_monitor_ina226_stop_stage(ctx, stage);
      continue;
    }

    INA226Data_t data = ina226_read_data();
    ctx->bus_voltage[ctx->index] = data.bus_voltage;
    ctx->shunt_current[ctx->index] = data.shunt_current;
    ctx->index++;

    if (ctx->index >= ESC_MONITOR_INA226_SAMPLE_COUNT) {
      esc_monitor_ina226_stop_stage(ctx, stage);
    }
  }
}

void esc_monitor_ina226_reset(void) {
  if (g_sampler.timer != RT_NULL) {
    rt_timer_stop(g_sampler.timer);
  }
  g_sampler.active_stage = ESC_MONITOR_INA_STAGE_MAX;
  if (g_sampler.event != RT_NULL) {
    rt_event_control(g_sampler.event, RT_IPC_CMD_RESET, RT_NULL);
  }
  for (rt_size_t i = 0; i < ESC_MONITOR_INA_STAGE_MAX; ++i) {
    esc_monitor_ina_stage_context_t* ctx = &g_sampler.stages[i];
    memset(ctx, 0, sizeof(*ctx));
  }
}

rt_err_t esc_monitor_ina226_start_stage(esc_monitor_ina_stage_e stage) {
  if (stage >= ESC_MONITOR_INA_STAGE_MAX) {
    return -RT_ERROR;
  }

  rt_err_t init_ret = esc_monitor_ina226_worker_init();
  if (init_ret != RT_EOK) {
    return init_ret;
  }

  esc_monitor_ina_stage_context_t* ctx = &g_sampler.stages[stage];
  if (ctx->sampling) {
    LOG_W("stage %d already running", stage);
    return -RT_EBUSY;
  }

  if (g_sampler.active_stage < ESC_MONITOR_INA_STAGE_MAX &&
      g_sampler.active_stage != stage &&
      g_sampler.stages[g_sampler.active_stage].sampling) {
    LOG_W("stage %d cannot start, stage %d busy", stage, g_sampler.active_stage);
    return -RT_EBUSY;
  }

  if (g_sampler.timer == RT_NULL) {
    g_sampler.timer = rt_timer_create("ina226_tm",
                                      esc_monitor_ina226_timer_callback,
                                      RT_NULL,
                                      esc_monitor_ina226_period_ticks(),
                                      RT_TIMER_FLAG_PERIODIC);
    if (g_sampler.timer == RT_NULL) {
      LOG_E("create timer failed");
      return -RT_ENOSYS;
    }
  } else {
    rt_tick_t period = esc_monitor_ina226_period_ticks();
    rt_timer_control(g_sampler.timer, RT_TIMER_CTRL_SET_TIME, &period);
  }

  memset(ctx->bus_voltage, 0, sizeof(ctx->bus_voltage));
  memset(ctx->shunt_current, 0, sizeof(ctx->shunt_current));
  memset(&ctx->result, 0, sizeof(ctx->result));
  ctx->index = 0;
  ctx->sampling = RT_TRUE;

  g_sampler.active_stage = stage;

  rt_err_t ret = rt_timer_start(g_sampler.timer);
  if (ret != RT_EOK) {
    LOG_E("start timer failed(%d)", ret);
    ctx->sampling = RT_FALSE;
    g_sampler.active_stage = ESC_MONITOR_INA_STAGE_MAX;
    return ret;
  }

  LOG_I("stage %d sampling start", stage);
  return RT_EOK;
}

rt_bool_t esc_monitor_ina226_stage_ready(esc_monitor_ina_stage_e stage) {
  if (stage >= ESC_MONITOR_INA_STAGE_MAX) {
    return RT_FALSE;
  }
  return g_sampler.stages[stage].result.valid;
}

const esc_monitor_ina_stage_result_t* esc_monitor_ina226_get_stage_result(esc_monitor_ina_stage_e stage) {
  if (!esc_monitor_ina226_stage_ready(stage)) {
    return RT_NULL;
  }
  return &g_sampler.stages[stage].result;
}


