#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "rtdef.h"
#include "uparam.h"
#include "flyerPidDefParam.h"

#ifdef PKG_USING_MIXER
extern void reloadMixer(void);
#endif

#define PARAM_SOURCE_MAX 8

typedef struct {
  param_list* list;
  size_t count;
} param_source_t;

static param_source_t g_param_sources[PARAM_SOURCE_MAX];
static size_t g_param_source_size = 0;

#ifdef PROJECT_MINIFLY_TASK05_PARAM_FLYER_DEFAULT_EN
static void register_param_source(param_list* list, size_t count) {
  if ((list == RT_NULL) || (count == 0)) {
    return;
  }
  RT_ASSERT(g_param_source_size < PARAM_SOURCE_MAX);
  g_param_sources[g_param_source_size].list = list;
  g_param_sources[g_param_source_size].count = count;
  g_param_source_size++;
}
#endif

static void init_param_sources(void) {
  if (g_param_source_size > 0) {
    return;
  }
#ifdef PROJECT_MINIFLY_TASK05_PARAM_FLYER_DEFAULT_EN
  register_param_source(flyerPidDefParam_list(), flyerPidDefParam_count());
#endif
}

static param_list* find_param_entry(const char* name) {
  if (!name) {
    return RT_NULL;
  }
  init_param_sources();
  for (size_t i = 0; i < g_param_source_size; i++) {
    param_source_t* source = &g_param_sources[i];
    for (size_t j = 0; j < source->count; j++) {
      if (strcmp(source->list[j].name, name) == 0) {
        return &source->list[j];
      }
    }
  }
  return RT_NULL;
}

static size_t aggregate_param_count(void) {
  init_param_sources();
  size_t total = 0;
  for (size_t i = 0; i < g_param_source_size; i++) {
    total += g_param_sources[i].count;
  }
  return total;
}

int uparam_data_init(void) {
  init_param_sources();
#ifdef PROJECT_MINIFLY_TASK05_PARAM_FLYER_DEFAULT_EN
  uparam_add_list(flyerPidDefParam_list(), flyerPidDefParam_count());
#endif
  return 0;  // 返回 0 表示成功
}

#ifdef PKG_USING_UPARAM
INIT_DEVICE_EXPORT(uparam_data_init);
#endif

rt_err_t getParam(const char* name, void* value, size_t value_size) {
  if ((name == RT_NULL) || (value == RT_NULL)) {
    return -RT_ERROR;
  }
  param_list* entry = find_param_entry(name);
  if (!entry) {
    return -RT_ERROR;
  }
  if (value_size < entry->size) {
    return -RT_ERROR;
  }
  memcpy(value, entry->address, entry->size);
  return RT_EOK;
}

rt_err_t setParam(const char* name, void* value, size_t value_size) {
  if ((name == RT_NULL) || (value == RT_NULL)) {
    return -RT_ERROR;
  }
  param_list* entry = find_param_entry(name);
  if (!entry) {
    return -RT_ERROR;
  }
  if (value_size != entry->size) {
    return -RT_ERROR;
  }
  memcpy(entry->address, value, entry->size);

// Apply motor_reverse to mixer in real time
#ifdef PKG_USING_MIXER
  if (strcmp(entry->name, "motor_reverse") == 0) {
    reloadMixer();
  }
#endif

  uint16_t flush_result = uparam_flush();
  if (flush_result > 0) {
    return RT_EOK;
  }
  return -RT_ERROR;
}

size_t getParamCount(void) { return aggregate_param_count(); }

/**
 * @brief 根据索引获取参数信息
 *
 * @param index 参数索引 (0-based)
 * @return const param_list* 参数信息指针，如果索引无效则返回NULL
 */
const param_list *getParamByIndex(size_t index) {
  init_param_sources();
  size_t remaining = index;
  for (size_t i = 0; i < g_param_source_size; i++) {
    param_source_t* source = &g_param_sources[i];
    if (remaining < source->count) {
      return &source->list[remaining];
    }
    remaining -= source->count;
  }
  return RT_NULL;
}
