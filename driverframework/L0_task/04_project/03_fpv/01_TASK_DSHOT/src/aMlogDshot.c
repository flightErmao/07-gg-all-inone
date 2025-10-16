#include "aMlogDshot.h"

#ifdef TASK_TOOL_02_SD_MLOG
#include "mlog.h"
#endif

#ifdef PROJECT_MINIFLY_TASK_DSHOT_MLOG_EN

static void mlogDshotStartCb(void);
/* Mlog bus definition for DShot data */
static mlog_elem_t DShot_Motor_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(dshot_mapped, MLOG_UINT16, 4),
#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
    MLOG_ELEMENT_VEC(erpm, MLOG_UINT16, 4),
#endif
};
MLOG_BUS_DEFINE(DShot_Motor, DShot_Motor_Elems);

/* Static variables */
static mlogDshotData_t mlog_dshot_data = {0};
static int DShot_Motor_ID = -1;
static uint8_t mlog_push_en = 0;

/**
 * @brief Initialize mlog DShot functionality
 */
void mlogDshotInit(void) {
  /* Initialize mlog bus ID for DShot data */
  DShot_Motor_ID = mlog_get_bus_id("DShot_Motor");
  if (DShot_Motor_ID < 0) {
    rt_kprintf("Failed to get mlog bus ID for DShot_Motor\n");
  } else {
    rt_kprintf("DShot_Motor mlog bus ID: %d\n", DShot_Motor_ID);
  }

  /* Register mlog start callback */
  mlog_register_callback(MLOG_CB_START, mlogDshotStartCb);
}

/**
 * @brief Mlog start callback - internal function
 */
static void mlogDshotStartCb(void) { mlog_push_en = 1; }

/**
 * @brief Push DShot data to mlog
 * @param data pointer to mlogDshotData_t structure
 */
void mlogDshotPush(const mlogDshotData_t* data) {
  if (DShot_Motor_ID < 0 || !mlog_push_en || data == RT_NULL) {
    return;
  }

  /* Copy all data from input structure */
  mlog_dshot_data.timestamp = data->timestamp;
  mlog_dshot_data.dshot_mapped[0] = data->dshot_mapped[0];
  mlog_dshot_data.dshot_mapped[1] = data->dshot_mapped[1];
  mlog_dshot_data.dshot_mapped[2] = data->dshot_mapped[2];
  mlog_dshot_data.dshot_mapped[3] = data->dshot_mapped[3];

#ifdef L1_MIDDLEWARE_01_MODULE_05_FILTER_RPM_EN
  mlog_dshot_data.erpm[0] = data->erpm[0];
  mlog_dshot_data.erpm[1] = data->erpm[1];
  mlog_dshot_data.erpm[2] = data->erpm[2];
  mlog_dshot_data.erpm[3] = data->erpm[3];
#endif

  /* Push to mlog */
  mlog_push_msg((uint8_t*)&mlog_dshot_data, DShot_Motor_ID, sizeof(mlogDshotData_t));
}

#else /* TASK_TOOL_02_SD_MLOG not defined */

/* Empty implementations when mlog is not enabled */
void mlogDshotInit(void) {
    /* Do nothing when mlog is disabled */
}

void mlogDshotPush(const mlogDshotData_t* data) {
  RT_UNUSED(data);
  /* Do nothing when mlog is disabled */
}

#endif /* TASK_TOOL_02_SD_MLOG */
