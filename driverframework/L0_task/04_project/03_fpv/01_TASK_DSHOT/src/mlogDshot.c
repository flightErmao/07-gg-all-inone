#include "mlogDshot.h"

#ifdef TASK_TOOL_02_SD_MLOG
#include "mlog.h"
#endif

/* Mlog DShot data structure - simplified */
typedef struct {
    uint32_t timestamp;
    uint16_t dshot_mapped[4];  // mapped DShot values (48~2048)
} mlogDshotData_t;

static void mlogDshotStartCb(void);

#ifdef TASK_TOOL_02_SD_MLOG
/* Mlog bus definition for DShot data - simplified */
static mlog_elem_t Minifly_DShot_Motor_Elems[] __attribute__((used)) = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT_VEC(dshot_mapped, MLOG_UINT16, 4),
};
MLOG_BUS_DEFINE(Minifly_DShot_Motor, Minifly_DShot_Motor_Elems);

/* Static variables */
static mlogDshotData_t mlog_dshot_data = {0};
static int Minifly_DShot_Motor_ID = -1;
static uint8_t mlog_push_en = 0;

/**
 * @brief Initialize mlog DShot functionality
 */
void mlogDshotInit(void) {
    /* Initialize mlog bus ID for DShot data */
    Minifly_DShot_Motor_ID = mlog_get_bus_id("Minifly_DShot_Motor");
    if (Minifly_DShot_Motor_ID < 0) {
        rt_kprintf("Failed to get mlog bus ID for Minifly_DShot_Motor\n");
    } else {
        rt_kprintf("Minifly_DShot_Motor mlog bus ID: %d\n", Minifly_DShot_Motor_ID);
    }
    
    /* Register mlog start callback */
    mlog_register_callback(MLOG_CB_START, mlogDshotStartCb);
}

/**
 * @brief Mlog start callback - internal function
 */
static void mlogDshotStartCb(void) {
    mlog_push_en = 1;
}

/**
 * @brief Push DShot data to mlog
 * @param dshot_mapped mapped DShot values (4 channels)
 * @param timestamp timestamp
 */
void mlogDshotPush(const uint16_t* dshot_mapped, uint32_t timestamp) {
    if (Minifly_DShot_Motor_ID < 0 || !mlog_push_en) {
        return;
    }
    
    /* Copy mapped DShot values */
    if (dshot_mapped != RT_NULL) {
        mlog_dshot_data.dshot_mapped[0] = dshot_mapped[0];
        mlog_dshot_data.dshot_mapped[1] = dshot_mapped[1];
        mlog_dshot_data.dshot_mapped[2] = dshot_mapped[2];
        mlog_dshot_data.dshot_mapped[3] = dshot_mapped[3];
    }
    
    /* Set timestamp */
    mlog_dshot_data.timestamp = timestamp;
    
    /* Push to mlog */
    mlog_push_msg((uint8_t*)&mlog_dshot_data, Minifly_DShot_Motor_ID, sizeof(mlogDshotData_t));
}

#else /* TASK_TOOL_02_SD_MLOG not defined */

/* Empty implementations when mlog is not enabled */
void mlogDshotInit(void) {
    /* Do nothing when mlog is disabled */
}

void mlogDshotPush(const uint16_t* dshot_mapped, uint32_t timestamp) {
    RT_UNUSED(dshot_mapped);
    RT_UNUSED(timestamp);
    /* Do nothing when mlog is disabled */
}

#endif /* TASK_TOOL_02_SD_MLOG */
