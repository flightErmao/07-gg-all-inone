/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */

#include <rtthread.h>
#include "dshotHwOpt.h"
#include "dshotConfig.h"
#include "dmaConfig.h"
#include "dshotEncodeDecode.h"

#define DSHOT_DMA_INT_FLAG DMA2_FDT7_FLAG

typedef enum { DSHOT_SENDED = 0, DSHOT_REC } DSHOT_DIR_e;

/* External variables from main driver */
extern dshot_config_t dshot_config_;

/* Set GPIO pins to output mode */
static void setGpioPinsToOutputMode(void) {
  for (rt_uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    dshot_config_.gpio->cfgr &= (uint32_t)(~(0x03 << (dshot_config_.pin_index_arr[i] * 2)));
    dshot_config_.gpio->cfgr |= (uint32_t)(1 << (dshot_config_.pin_index_arr[i] * 2));
  }
  dshot_config_.dshot_dma_dir = DSHOT_SENDED;
}

/* Set GPIO pins to input mode */
static void setGpioPinsToInputMode(void) {
  dshot_config_.dshot_dma_dir = DSHOT_REC;
  for (rt_uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    dshot_config_.gpio->cfgr &= (uint32_t)(~(0x03 << (dshot_config_.pin_index_arr[i] * 2)));
  }
}

/* Save DMA registers */
void bbSaveDMARegs(dma_channel_type *dmaResource, dmaRegCache_t *dmaRegCache) {
  dmaRegCache->CCR = ((dma_channel_type *)dmaResource)->ctrl;
  dmaRegCache->CNDTR = ((dma_channel_type *)dmaResource)->dtcnt;
  dmaRegCache->CPAR = ((dma_channel_type *)dmaResource)->paddr;
  dmaRegCache->CMAR = ((dma_channel_type *)dmaResource)->maddr;
}

/* Load DMA registers */
static void bbLoadDMARegs(dma_channel_type *dmaResource, dmaRegCache_t dmaRegCache) {
  dma_channel_enable(dshot_config_.dma_cfg->dma_channel, FALSE);
  ((dma_channel_type *)dmaResource)->ctrl = dmaRegCache.CCR;     // ctrl info
  ((dma_channel_type *)dmaResource)->dtcnt = dmaRegCache.CNDTR;  // dtcnt data count
  ((dma_channel_type *)dmaResource)->paddr = dmaRegCache.CPAR;   // pheriph address
  ((dma_channel_type *)dmaResource)->maddr = dmaRegCache.CMAR;   // Memory address
  dma_channel_enable(dshot_config_.dma_cfg->dma_channel, TRUE);
}

/* Set timer for DShot output */
static void setTimerForDshotOutput(void) { tmr_base_init(dshot_config_.timer_x, dshot_config_.timer_count_send, 0); }

/* Set timer for receive */
static void timerSetForRec(void) { tmr_base_init(dshot_config_.timer_x, dshot_config_.timer_count_rec, 0); }

/* Clear DMA interrupt flag */
static void dma_isr_clear_flag(struct dma_config *dma_instance) {
  if (dma_flag_get(DSHOT_DMA_INT_FLAG)) {
    dma_flag_clear(DSHOT_DMA_INT_FLAG);

    tmr_dma_request_enable(dshot_config_.timer_x, dshot_config_.tmr_dma_request, FALSE);

    if (dshot_config_.dshot_dma_dir == DSHOT_SENDED) {
      setGpioPinsToInputMode();
      timerSetForRec();
      bbLoadDMARegs(dshot_config_.dma_cfg->dma_channel, dshot_config_.dmaRegInput);
      tmr_dma_request_enable(dshot_config_.timer_x, dshot_config_.tmr_dma_request, TRUE);
    } else if (dshot_config_.dshot_dma_dir == DSHOT_REC) {
      rt_event_send(dshot_config_.event_dma, EVENT_DMA_SAMPLING_DONE);
    }
  }
}

/* DShot DMA interrupt handler */
void DSHOT_DMA_IRQHandler(void) {
  rt_interrupt_enter();
  dma_isr_clear_flag(dshot_config_.dma_cfg);
  rt_interrupt_leave();
}

/* Set DShot value and start transmission */
rt_err_t setDshotValue(void) {
  encodeDshot();
  setGpioPinsToOutputMode();
  setTimerForDshotOutput();
  bbLoadDMARegs(dshot_config_.dma_cfg->dma_channel, dshot_config_.dmaRegOutput);
  tmr_dma_request_enable(dshot_config_.timer_x, dshot_config_.tmr_dma_request, TRUE);
  // now dshot is sending
  return RT_EOK;
}