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

#ifdef DSHOT_PLATFORM_STM32
/* STM32 DMA interrupt flag - defined in dmaConfig.h */
#ifndef DSHOT_DMA_INT_FLAG
#error "DSHOT_DMA_INT_FLAG must be defined in dmaConfig.h"
#endif
#elif defined(DSHOT_PLATFORM_AT32)
#define DSHOT_DMA_INT_FLAG DMA2_FDT7_FLAG
#endif

typedef enum { DSHOT_SENDED = 0, DSHOT_REC } DSHOT_DIR_e;

/* External variables from main driver */
extern dshot_config_t dshot_config_;

/* Set GPIO pins to output mode */
static void setGpioPinsToOutputMode(void) {
#ifdef DSHOT_PLATFORM_STM32
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  for (rt_uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    GPIO_InitStruct.Pin = (1 << dshot_config_.pin_index_arr[i]);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init((GPIO_TypeDef*)dshot_config_.gpio, &GPIO_InitStruct);
  }
#elif defined(DSHOT_PLATFORM_AT32)
  for (rt_uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    dshot_config_.gpio->cfgr &= (uint32_t)(~(0x03 << (dshot_config_.pin_index_arr[i] * 2)));
    dshot_config_.gpio->cfgr |= (uint32_t)(1 << (dshot_config_.pin_index_arr[i] * 2));
  }
#endif
  dshot_config_.dshot_dma_dir = DSHOT_SENDED;
}

/* Set GPIO pins to input mode */
static void setGpioPinsToInputMode(void) {
  dshot_config_.dshot_dma_dir = DSHOT_REC;
#ifdef DSHOT_PLATFORM_STM32
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  for (rt_uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    GPIO_InitStruct.Pin = (1 << dshot_config_.pin_index_arr[i]);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init((GPIO_TypeDef*)dshot_config_.gpio, &GPIO_InitStruct);
  }
#elif defined(DSHOT_PLATFORM_AT32)
  for (rt_uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    dshot_config_.gpio->cfgr &= (uint32_t)(~(0x03 << (dshot_config_.pin_index_arr[i] * 2)));
  }
#endif
}

/* Save DMA registers */
void bbSaveDMARegs(void *dmaResource, dmaRegCache_t *dmaRegCache) {
#ifdef DSHOT_PLATFORM_STM32
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)dmaResource;
  dmaRegCache->CCR = dma_stream->CR;
  dmaRegCache->CNDTR = dma_stream->NDTR;
  dmaRegCache->CPAR = dma_stream->PAR;
  dmaRegCache->CMAR = dma_stream->M0AR;
#elif defined(DSHOT_PLATFORM_AT32)
  dmaRegCache->CCR = ((dma_channel_type *)dmaResource)->ctrl;
  dmaRegCache->CNDTR = ((dma_channel_type *)dmaResource)->dtcnt;
  dmaRegCache->CPAR = ((dma_channel_type *)dmaResource)->paddr;
  dmaRegCache->CMAR = ((dma_channel_type *)dmaResource)->maddr;
#endif
}

/* Load DMA registers */
static void bbLoadDMARegs(void *dmaResource, dmaRegCache_t dmaRegCache) {
#ifdef DSHOT_PLATFORM_STM32
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)dmaResource;
  dma_stream->CR &= ~DMA_SxCR_EN;  /* Disable DMA */
  while (dma_stream->CR & DMA_SxCR_EN); /* Wait for disable */
  dma_stream->CR = dmaRegCache.CCR;
  dma_stream->NDTR = dmaRegCache.CNDTR;
  dma_stream->PAR = dmaRegCache.CPAR;
  dma_stream->M0AR = dmaRegCache.CMAR;
  dma_stream->CR |= DMA_SxCR_EN;  /* Enable DMA */
#elif defined(DSHOT_PLATFORM_AT32)
  dma_channel_enable(dshot_config_.dma_cfg->dma_channel, FALSE);
  ((dma_channel_type *)dmaResource)->ctrl = dmaRegCache.CCR;
  ((dma_channel_type *)dmaResource)->dtcnt = dmaRegCache.CNDTR;
  ((dma_channel_type *)dmaResource)->paddr = dmaRegCache.CPAR;
  ((dma_channel_type *)dmaResource)->maddr = dmaRegCache.CMAR;
  dma_channel_enable(dshot_config_.dma_cfg->dma_channel, TRUE);
#endif
}

/* Set timer for DShot output */
static void setTimerForDshotOutput(void) {
#ifdef DSHOT_PLATFORM_STM32
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  __HAL_TIM_SET_AUTORELOAD(htim, dshot_config_.timer_count_send);
  __HAL_TIM_SET_COUNTER(htim, 0);
#elif defined(DSHOT_PLATFORM_AT32)
  tmr_base_init((tmr_type *)dshot_config_.timer_x, dshot_config_.timer_count_send, 0);
#endif
}

/* Set timer for receive */
static void timerSetForRec(void) {
#ifdef DSHOT_PLATFORM_STM32
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  __HAL_TIM_SET_AUTORELOAD(htim, dshot_config_.timer_count_rec);
  __HAL_TIM_SET_COUNTER(htim, 0);
#elif defined(DSHOT_PLATFORM_AT32)
  tmr_base_init((tmr_type *)dshot_config_.timer_x, dshot_config_.timer_count_rec, 0);
#endif
}

/* Global DMA handle for STM32 interrupt handling */
#ifdef DSHOT_PLATFORM_STM32
extern DMA_HandleTypeDef dshot_dma_handle_static;

/* DMA transfer complete callback function */
void dshot_dma_xfer_cplt_callback(DMA_HandleTypeDef *hdma) {
  /* 安全检查：确保所有必要的指针都有效 */
  if (hdma == NULL || 
      dshot_config_.dma_cfg == NULL || 
      dshot_config_.timer_x == NULL ||
      dshot_config_.event_dma == NULL) {
    return;  /* Invalid pointers */
  }
  
  /* 检查是否是我们的 DMA handle */
  if (hdma->Instance != dshot_config_.dma_cfg->Instance) {
    return;  /* Not our DMA stream */
  }
  
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  if (htim == NULL || htim->Instance == NULL) {
    return;  /* Invalid timer handle */
  }
  
  __HAL_TIM_DISABLE_DMA(htim, dshot_config_.tmr_dma_request);
  
  if (dshot_config_.dshot_dma_dir == DSHOT_SENDED) {
    setGpioPinsToInputMode();
    timerSetForRec();
    if (dshot_config_.dma_cfg->Instance != NULL) {
      bbLoadDMARegs((void *)dshot_config_.dma_cfg->Instance, dshot_config_.dmaRegInput);
    }
    __HAL_TIM_ENABLE_DMA(htim, dshot_config_.tmr_dma_request);
  } else if (dshot_config_.dshot_dma_dir == DSHOT_REC) {
    if (dshot_config_.event_dma != NULL) {
      rt_event_send(dshot_config_.event_dma, EVENT_DMA_SAMPLING_DONE);
    }
  }
}
#endif

/* Clear DMA interrupt flag */
static void dma_isr_clear_flag(struct dma_config *dma_instance) {
#ifdef DSHOT_PLATFORM_STM32
  /* 使用 HAL 标准中断处理 */
  if (dshot_dma_handle_static.Instance == NULL) {
    return;  /* DMA not initialized yet */
  }
  
  /* 调用 HAL DMA 中断处理函数，它会自动调用回调函数 */
  HAL_DMA_IRQHandler(&dshot_dma_handle_static);
#elif defined(DSHOT_PLATFORM_AT32)
  if (dma_flag_get(DSHOT_DMA_INT_FLAG)) {
    dma_flag_clear(DSHOT_DMA_INT_FLAG);
    
    tmr_dma_request_enable((tmr_type *)dshot_config_.timer_x, dshot_config_.tmr_dma_request, FALSE);
    
    if (dshot_config_.dshot_dma_dir == DSHOT_SENDED) {
      setGpioPinsToInputMode();
      timerSetForRec();
      bbLoadDMARegs(dshot_config_.dma_cfg->dma_channel, dshot_config_.dmaRegInput);
      tmr_dma_request_enable((tmr_type *)dshot_config_.timer_x, dshot_config_.tmr_dma_request, TRUE);
    } else if (dshot_config_.dshot_dma_dir == DSHOT_REC) {
      rt_event_send(dshot_config_.event_dma, EVENT_DMA_SAMPLING_DONE);
    }
  }
#endif
}

/* DShot DMA interrupt handler */
void DSHOT_DMA_IRQHandler(void) {
  rt_interrupt_enter();
  /* 安全检查 */
  if (dshot_config_.dma_cfg != NULL) {
    dma_isr_clear_flag(dshot_config_.dma_cfg);
  }
  rt_interrupt_leave();
}

/* Set DShot value and start transmission */
rt_err_t setDshotValue(void) {
  encodeDshot();
  setGpioPinsToOutputMode();
  setTimerForDshotOutput();
#ifdef DSHOT_PLATFORM_STM32
  bbLoadDMARegs((void *)dshot_config_.dma_cfg->Instance, dshot_config_.dmaRegOutput);
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  __HAL_TIM_ENABLE_DMA(htim, dshot_config_.tmr_dma_request);
#elif defined(DSHOT_PLATFORM_AT32)
  bbLoadDMARegs(dshot_config_.dma_cfg->dma_channel, dshot_config_.dmaRegOutput);
  tmr_dma_request_enable((tmr_type *)dshot_config_.timer_x, dshot_config_.tmr_dma_request, TRUE);
#endif
  // now dshot is sending
  return RT_EOK;
}