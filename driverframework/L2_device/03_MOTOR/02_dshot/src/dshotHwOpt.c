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
#include "timestamp.h"

#ifdef DSHOT_DEBUGPIN_EN
#include "debugPin.h"
#endif

#ifdef DSHOT_PLATFORM_STM32
/* STM32 DMA interrupt flag - defined in dmaConfig.h */
#ifndef DSHOT_DMA_INT_FLAG
#error "DSHOT_DMA_INT_FLAG must be defined in dmaConfig.h"
#endif

/* Direct register access macros for PD12 - fast GPIO toggle in interrupt */
/* PD12 corresponds to GPIOD pin 12 */
/* 
 * 寄存器说明：
 * 
 * ODR (Output Data Register) - 输出数据寄存器
 *   - 作用：存储GPIO引脚的输出电平值
 *   - 读取：返回当前引脚的输出状态（0=低电平，1=高电平）
 *   - 写入：直接设置引脚的输出电平
 *   - 特点：读-修改-写操作不是原子的，在中断中可能不安全
 * 
 * BSRR (Bit Set/Reset Register) - 位设置/复位寄存器
 *   - 作用：原子性地设置或清除GPIO引脚电平
 *   - BSRR[15:0]：BS (Bit Set) - 设置位，写1设置对应引脚为高电平，写0无效
 *   - BSRR[31:16]：BR (Bit Reset) - 复位位，写1清除对应引脚为低电平，写0无效
 *   - 特点：原子操作，无需读-修改-写，适合中断中使用
 * 
 * GPIO_PIN_12 = 0x1000 (bit 12)
 * 
 * 性能优化：使用静态bool变量跟踪状态，避免读取ODR寄存器
 *   - 优势：内存读取比硬件寄存器读取更快
 *   - 优势：减少总线访问，提高翻转速度
 *   - 注意：需要确保SET_HIGH/SET_LOW和TOGGLE使用同一个状态变量
 */
#ifdef DSHOT_DEBUGPIN_EN
/* Static variable to track PD12 state - faster than reading ODR register */
static rt_bool_t pd12_state = RT_FALSE;  /* Track PD12 state: RT_FALSE=low, RT_TRUE=high */
#endif

#define PD12_SET_HIGH()    do { \
                              GPIOD->BSRR = GPIO_PIN_12; \
                              pd12_state = RT_TRUE; \
                          } while(0)  /* Set PD12 high: BSRR = 0x00001000, state = true */

#define PD12_SET_LOW()     do { \
                              GPIOD->BSRR = (GPIO_PIN_12 << 16); \
                              pd12_state = RT_FALSE; \
                          } while(0)  /* Set PD12 low: BSRR = 0x10000000, state = false */

/* Toggle PD12 using BSRR register - use bool variable instead of reading ODR for faster operation */
#define PD12_TOGGLE()      do { \
                              if (pd12_state) { \
                                  GPIOD->BSRR = (GPIO_PIN_12 << 16); \
                                  pd12_state = RT_FALSE; \
                              } else { \
                                  GPIOD->BSRR = GPIO_PIN_12; \
                                  pd12_state = RT_TRUE; \
                              } \
                          } while(0)  /* Fast toggle: check bool, write BSRR, update bool */

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
#ifdef DSHOT_DEBUGPIN_EN
  PD12_SET_HIGH();  /* Mark: Start loading DMA - direct register access */
#endif
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)dmaResource;
  dma_stream->CR &= ~DMA_SxCR_EN;  /* Disable DMA */
  /* Wait for disable with timeout to avoid infinite loop */
  rt_uint32_t timeout = 1000;
  while ((dma_stream->CR & DMA_SxCR_EN) && (timeout-- > 0));
  if (timeout == 0) {
    rt_kprintf("[DSHOT] Warning: bbLoadDMARegs DMA disable timeout\n");
  }
  dma_stream->CR = dmaRegCache.CCR;
  dma_stream->NDTR = dmaRegCache.CNDTR;
  dma_stream->PAR = dmaRegCache.CPAR;
  dma_stream->M0AR = dmaRegCache.CMAR;
  dma_stream->CR |= DMA_SxCR_EN;  /* Enable DMA */
#ifdef DSHOT_DEBUGPIN_EN
  PD12_SET_LOW();  /* Mark: DMA loaded - direct register access */
#endif
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
  /* 
   * 重要：这里会重新设置ARR寄存器，覆盖初始化时的Period值
   * 如果需要修改定时器频率，应该修改 dshot_config_.timer_count_send
   * 而不是修改 htim->Init.Period
   */
  __HAL_TIM_SET_AUTORELOAD(htim, dshot_config_.timer_count_send);
  __HAL_TIM_SET_COUNTER(htim, 0);
  
#ifdef DSHOT_DEBUGPIN_EN
  /* Debug: Print actual ARR value when setting timer (only first few times) */
  static rt_uint32_t debug_set_timer_count = 0;
  if (debug_set_timer_count < 3) {
    rt_kprintf("[DSHOT] setTimerForDshotOutput: timer_count_send=%u, actual ARR=%u\n",
               dshot_config_.timer_count_send, htim->Instance->ARR);
    debug_set_timer_count++;
  }
#endif
#elif defined(DSHOT_PLATFORM_AT32)
  tmr_base_init((tmr_type *)dshot_config_.timer_x, dshot_config_.timer_count_send, 0);
#endif
}

/* Timer interrupt handler for high-precision timing measurement */
#ifdef DSHOT_DEBUGPIN_EN
#ifdef DSHOT_PLATFORM_STM32
void DSHOT_TIMER_IRQHandler(void) {
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  
  /* High-precision timing measurement using DWT cycle counter */
  static rt_uint32_t last_cycle = 0;
  static rt_uint32_t cycle_sum = 0;
  static rt_uint32_t cycle_min = 0xFFFFFFFF;
  static rt_uint32_t cycle_max = 0;
  static rt_uint32_t sample_count = 0;
  
  rt_uint32_t current_cycle = timestamp_cycles();
  
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)) {
    /* Calculate cycle delta (interrupt interval) */
    if (last_cycle != 0) {
      rt_uint32_t cycle_delta = current_cycle - last_cycle;
      
      /* Accumulate statistics */
      cycle_sum += cycle_delta;
      sample_count++;
      
      if (cycle_delta < cycle_min) {
        cycle_min = cycle_delta;
      }
      if (cycle_delta > cycle_max) {
        cycle_max = cycle_delta;
      }
    }
    last_cycle = current_cycle;
    
    /* Print statistics periodically (every 1000 interrupts) */
    static rt_uint32_t debug_count = 0;
    if (sample_count > 0 && (sample_count % 1000 == 0 || debug_count < 5)) {
      rt_uint32_t sys_freq = timestamp_get_sys_freq();
      rt_uint32_t avg_cycles = cycle_sum / sample_count;
      
      /* Calculate period in nanoseconds: period_ns = cycles / freq_hz * 1e9 */
      float avg_period_ns = (float)avg_cycles * 1000000000.0f / (float)sys_freq;
      float avg_freq_hz = (float)sys_freq / (float)avg_cycles;
      float min_period_ns = (float)cycle_min * 1000000000.0f / (float)sys_freq;
      float max_period_ns = (float)cycle_max * 1000000000.0f / (float)sys_freq;
      float min_freq_hz = (float)sys_freq / (float)cycle_max;
      float max_freq_hz = (float)sys_freq / (float)cycle_min;
      
      rt_kprintf("[DSHOT_TIMING] Samples=%lu, sys_freq=%luHz\n", sample_count, sys_freq);
      rt_kprintf("  Frequency: avg=%.3fHz, min=%.3fHz, max=%.3fHz\n",
                 avg_freq_hz, min_freq_hz, max_freq_hz);
      rt_kprintf("  Period: avg=%.3fns, min=%.3fns, max=%.3fns\n",
                 avg_period_ns, min_period_ns, max_period_ns);
      rt_kprintf("  Cycles: avg=%lu, min=%lu, max=%lu\n",
                 avg_cycles, cycle_min, cycle_max);
      
      debug_count++;
      
      /* Reset statistics every 10000 samples to avoid overflow */
      if (sample_count >= 10000) {
        cycle_sum = 0;
        sample_count = 0;
        cycle_min = 0xFFFFFFFF;
        cycle_max = 0;
      }
    }
    
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
  }
}
#elif defined(DSHOT_PLATFORM_AT32)
void DSHOT_TIMER_IRQHandler(void) {
  tmr_type *tmr_x = (tmr_type *)dshot_config_.timer_x;
  if (tmr_interrupt_flag_get(tmr_x, TMR_OVF_FLAG)) {
#ifdef DSHOT_DEBUGPIN_EN
    DEBUG_PIN_DEBUG2_TOGGLE();  /* Toggle on timer overflow - indicates timer is running */
#endif
    tmr_flag_clear(tmr_x, TMR_OVF_FLAG);
  }
}
#endif
#endif

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

/* DMA transfer complete callback function */
#ifdef DSHOT_PLATFORM_STM32
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

/* Clear DMA interrupt flag - 仅用于 AT32 */
#ifdef DSHOT_PLATFORM_AT32
static void dma_isr_clear_flag(struct dma_config *dma_instance) {
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
}
#endif

/* STM32H7 DMA interrupt handler - 直接调用 HAL 库函数 */
/* 注意：对于 STM32H7，只需要调用 HAL_DMA_IRQHandler，业务逻辑在回调函数中处理 */
#ifdef DSHOT_PLATFORM_STM32
void DSHOT_DMA_IRQHandler(void) {
#ifdef DSHOT_DEBUGPIN_EN
  PD12_SET_HIGH();  /* Mark: DMA IRQ entered - direct register access */
#endif
  rt_interrupt_enter();
  /* 调用 HAL 库的中断处理函数，它会自动调用回调函数 dshot_dma_xfer_cplt_callback */
  /* 使用 dshot_config_ 中的 dma_handle 成员 */
  if (dshot_config_.dma_handle.Instance != NULL) {
    HAL_DMA_IRQHandler(&dshot_config_.dma_handle);
  }
  rt_interrupt_leave();
#ifdef DSHOT_DEBUGPIN_EN
  PD12_SET_LOW();  /* Mark: DMA IRQ exited - direct register access */
#endif
}
#endif

/* AT32 DMA interrupt handler - 需要手动处理中断标志和业务逻辑 */
#ifdef DSHOT_PLATFORM_AT32
void DSHOT_DMA_IRQHandler(void) {
#ifdef DSHOT_DEBUGPIN_EN
  DEBUG_PIN_DEBUG1_HIGH();  /* Mark: DMA IRQ entered */
#endif
  rt_interrupt_enter();
  /* 安全检查 */
  if (dshot_config_.dma_cfg != NULL) {
    dma_isr_clear_flag(dshot_config_.dma_cfg);
  }
  rt_interrupt_leave();
#ifdef DSHOT_DEBUGPIN_EN
  DEBUG_PIN_DEBUG1_LOW();  /* Mark: DMA IRQ exited */
#endif
}
#endif

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

/* Timer interrupt service function bindings - automatically generated based on DSHOT_TIMER_SELECT */
/* These functions override the weak symbols in startup files and call DSHOT_TIMER_IRQHandler */
#ifdef DSHOT_DEBUGPIN_EN
#ifdef DSHOT_PLATFORM_STM32
/* STM32 Timer interrupt handlers */
#if DSHOT_TIMER_SELECT == 1
void TIM1_UP_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 2
void TIM2_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 3
void TIM3_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 4
void TIM4_IRQHandler(void) {
  // rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  // rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 5
void TIM5_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 8
void TIM8_UP_TIM13_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#ifdef TIM9
#if DSHOT_TIMER_SELECT == 9
void TIM1_BRK_TIM9_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif
#endif

#ifdef TIM10
#if DSHOT_TIMER_SELECT == 10
void TIM1_UP_TIM10_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif
#endif

#ifdef TIM11
#if DSHOT_TIMER_SELECT == 11
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif
#endif

#elif defined(DSHOT_PLATFORM_AT32)
/* AT32 Timer interrupt handlers */
#if DSHOT_TIMER_SELECT == 1
void TMR1_OVF_TMR10_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 2
void TMR2_GLOBAL_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 3
void TMR3_GLOBAL_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 4
void TMR4_GLOBAL_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 5
void TMR5_GLOBAL_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 8
void TMR8_OVF_TMR13_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 9
void TMR1_BRK_TMR9_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 10
void TMR1_OVF_TMR10_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif

#if DSHOT_TIMER_SELECT == 11
void TMR1_TRG_HALL_TMR11_IRQHandler(void) {
  rt_interrupt_enter();
  DSHOT_TIMER_IRQHandler();
  rt_interrupt_leave();
}
#endif
#endif
#endif