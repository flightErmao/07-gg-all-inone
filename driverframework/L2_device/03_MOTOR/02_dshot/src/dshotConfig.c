/*
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */

#include <rtconfig.h>
#include <rtthread.h>
#include <string.h>
#include "dmaConfig.h"
#include "dshotHwOpt.h"
#include "dshotConfig.h"
#include "stdbool.h"

#ifdef DSHOT_PLATFORM_STM32
/* External functions from drv_tim.c */
extern void stm32_tim_enable_clock(TIM_HandleTypeDef* htim_base);
extern void stm32_tim_pclkx_doubler_get(rt_uint32_t *pclk1_doubler, rt_uint32_t *pclk2_doubler);

/* Global DMA handle for STM32 interrupt handling */
DMA_HandleTypeDef dshot_dma_handle_static = {0};
#endif

/*about DMA config parameter*/
#ifdef DSHOT_PLATFORM_STM32
#define DSHOT_DAM_CONFIG                       \
  {                                            \
      .Instance = DSHOT_DMA_STREAM,            \
      .dma_rcc = DSHOT_DMA_CLOCK,              \
      .dma_irq = DSHOT_DMA_IRQ,                \
      .request = DSHOT_DMA_REQUEST,            \
  }
#elif defined(DSHOT_PLATFORM_AT32)
#define DSHOT_DAM_CONFIG                       \
  {                                            \
      .dma_channel = DSHOT_DMA_CHANNEL,        \
      .dma_clock = DSHOT_DMA_CLOCK,            \
      .dma_irqn = DSHOT_DMA_IRQ,               \
      .dmamux_channel = DSHOT_DMA_MUX_CHANNEL, \
      .request_id = DSHOT_DMA_REQ_ID,          \
  }
#else
#error "Unsupported platform for DShot DMA configuration"
#endif

/* DShot symbol rates */
#define MOTOR_DSHOT600_SYMBOL_RATE (600 * 1000)
#define MOTOR_DSHOT300_SYMBOL_RATE (300 * 1000)
#define MOTOR_DSHOT150_SYMBOL_RATE (150 * 1000)

/* Global runtime configuration */
dshot_config_t dshot_config_ = {0};

/* Convert GPIO port string to GPIO type */
static void *convert_gpio_port(void) {
  const char *port_str = DSHOT_GPIO_PORT;
  // Check for "GPIO" prefix and get the port letter (4th character)
  if (strlen(port_str) >= 4 && strncmp(port_str, "GPIO", 4) == 0) {
    char port_letter = port_str[4];  // Get the letter after "GPIO"
    if (port_letter == 'A') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOA;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOA;
#endif
    } else if (port_letter == 'B') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOB;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOB;
#endif
    } else if (port_letter == 'C') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOC;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOC;
#endif
    }
  }
  // Fallback: check first character for backward compatibility
  else if (port_str[0] == 'A') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOA;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOA;
#endif
  } else if (port_str[0] == 'B') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOB;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOB;
#endif
  } else if (port_str[0] == 'C') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOC;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOC;
#endif
  }

#ifdef DSHOT_PLATFORM_STM32
  return (void *)GPIOB;  // default fallback
#elif defined(DSHOT_PLATFORM_AT32)
  return (void *)GPIOB;  // default fallback
#endif
}

/* Convert GPIO port string to GPIO clock */
static void *convert_gpio_clock(void) {
#ifdef DSHOT_PLATFORM_STM32
  /* STM32 uses RCC macros, return NULL as placeholder */
  return NULL;
#elif defined(DSHOT_PLATFORM_AT32)
  const char *port_str = DSHOT_GPIO_PORT;
  // Check for "GPIO" prefix and get the port letter (4th character)
  if (strlen(port_str) >= 4 && strncmp(port_str, "GPIO", 4) == 0) {
    char port_letter = port_str[4];  // Get the letter after "GPIO"
    if (port_letter == 'A') {
      return (void *)CRM_GPIOA_PERIPH_CLOCK;
    } else if (port_letter == 'B') {
      return (void *)CRM_GPIOB_PERIPH_CLOCK;
    } else if (port_letter == 'C') {
      return (void *)CRM_GPIOC_PERIPH_CLOCK;
    }
  }
  // Fallback: check first character for backward compatibility
  else if (port_str[0] == 'A') {
    return (void *)CRM_GPIOA_PERIPH_CLOCK;
  } else if (port_str[0] == 'B') {
    return (void *)CRM_GPIOB_PERIPH_CLOCK;
  } else if (port_str[0] == 'C') {
    return (void *)CRM_GPIOC_PERIPH_CLOCK;
  }

  return (void *)CRM_GPIOB_PERIPH_CLOCK;  // default fallback
#endif
}

/* Convert timer selection to timer type */
static void *convert_timer_type(void) {
#ifdef DSHOT_PLATFORM_STM32
  /* For STM32, we need to return TIM_HandleTypeDef pointer */
  /* This will be initialized in dshotTimerInit */
  static TIM_HandleTypeDef htim_dshot;
  return (void *)&htim_dshot;
#elif defined(DSHOT_PLATFORM_AT32)
  switch (DSHOT_TIMER_SELECT) {
    case 1:
      return (void *)TMR1;
    case 2:
      return (void *)TMR2;
    case 3:
      return (void *)TMR3;
    case 4:
      return (void *)TMR4;
    case 5:
      return (void *)TMR5;
    case 6:
      return (void *)TMR6;
    case 7:
      return (void *)TMR7;
    case 8:
      return (void *)TMR8;
    case 9:
      return (void *)TMR9;
    case 10:
      return (void *)TMR10;
    case 11:
      return (void *)TMR11;
    default:
      return (void *)TMR4;  // default fallback
  }
#endif
}

/* Convert timer selection to timer clock */
static void *convert_timer_clock(void) {
#ifdef DSHOT_PLATFORM_STM32
  /* STM32 uses RCC macros, return NULL as placeholder */
  return NULL;
#elif defined(DSHOT_PLATFORM_AT32)
  switch (DSHOT_TIMER_SELECT) {
    case 1:
      return (void *)CRM_TMR1_PERIPH_CLOCK;
    case 2:
      return (void *)CRM_TMR2_PERIPH_CLOCK;
    case 3:
      return (void *)CRM_TMR3_PERIPH_CLOCK;
    case 4:
      return (void *)CRM_TMR4_PERIPH_CLOCK;
    case 5:
      return (void *)CRM_TMR5_PERIPH_CLOCK;
    case 6:
      return (void *)CRM_TMR6_PERIPH_CLOCK;
    case 7:
      return (void *)CRM_TMR7_PERIPH_CLOCK;
    case 8:
      return (void *)CRM_TMR8_PERIPH_CLOCK;
    case 9:
      return (void *)CRM_TMR9_PERIPH_CLOCK;
    case 10:
      return (void *)CRM_TMR10_PERIPH_CLOCK;
    case 11:
      return (void *)CRM_TMR11_PERIPH_CLOCK;
    default:
      return (void *)CRM_TMR4_PERIPH_CLOCK;  // default fallback
  }
#endif
}

/* Convert GPIO port to DMA peripheral register addresses */
static rt_uint32_t __attribute__((unused)) convert_dma_input_io_addr(void) {
  void *gpio = convert_gpio_port();
#ifdef DSHOT_PLATFORM_STM32
  if (gpio == (void *)GPIOA) {
    return (rt_uint32_t)&GPIOA->IDR;
  } else if (gpio == (void *)GPIOB) {
    return (rt_uint32_t)&GPIOB->IDR;
  } else if (gpio == (void *)GPIOC) {
    return (rt_uint32_t)&GPIOC->IDR;
  }
  return (rt_uint32_t)&GPIOB->IDR;
#elif defined(DSHOT_PLATFORM_AT32)
  if (gpio == (void *)GPIOA) {
    return (rt_uint32_t)&GPIOA->idt;
  } else if (gpio == (void *)GPIOB) {
    return (rt_uint32_t)&GPIOB->idt;
  } else if (gpio == (void *)GPIOC) {
    return (rt_uint32_t)&GPIOC->idt;
  }
  return (rt_uint32_t)&GPIOB->idt;
#endif
}

static rt_uint32_t __attribute__((unused)) convert_dma_output_io_addr(void) {
  void *gpio = convert_gpio_port();
#ifdef DSHOT_PLATFORM_STM32
  if (gpio == (void *)GPIOA) {
    return (rt_uint32_t)&GPIOA->ODR;
  } else if (gpio == (void *)GPIOB) {
    return (rt_uint32_t)&GPIOB->ODR;
  } else if (gpio == (void *)GPIOC) {
    return (rt_uint32_t)&GPIOC->ODR;
  }
  return (rt_uint32_t)&GPIOB->ODR;
#elif defined(DSHOT_PLATFORM_AT32)
  if (gpio == (void *)GPIOA) {
    return (rt_uint32_t)&GPIOA->odt;
  } else if (gpio == (void *)GPIOB) {
    return (rt_uint32_t)&GPIOB->odt;
  } else if (gpio == (void *)GPIOC) {
    return (rt_uint32_t)&GPIOC->odt;
  }
  return (rt_uint32_t)&GPIOB->odt;
#endif
}

/* Convert motor pins to array */
static void convert_motor_pins(uint8_t *pin_array) {
  pin_array[0] = DSHOT_GPIO_PIN_MOTOR1;
  pin_array[1] = DSHOT_GPIO_PIN_MOTOR2;
  pin_array[2] = DSHOT_GPIO_PIN_MOTOR3;
  pin_array[3] = DSHOT_GPIO_PIN_MOTOR4;

  // Initialize remaining pins to 0 if motor count is less than 4
  for (int i = 4; i < DSHOT_MOTOR_NUMS; i++) {
    pin_array[i] = 0;
  }
}

/* Convert bi-directional config */
static rt_bool_t convert_bi_directional_config(void) {
#ifdef DSHOT_BI_DIRECTIONAL
  return RT_TRUE;
#else
  return RT_FALSE;
#endif
}

/* Convert DShot protocol type */
static motorPwmProtocolTypes_e convert_dshot_protocol_type(void) {
  const char *protocol_str = DSHOT_PROTOCOL_TYPE;

  if (strcmp(protocol_str, "dshot_150") == 0) {
    return PWM_TYPE_DSHOT150;
  } else if (strcmp(protocol_str, "dshot_300") == 0) {
    return PWM_TYPE_DSHOT300;
  } else if (strcmp(protocol_str, "dshot_600") == 0) {
    return PWM_TYPE_DSHOT600;
  } else {
    return PWM_TYPE_DSHOT600;  // default fallback
  }
}

/* Convert DMA configuration */
static struct dma_config *convert_dma_config(void) {
  static struct dma_config dshot_dma_temp = DSHOT_DAM_CONFIG;
#ifdef DSHOT_PLATFORM_STM32
  /* STM32 DMA config is already set by DSHOT_DAM_CONFIG macro */
  /* For STM32H7, Instance is DMA_Stream_TypeDef* */
  /* STM32 dma_config doesn't have dma_done member */
#elif defined(DSHOT_PLATFORM_AT32)
  /* search dma base and channel index */
  dma_channel_type *dshot_channel = (dma_channel_type *)dshot_dma_temp.dma_channel;
  dshot_dma_temp.dma_x = (dma_type *)((rt_uint32_t)dshot_channel & ~0xFF);
  dshot_dma_temp.channel_index = ((((rt_uint32_t)dshot_channel & 0xFF) - 8) / 0x14) + 1;
  dshot_dma_temp.dma_done = RT_TRUE;
#endif
  return &dshot_dma_temp;
}

static rt_event_t convert_event_dma(void) {
  static struct rt_event event_dshot_dma;
  rt_event_init(&event_dshot_dma, "event_dshot_rec", RT_IPC_FLAG_PRIO);
  return &event_dshot_dma;
}

/* Main configuration initialization function */
rt_err_t dshotConfigInit(void) {
  /* GPIO configuration from rtconfig.h */
  dshot_config_.gpio = convert_gpio_port();
  dshot_config_.gpio_clock = convert_gpio_clock();
  convert_motor_pins(dshot_config_.pin_index_arr);

  /* Timer configuration from rtconfig.h */
  dshot_config_.timer_x = convert_timer_type();
  dshot_config_.timer_clock = convert_timer_clock();
#ifdef DSHOT_PLATFORM_STM32
  dshot_config_.tmr_dma_request = TIM_DMA_UPDATE;  /* STM32 DMA request for update event */
#elif defined(DSHOT_PLATFORM_AT32)
  dshot_config_.tmr_dma_request = TMR_OVERFLOW_DMA_REQUEST;
#endif

  /* DMA configuration */
  dshot_config_.dma_cfg = convert_dma_config();

  /* Event initialization */
  dshot_config_.event_dma = convert_event_dma();

  /* Protocol configuration */
  dshot_config_.dshot_protocol = convert_dshot_protocol_type();
  dshot_config_.bi_dshot_en = convert_bi_directional_config();
  dshot_config_.act_cmd_en = RT_FALSE;

  dshot_config_.motor_dir_ctrl[0].motor_dir = DSHOT_CMD_SPIN_DIRECTION_1;  // motor 1
  dshot_config_.motor_dir_ctrl[1].motor_dir = DSHOT_CMD_SPIN_DIRECTION_1;  // motor 2
  dshot_config_.motor_dir_ctrl[2].motor_dir = DSHOT_CMD_SPIN_DIRECTION_1;  // motor 3
  dshot_config_.motor_dir_ctrl[3].motor_dir = DSHOT_CMD_SPIN_DIRECTION_1;  // motor 4

  return RT_EOK;
}

/* Initialize DShot GPIO */
void dshotGpioInit(void) {
#ifdef DSHOT_PLATFORM_STM32
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_TypeDef *gpio = (GPIO_TypeDef *)dshot_config_.gpio;
  
  /* Enable GPIO clock */
  if (gpio == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  } else if (gpio == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  } else if (gpio == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  
  GPIO_InitStruct.Pin = 0;
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    GPIO_InitStruct.Pin |= (1 << dshot_config_.pin_index_arr[i]);
  }
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(gpio, &GPIO_InitStruct);
#elif defined(DSHOT_PLATFORM_AT32)
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable((crm_periph_clock_type)(rt_uint32_t)dshot_config_.gpio_clock, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    gpio_init_struct.gpio_pins |= (GPIO_PINS_0 << dshot_config_.pin_index_arr[i]);
  }
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init((gpio_type *)dshot_config_.gpio, &gpio_init_struct);
#endif
}

/* Initialize DShot timer */
void dshotTimerInit(void) {
  rt_uint32_t pclk1_doubler = 1, pclk2_doubler = 1;
  rt_uint32_t tmr_clock = 0;
  rt_uint32_t dshot_send_dma_freq = 0;
  rt_uint32_t dshot_sampleing_freq = 0;

  switch (dshot_config_.dshot_protocol) {
    case (PWM_TYPE_DSHOT600):
      dshot_send_dma_freq = MOTOR_DSHOT600_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
      break;
    case (PWM_TYPE_DSHOT300):
      dshot_send_dma_freq = MOTOR_DSHOT300_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
      break;
    case (PWM_TYPE_DSHOT150):
      dshot_send_dma_freq = MOTOR_DSHOT150_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
      break;
    default:
      break;
  }

  /*the erpm bit rate is bi-dshot 5/4*/
  dshot_sampleing_freq = dshot_send_dma_freq * 5 / 4;

#ifdef DSHOT_PLATFORM_STM32
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  TIM_TypeDef *tim_instance = NULL;
  
  /* Get timer instance based on DSHOT_TIMER_SELECT */
  switch (DSHOT_TIMER_SELECT) {
    case 1: tim_instance = TIM1; break;
    case 2: tim_instance = TIM2; break;
    case 3: tim_instance = TIM3; break;
    case 4: tim_instance = TIM4; break;
    case 5: tim_instance = TIM5; break;
    case 6: tim_instance = TIM6; break;
    case 7: tim_instance = TIM7; break;
    case 8: tim_instance = TIM8; break;
#ifdef TIM9
    case 9: tim_instance = TIM9; break;
#endif
#ifdef TIM10
    case 10: tim_instance = TIM10; break;
#endif
#ifdef TIM11
    case 11: tim_instance = TIM11; break;
#endif
    default: tim_instance = TIM4; break;
  }
  
  /* Enable timer clock */
  stm32_tim_enable_clock(htim);
  
  /* Get timer clock frequency with doubler */
  stm32_tim_pclkx_doubler_get(&pclk1_doubler, &pclk2_doubler);
  
  if ((tim_instance == TIM1) || (tim_instance == TIM8)
#ifdef TIM9
      || (tim_instance == TIM9)
#endif
#ifdef TIM10
      || (tim_instance == TIM10)
#endif
#ifdef TIM11
      || (tim_instance == TIM11)
#endif
      ) {
    tmr_clock = HAL_RCC_GetPCLK2Freq() * pclk2_doubler;
  } else {
    tmr_clock = HAL_RCC_GetPCLK1Freq() * pclk1_doubler;
  }
  
  dshot_config_.timer_count_send = tmr_clock / dshot_send_dma_freq - 1;
  dshot_config_.timer_count_rec = tmr_clock / dshot_sampleing_freq - 1;
  
  /* Initialize timer */
  htim->Instance = tim_instance;
  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = dshot_config_.timer_count_send;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(htim) != HAL_OK) {
    return;
  }
  
  __HAL_TIM_ENABLE(htim);
  
#elif defined(DSHOT_PLATFORM_AT32)
  crm_clocks_freq_type clocks_struct;
  tmr_type *tmr_x = (tmr_type *)dshot_config_.timer_x;

  // Get timer clock doubler values
  crm_clocks_freq_get(&clocks_struct);
  if (clocks_struct.ahb_freq != clocks_struct.apb1_freq) {
    pclk1_doubler = 2;
  }
  if (clocks_struct.ahb_freq != clocks_struct.apb2_freq) {
    pclk2_doubler = 2;
  }
  if ((tmr_x == TMR1) || (tmr_x == TMR8) || (tmr_x == TMR9) || (tmr_x == TMR10) || (tmr_x == TMR11)) {
    tmr_clock = clocks_struct.apb2_freq * pclk2_doubler;
  } else {
    tmr_clock = clocks_struct.apb1_freq * pclk1_doubler;
  }
  dshot_config_.timer_count_send = tmr_clock / dshot_send_dma_freq - 1;
  dshot_config_.timer_count_rec = tmr_clock / dshot_sampleing_freq - 1;

  crm_periph_clock_enable((crm_periph_clock_type)(rt_uint32_t)dshot_config_.timer_clock, TRUE);
  tmr_cnt_dir_set(tmr_x, TMR_COUNT_UP);
  tmr_counter_enable(tmr_x, TRUE);
#endif
}

/* Initialize DShot DMA configuration */
void dshotDmaConfigureInit(void) {
#ifdef DSHOT_PLATFORM_STM32
  DMA_HandleTypeDef hdma;
  
  /* STM32 dma_config doesn't have dma_done member */
  
  /* Enable DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  
  /* Configure DMA for input (GPIO IDR -> memory) */
  hdma.Instance = (DMA_Stream_TypeDef *)dshot_config_.dma_cfg->Instance;
#if defined(SOC_SERIES_STM32H7)
  hdma.Init.Request = dshot_config_.dma_cfg->request;
#else
  hdma.Init.Channel = dshot_config_.dma_cfg->channel;
#endif
  hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma.Init.MemInc = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma.Init.Mode = DMA_NORMAL;
  hdma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
  
  /* These are not part of Init structure, set them directly */
  /* Note: PeriphAddr and Mem0Addr are set via HAL_DMA_Start_IT, not in Init */
  
  if (HAL_DMA_Init(&hdma) != HAL_OK) {
    return;
  }
  
  /* Save DMA registers for input */
  bbSaveDMARegs((void *)hdma.Instance, &dshot_config_.dmaRegInput);
  
  /* Save DMA handle for interrupt handling */
  dshot_dma_handle_static = hdma;
  
  /* Configure DMA for output (memory -> GPIO ODR) */
  hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  
  if (HAL_DMA_Init(&hdma) != HAL_OK) {
    return;
  }
  
  /* Save DMA registers for output */
  bbSaveDMARegs((void *)hdma.Instance, &dshot_config_.dmaRegOutput);
  
  /* Update DMA handle for interrupt handling (output config) */
  dshot_dma_handle_static = hdma;
  
  /* Set DMA transfer complete callback - declared in dshotHwOpt.c */
  extern void dshot_dma_xfer_cplt_callback(DMA_HandleTypeDef *hdma);
  dshot_dma_handle_static.XferCpltCallback = dshot_dma_xfer_cplt_callback;
  
  /* Enable DMA transfer complete interrupt */
  __HAL_DMA_ENABLE_IT(&dshot_dma_handle_static, DMA_IT_TC);
  
  /* Enable DMA interrupt in NVIC */
  HAL_NVIC_SetPriority(dshot_config_.dma_cfg->dma_irq, 1, 0);
  HAL_NVIC_EnableIRQ(dshot_config_.dma_cfg->dma_irq);
  
#elif defined(DSHOT_PLATFORM_AT32)
  dma_init_type dmainit;
  dshot_config_.dma_cfg->dma_done = RT_TRUE;

  crm_periph_clock_enable((crm_periph_clock_type)(rt_uint32_t)dshot_config_.dma_cfg->dma_clock, TRUE);
  dmamux_enable(dshot_config_.dma_cfg->dma_x, TRUE);
  dmamux_init(dshot_config_.dma_cfg->dmamux_channel, (dmamux_requst_id_sel_type)dshot_config_.dma_cfg->request_id);

  // dma for input
  dma_default_para_init(&dmainit);
  dma_reset((dma_channel_type *)dshot_config_.dma_cfg->dma_channel);
  dmainit.loop_mode_enable = FALSE;
  dmainit.peripheral_inc_enable = FALSE;
  dmainit.memory_inc_enable = TRUE;
  dmainit.priority = DMA_PRIORITY_VERY_HIGH;
  dmainit.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dmainit.buffer_size = RECEIVE_BUFFER_NUMBER;
  dmainit.peripheral_base_addr = convert_dma_input_io_addr();
  dmainit.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dmainit.memory_base_addr = (rt_uint32_t)(dshot_config_.dshot_dma_rec_buf);
  dmainit.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init((dma_channel_type *)dshot_config_.dma_cfg->dma_channel, &dmainit);
  dma_interrupt_enable((dma_channel_type *)dshot_config_.dma_cfg->dma_channel, DMA_FDT_INT, TRUE);
  bbSaveDMARegs(dshot_config_.dma_cfg->dma_channel, &dshot_config_.dmaRegInput);

  // dma for output
  dma_default_para_init(&dmainit);
  dma_reset((dma_channel_type *)dshot_config_.dma_cfg->dma_channel);
  dmainit.loop_mode_enable = FALSE;
  dmainit.peripheral_inc_enable = FALSE;
  dmainit.memory_inc_enable = TRUE;
  dmainit.priority = DMA_PRIORITY_VERY_HIGH;
  dmainit.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dmainit.buffer_size = MOTOR_DSHOT_BUF_LENGTH;
  dmainit.peripheral_base_addr = convert_dma_output_io_addr();
  dmainit.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  dmainit.memory_base_addr = (rt_uint32_t)(dshot_config_.dshot_dma_send_buf);
  dmainit.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
  dma_init((dma_channel_type *)dshot_config_.dma_cfg->dma_channel, &dmainit);
  dma_interrupt_enable((dma_channel_type *)dshot_config_.dma_cfg->dma_channel, DMA_FDT_INT, TRUE);
  bbSaveDMARegs(dshot_config_.dma_cfg->dma_channel, &dshot_config_.dmaRegOutput);

  nvic_irq_enable(dshot_config_.dma_cfg->dma_irqn, 1, 0);
#endif
}