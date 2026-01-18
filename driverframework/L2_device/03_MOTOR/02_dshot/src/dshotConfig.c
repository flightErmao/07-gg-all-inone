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

/* DShot symbol rates (bit rates)
 * DSHOT600: 600kbit/s, bit period = 1.67µs, T1H=1.25µs, T0H=0.625µs, frame=26.72µs
 * DSHOT300: 300kbit/s, bit period = 3.33µs, T1H=2.50µs, T0H=1.25µs, frame=53.28µs
 * DSHOT150: 150kbit/s, bit period = 6.67µs, T1H=5.00µs, T0H=2.50µs, frame=106.72µs
 * 
 * Note: 定时器中断频率（DMA触发频率）使用T0H时间的倒数：
 * - DSHOT600: 1.6MHz (中断间隔 = 0.625µs = T0H)
 * - DSHOT300: 0.8MHz (中断间隔 = 1.25µs = T0H)
 * - DSHOT150: 0.4MHz (中断间隔 = 2.5µs = T0H)
 * 每个bit有3个状态，每个状态需要一次定时器中断触发DMA传输
 */
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
    } else if (port_letter == 'D') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOD;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOD;
#endif
    } else if (port_letter == 'E') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOE;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOE;
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
  } else if (port_str[0] == 'D') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOD;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOD;
#endif
  } else if (port_str[0] == 'E') {
#ifdef DSHOT_PLATFORM_STM32
      return (void *)GPIOE;
#elif defined(DSHOT_PLATFORM_AT32)
      return (void *)GPIOE;
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
    } else if (port_letter == 'D') {
      return (void *)CRM_GPIOD_PERIPH_CLOCK;
    } else if (port_letter == 'E') {
      return (void *)CRM_GPIOE_PERIPH_CLOCK;
    }
  }
  // Fallback: check first character for backward compatibility
  else if (port_str[0] == 'A') {
    return (void *)CRM_GPIOA_PERIPH_CLOCK;
  } else if (port_str[0] == 'B') {
    return (void *)CRM_GPIOB_PERIPH_CLOCK;
  } else if (port_str[0] == 'C') {
    return (void *)CRM_GPIOC_PERIPH_CLOCK;
  } else if (port_str[0] == 'D') {
    return (void *)CRM_GPIOD_PERIPH_CLOCK;
  } else if (port_str[0] == 'E') {
    return (void *)CRM_GPIOE_PERIPH_CLOCK;
  }

  return (void *)CRM_GPIOB_PERIPH_CLOCK;  // default fallback
#endif
}

/* Convert timer selection to timer type */
static void *convert_timer_type(void) {
#ifdef DSHOT_PLATFORM_STM32
  /* For STM32, we need to return TIM_HandleTypeDef pointer */
  /* Set timer instance based on DSHOT_TIMER_SELECT */
  static TIM_HandleTypeDef htim_dshot = {0};
  TIM_TypeDef *tim_instance = NULL;
  
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
    default: tim_instance = TIM4; break;  // default fallback
  }
  
  htim_dshot.Instance = tim_instance;
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
  } else if (gpio == (void *)GPIOD) {
    return (rt_uint32_t)&GPIOD->IDR;
  } else if (gpio == (void *)GPIOE) {
    return (rt_uint32_t)&GPIOE->IDR;
  }
  return (rt_uint32_t)&GPIOB->IDR;
#elif defined(DSHOT_PLATFORM_AT32)
  if (gpio == (void *)GPIOA) {
    return (rt_uint32_t)&GPIOA->idt;
  } else if (gpio == (void *)GPIOB) {
    return (rt_uint32_t)&GPIOB->idt;
  } else if (gpio == (void *)GPIOC) {
    return (rt_uint32_t)&GPIOC->idt;
  } else if (gpio == (void *)GPIOD) {
    return (rt_uint32_t)&GPIOD->idt;
  } else if (gpio == (void *)GPIOE) {
    return (rt_uint32_t)&GPIOE->idt;
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
  } else if (gpio == (void *)GPIOD) {
    return (rt_uint32_t)&GPIOD->ODR;
  } else if (gpio == (void *)GPIOE) {
    return (rt_uint32_t)&GPIOE->ODR;
  }
  return (rt_uint32_t)&GPIOB->ODR;
#elif defined(DSHOT_PLATFORM_AT32)
  if (gpio == (void *)GPIOA) {
    return (rt_uint32_t)&GPIOA->odt;
  } else if (gpio == (void *)GPIOB) {
    return (rt_uint32_t)&GPIOB->odt;
  } else if (gpio == (void *)GPIOC) {
    return (rt_uint32_t)&GPIOC->odt;
  } else if (gpio == (void *)GPIOD) {
    return (rt_uint32_t)&GPIOD->odt;
  } else if (gpio == (void *)GPIOE) {
    return (rt_uint32_t)&GPIOE->odt;
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
  /* STM32: Update Event = 定时器溢出事件（计数器达到 ARR 值并重载时产生） */
  dshot_config_.tmr_dma_request = TIM_DMA_UPDATE;
#elif defined(DSHOT_PLATFORM_AT32)
  /* AT32: Overflow Event = 定时器溢出事件（计数器达到 Period 值并溢出时产生） */
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
  } else if (gpio == GPIOD) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  } else if (gpio == GPIOE) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  
  GPIO_InitStruct.Pin = 0;
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    GPIO_InitStruct.Pin |= (1 << dshot_config_.pin_index_arr[i]);
  }
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(gpio, &GPIO_InitStruct);
  
#ifdef DSHOT_DEBUGPIN_EN
#ifdef DSHOT_PLATFORM_STM32
  /* Initialize PD12 for debug pin - direct register access for fast toggle */
  /* Enable GPIOD clock if not already enabled */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PD12 as output, push-pull, maximum speed */
  GPIO_InitTypeDef PD12_InitStruct = {0};
  PD12_InitStruct.Pin = GPIO_PIN_12;
  PD12_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  PD12_InitStruct.Pull = GPIO_NOPULL;  /* No pull-up/pull-down for debug pin */
  PD12_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* Maximum speed for fast toggle */
  HAL_GPIO_Init(GPIOD, &PD12_InitStruct);
  
  /* Set initial state to low */
  GPIOD->BSRR = (GPIO_PIN_12 << 16);  /* Set PD12 low */
#endif
#endif
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
      /* 定时器中断频率 = 1.6MHz (中断间隔 = 0.625µs)
       * 每个bit周期1.67µs，每个bit有3个状态，每个状态需要一次定时器中断触发DMA
       * 0.625µs × 3 ≈ 1.875µs (略大于1.67µs，但符合编码需求)
       * 0.625µs对应T0H时间（0 bit的高电平时间）
       */
      dshot_send_dma_freq = 1600000;  /* 1.6MHz = 1/0.625µs */
      break;
    case (PWM_TYPE_DSHOT300):
      /* 定时器中断频率 = 0.8MHz (中断间隔 = 1.25µs)
       * 每个bit周期3.33µs，每个bit有3个状态
       * 1.25µs × 3 = 3.75µs (略大于3.33µs)
       * 1.25µs对应T0H时间
       */
      dshot_send_dma_freq = 800000;  /* 0.8MHz = 1/1.25µs */
      break;
    case (PWM_TYPE_DSHOT150):
      /* 定时器中断频率 = 0.4MHz (中断间隔 = 2.5µs)
       * 每个bit周期6.67µs，每个bit有3个状态
       * 2.5µs × 3 = 7.5µs (略大于6.67µs)
       * 2.5µs对应T0H时间
       */
      dshot_send_dma_freq = 400000;  /* 0.4MHz = 1/2.5µs */
      break;
    default:
      break;
  }

  /*the erpm bit rate is bi-dshot 5/4*/
  dshot_sampleing_freq = dshot_send_dma_freq * 5 / 4;

#ifdef DSHOT_PLATFORM_STM32
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)dshot_config_.timer_x;
  TIM_TypeDef *tim_instance = htim->Instance;  /* Instance already set in convert_timer_type() */
  
  if (tim_instance == NULL) {
    return;  /* Timer instance not set, initialization failed */
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
  /* Note: htim->Instance is already set in convert_timer_type() */
  htim->Init.Prescaler = 0;  /* Critical: Prescaler must be 0 for correct timing */
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = dshot_config_.timer_count_send;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(htim) != HAL_OK) {
    return;
  }
  
  /* Ensure Prescaler is set to 0 after HAL initialization (HAL may modify it) */
  __HAL_TIM_SET_PRESCALER(htim, 0);
  
  /* Ensure Repetition Counter (RCR) is 0 - critical for UPDATE event frequency */
  /* RCR causes UPDATE event only after (RCR+1) overflows, default should be 0 */
  if (htim->Instance->RCR != 0) {
    rt_kprintf("[DSHOT] WARNING: RCR is %u, expected 0! Fixing...\n", htim->Instance->RCR);
    htim->Instance->RCR = 0;  /* Force set to 0 */
  }
  
  /* Set counter initial value to 0 */
  __HAL_TIM_SET_COUNTER(htim, 0);
  
  /* Verify Prescaler is actually 0 */
  if (htim->Instance->PSC != 0) {
    rt_kprintf("[DSHOT] WARNING: Prescaler is %u, expected 0! Fixing...\n", htim->Instance->PSC);
    htim->Instance->PSC = 0;  /* Force set to 0 */
  }
  
  /* Ensure timer DMA request is disabled during initialization */
  __HAL_TIM_DISABLE_DMA(htim, dshot_config_.tmr_dma_request);
  
  /* Debug: Print essential timer configuration */
  rt_uint32_t actual_prescaler = htim->Instance->PSC + 1;
  rt_uint32_t actual_period = htim->Instance->ARR + 1;
  rt_uint32_t actual_timer_freq = tmr_clock / actual_prescaler;
  rt_uint32_t actual_interrupt_freq = actual_timer_freq / actual_period;
  float actual_interrupt_period_us = 1000000.0f / actual_interrupt_freq;
  
  rt_kprintf("[DSHOT] Timer: DSHOT%d, tmr_clock=%luHz, PSC=%u, ARR=%u, freq=%luHz (%.3fus), expected=%luHz\n",
             (dshot_config_.dshot_protocol == PWM_TYPE_DSHOT600) ? 600 :
             (dshot_config_.dshot_protocol == PWM_TYPE_DSHOT300) ? 300 : 150,
             tmr_clock, htim->Instance->PSC, htim->Instance->ARR, 
             actual_interrupt_freq, actual_interrupt_period_us, dshot_send_dma_freq);
  
  if (actual_interrupt_freq != dshot_send_dma_freq) {
    rt_kprintf("[DSHOT] WARNING: freq mismatch! diff=%.1f%%\n",
               ((float)actual_interrupt_freq - (float)dshot_send_dma_freq) * 100.0f / dshot_send_dma_freq);
  }
  
#ifdef DSHOT_DEBUGPIN_EN
  /* Enable timer update interrupt for debugging */
  __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
  /* Enable timer interrupt in NVIC */
  rt_uint32_t tim_irq = 0;
  if (tim_instance == TIM1) tim_irq = TIM1_UP_IRQn;
  else if (tim_instance == TIM2) tim_irq = TIM2_IRQn;
  else if (tim_instance == TIM3) tim_irq = TIM3_IRQn;
  else if (tim_instance == TIM4) tim_irq = TIM4_IRQn;
  else if (tim_instance == TIM5) tim_irq = TIM5_IRQn;
#ifdef TIM8
  else if (tim_instance == TIM8) tim_irq = TIM8_UP_TIM13_IRQn;  /* TIM8和TIM13共享中断向量 */
#endif
#ifdef TIM9
  else if (tim_instance == TIM9) tim_irq = TIM1_BRK_TIM9_IRQn;
#endif
#ifdef TIM10
  else if (tim_instance == TIM10) tim_irq = TIM1_UP_TIM10_IRQn;
#endif
#ifdef TIM11
  else if (tim_instance == TIM11) tim_irq = TIM1_TRG_COM_TIM11_IRQn;
#endif
  if (tim_irq != 0) {
    HAL_NVIC_SetPriority(tim_irq, 2, 0);
    HAL_NVIC_EnableIRQ(tim_irq);
  }
#endif
  
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
  
#ifdef DSHOT_DEBUGPIN_EN
  /* Enable timer overflow interrupt for debugging */
  tmr_interrupt_enable(tmr_x, TMR_OVF_INT, TRUE);
  /* Enable timer interrupt in NVIC */
  rt_uint32_t tmr_irqn = 0;
  if (tmr_x == TMR1) tmr_irqn = TMR1_OVF_TMR10_IRQn;
  else if (tmr_x == TMR2) tmr_irqn = TMR2_GLOBAL_IRQn;
  else if (tmr_x == TMR3) tmr_irqn = TMR3_GLOBAL_IRQn;
  else if (tmr_x == TMR4) tmr_irqn = TMR4_GLOBAL_IRQn;
  else if (tmr_x == TMR5) tmr_irqn = TMR5_GLOBAL_IRQn;
  else if (tmr_x == TMR8) tmr_irqn = TMR8_OVF_TMR13_IRQn;
  else if (tmr_x == TMR9) tmr_irqn = TMR1_BRK_TMR9_IRQn;
  else if (tmr_x == TMR10) tmr_irqn = TMR1_OVF_TMR10_IRQn;
  else if (tmr_x == TMR11) tmr_irqn = TMR1_TRG_HALL_TMR11_IRQn;
  if (tmr_irqn != 0) {
    nvic_irq_enable(tmr_irqn, 2, 0);
  }
#endif
  
  tmr_counter_enable(tmr_x, TRUE);
#endif
}

/* Initialize DShot DMA configuration */
void dshotDmaConfigureInit(void) {
#ifdef DSHOT_PLATFORM_STM32
  /* STM32 dma_config doesn't have dma_done member */
  
  /* Enable DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Initialize DMA handle in dshot_config_ */
  memset(&dshot_config_.dma_handle, 0, sizeof(DMA_HandleTypeDef));
  
  /* Configure DMA for input (GPIO IDR -> memory) */
  dshot_config_.dma_handle.Instance = (DMA_Stream_TypeDef *)dshot_config_.dma_cfg->Instance;
#if defined(SOC_SERIES_STM32H7)
  dshot_config_.dma_handle.Init.Request = dshot_config_.dma_cfg->request;
#else
  dshot_config_.dma_handle.Init.Channel = dshot_config_.dma_cfg->channel;
#endif
  dshot_config_.dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dshot_config_.dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
  dshot_config_.dma_handle.Init.MemInc = DMA_MINC_ENABLE;
  dshot_config_.dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dshot_config_.dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  dshot_config_.dma_handle.Init.Mode = DMA_NORMAL;
  dshot_config_.dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  dshot_config_.dma_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  dshot_config_.dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  dshot_config_.dma_handle.Init.MemBurst = DMA_MBURST_SINGLE;
  dshot_config_.dma_handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
  
  /* These are not part of Init structure, set them directly */
  /* Note: PeriphAddr and Mem0Addr are set via HAL_DMA_Start_IT, not in Init */
  
  if (HAL_DMA_Init(&dshot_config_.dma_handle) != HAL_OK) {
    return;
  }
  
  /* Ensure DMA is disabled before modifying registers */
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)dshot_config_.dma_handle.Instance;
  dma_stream->CR &= ~DMA_SxCR_EN;  /* Disable DMA */
  /* Wait for disable with timeout to avoid infinite loop */
  rt_uint32_t timeout = 1000;
  while ((dma_stream->CR & DMA_SxCR_EN) && (timeout-- > 0));
  if (timeout == 0) {
    rt_kprintf("[DSHOT] Warning: DMA disable timeout\n");
  }
  
  /* Set DMA peripheral and memory addresses for input configuration */
  dma_stream->PAR = (rt_uint32_t)convert_dma_input_io_addr();  /* GPIO IDR address */
  dma_stream->M0AR = (rt_uint32_t)(dshot_config_.dshot_dma_rec_buf);  /* Receive buffer address */
  dma_stream->NDTR = RECEIVE_BUFFER_NUMBER;  /* Transfer count */
  
  /* Save DMA registers for input (DMA is disabled, so safe to save) */
  bbSaveDMARegs((void *)dshot_config_.dma_handle.Instance, &dshot_config_.dmaRegInput);
  
  /* Configure DMA for output (memory -> GPIO ODR) */
  dshot_config_.dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dshot_config_.dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dshot_config_.dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  
  if (HAL_DMA_Init(&dshot_config_.dma_handle) != HAL_OK) {
    return;
  }
  
  /* Ensure DMA is disabled before modifying registers */
  dma_stream = (DMA_Stream_TypeDef *)dshot_config_.dma_handle.Instance;
  dma_stream->CR &= ~DMA_SxCR_EN;  /* Disable DMA */
  /* Wait for disable with timeout to avoid infinite loop */
  timeout = 1000;
  while ((dma_stream->CR & DMA_SxCR_EN) && (timeout-- > 0));
  if (timeout == 0) {
    rt_kprintf("[DSHOT] Warning: DMA disable timeout\n");
  }
  
  /* Set DMA peripheral and memory addresses for output configuration */
  /* This is critical - without these addresses, DMA cannot transfer data */
  dma_stream->PAR = (rt_uint32_t)convert_dma_output_io_addr();  /* GPIO ODR address */
  dma_stream->M0AR = (rt_uint32_t)(dshot_config_.dshot_dma_send_buf);  /* Send buffer address */
  dma_stream->NDTR = MOTOR_DSHOT_BUF_LENGTH;  /* Transfer count */
  
  /* Save DMA registers for output (DMA is disabled, so safe to save) */
  bbSaveDMARegs((void *)dshot_config_.dma_handle.Instance, &dshot_config_.dmaRegOutput);
  
  /* Set DMA transfer complete callback - declared in dshotHwOpt.c */
  extern void dshot_dma_xfer_cplt_callback(DMA_HandleTypeDef *hdma);
  dshot_config_.dma_handle.XferCpltCallback = dshot_dma_xfer_cplt_callback;
  
  /* Enable DMA transfer complete interrupt */
  /* 使用 DSHOT_DMA_INT_FLAG 宏，对于 STM32H7 它等于 DMA_IT_TC */
  __HAL_DMA_ENABLE_IT(&dshot_config_.dma_handle, DSHOT_DMA_INT_FLAG);
  
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