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

/*about DMA config parameter*/
#define DSHOT_DAM_CONFIG                       \
  {                                            \
      .dma_channel = DSHOT_DMA_CHANNEL,        \
      .dma_clock = DSHOT_DMA_CLOCK,            \
      .dma_irqn = DSHOT_DMA_IRQ,               \
      .dmamux_channel = DSHOT_DMA_MUX_CHANNEL, \
      .request_id = DSHOT_DMA_REQ_ID,          \
  }

/* DShot symbol rates */
#define MOTOR_DSHOT600_SYMBOL_RATE (600 * 1000)
#define MOTOR_DSHOT300_SYMBOL_RATE (300 * 1000)
#define MOTOR_DSHOT150_SYMBOL_RATE (150 * 1000)

/* Global runtime configuration */
dshot_config_t dshot_config_ = {0};

/* Convert GPIO port string to GPIO type */
static gpio_type *convert_gpio_port(void) {
  const char *port_str = DSHOT_GPIO_PORT;
  // Check for "GPIO" prefix and get the port letter (4th character)
  if (strlen(port_str) >= 4 && strncmp(port_str, "GPIO", 4) == 0) {
    char port_letter = port_str[4];  // Get the letter after "GPIO"
    if (port_letter == 'A') {
      return GPIOA;
    } else if (port_letter == 'B') {
      return GPIOB;
    } else if (port_letter == 'C') {
      return GPIOC;
    }
  }
  // Fallback: check first character for backward compatibility
  else if (port_str[0] == 'A') {
    return GPIOA;
  } else if (port_str[0] == 'B') {
    return GPIOB;
  } else if (port_str[0] == 'C') {
    return GPIOC;
  }

  return GPIOB;  // default fallback
}

/* Convert GPIO port string to GPIO clock */
static crm_periph_clock_type convert_gpio_clock(void) {
  const char *port_str = DSHOT_GPIO_PORT;
  // Check for "GPIO" prefix and get the port letter (4th character)
  if (strlen(port_str) >= 4 && strncmp(port_str, "GPIO", 4) == 0) {
    char port_letter = port_str[4];  // Get the letter after "GPIO"
    if (port_letter == 'A') {
      return CRM_GPIOA_PERIPH_CLOCK;
    } else if (port_letter == 'B') {
      return CRM_GPIOB_PERIPH_CLOCK;
    } else if (port_letter == 'C') {
      return CRM_GPIOC_PERIPH_CLOCK;
    }
  }
  // Fallback: check first character for backward compatibility
  else if (port_str[0] == 'A') {
    return CRM_GPIOA_PERIPH_CLOCK;
  } else if (port_str[0] == 'B') {
    return CRM_GPIOB_PERIPH_CLOCK;
  } else if (port_str[0] == 'C') {
    return CRM_GPIOC_PERIPH_CLOCK;
  }

  return CRM_GPIOB_PERIPH_CLOCK;  // default fallback
}

/* Convert timer selection to timer type */
static tmr_type *convert_timer_type(void) {
  switch (DSHOT_TIMER_SELECT) {
    case 1:
      return TMR1;
    case 2:
      return TMR2;
    case 3:
      return TMR3;
    case 4:
      return TMR4;
    case 5:
      return TMR5;
    case 6:
      return TMR6;
    case 7:
      return TMR7;
    case 8:
      return TMR8;
    case 9:
      return TMR9;
    case 10:
      return TMR10;
    case 11:
      return TMR11;
    default:
      return TMR4;  // default fallback
  }
}

/* Convert timer selection to timer clock */
static crm_periph_clock_type convert_timer_clock(void) {
  switch (DSHOT_TIMER_SELECT) {
    case 1:
      return CRM_TMR1_PERIPH_CLOCK;
    case 2:
      return CRM_TMR2_PERIPH_CLOCK;
    case 3:
      return CRM_TMR3_PERIPH_CLOCK;
    case 4:
      return CRM_TMR4_PERIPH_CLOCK;
    case 5:
      return CRM_TMR5_PERIPH_CLOCK;
    case 6:
      return CRM_TMR6_PERIPH_CLOCK;
    case 7:
      return CRM_TMR7_PERIPH_CLOCK;
    case 8:
      return CRM_TMR8_PERIPH_CLOCK;
    case 9:
      return CRM_TMR9_PERIPH_CLOCK;
    case 10:
      return CRM_TMR10_PERIPH_CLOCK;
    case 11:
      return CRM_TMR11_PERIPH_CLOCK;
    default:
      return CRM_TMR4_PERIPH_CLOCK;  // default fallback
  }
}

/* Convert GPIO port to DMA peripheral register addresses */
static rt_uint32_t convert_dma_input_io_addr(void) {
  gpio_type *gpio = convert_gpio_port();
  if (gpio == GPIOA) {
    return (rt_uint32_t)&GPIOA->idt;
  } else if (gpio == GPIOB) {
    return (rt_uint32_t)&GPIOB->idt;
  } else if (gpio == GPIOC) {
    return (rt_uint32_t)&GPIOC->idt;
  }
  return (rt_uint32_t)&GPIOB->idt;
}

static rt_uint32_t convert_dma_output_io_addr(void) {
  gpio_type *gpio = convert_gpio_port();
  if (gpio == GPIOA) {
    return (rt_uint32_t)&GPIOA->odt;
  } else if (gpio == GPIOB) {
    return (rt_uint32_t)&GPIOB->odt;
  } else if (gpio == GPIOC) {
    return (rt_uint32_t)&GPIOC->odt;
  }
  return (rt_uint32_t)&GPIOB->odt;
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
  /* search dma base and channel index */
  dma_channel_type *dshot_channel = dshot_dma_temp.dma_channel;
  dshot_dma_temp.dma_x = (dma_type *)((rt_uint32_t)dshot_channel & ~0xFF);
  dshot_dma_temp.channel_index = ((((rt_uint32_t)dshot_channel & 0xFF) - 8) / 0x14) + 1;

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
  dshot_config_.tmr_dma_request = TMR_OVERFLOW_DMA_REQUEST;

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
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(dshot_config_.gpio_clock, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    gpio_init_struct.gpio_pins |= (GPIO_PINS_0 << dshot_config_.pin_index_arr[i]);
  }
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(dshot_config_.gpio, &gpio_init_struct);
}

/* Initialize DShot timer */
void dshotTimerInit(void) {
  crm_clocks_freq_type clocks_struct;
  rt_uint32_t pclk1_doubler = 1, pclk2_doubler = 1;
  rt_uint32_t tmr_clock = 0;
  rt_uint32_t dshot_send_dma_freq = 0;
  rt_uint32_t dshot_sampleing_freq = 0;
  tmr_type *tmr_x = dshot_config_.timer_x;

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

  crm_periph_clock_enable(dshot_config_.timer_clock, TRUE);
  tmr_cnt_dir_set(tmr_x, TMR_COUNT_UP);
  tmr_counter_enable(tmr_x, TRUE);
}

/* Initialize DShot DMA configuration */
void dshotDmaConfigureInit(void) {
  dma_init_type dmainit;
  dshot_config_.dma_cfg->dma_done = RT_TRUE;

  crm_periph_clock_enable(dshot_config_.dma_cfg->dma_clock, TRUE);
  dmamux_enable(dshot_config_.dma_cfg->dma_x, TRUE);
  dmamux_init(dshot_config_.dma_cfg->dmamux_channel, (dmamux_requst_id_sel_type)dshot_config_.dma_cfg->request_id);

  // dma for input
  dma_default_para_init(&dmainit);
  dma_reset(dshot_config_.dma_cfg->dma_channel);
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
  dma_init(dshot_config_.dma_cfg->dma_channel, &dmainit);
  dma_interrupt_enable(dshot_config_.dma_cfg->dma_channel, DMA_FDT_INT, TRUE);
  bbSaveDMARegs(dshot_config_.dma_cfg->dma_channel, &dshot_config_.dmaRegInput);

  // dma for output
  dma_default_para_init(&dmainit);
  dma_reset(dshot_config_.dma_cfg->dma_channel);
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
  dma_init(dshot_config_.dma_cfg->dma_channel, &dmainit);
  dma_interrupt_enable(dshot_config_.dma_cfg->dma_channel, DMA_FDT_INT, TRUE);
  bbSaveDMARegs(dshot_config_.dma_cfg->dma_channel, &dshot_config_.dmaRegOutput);

  nvic_irq_enable(dshot_config_.dma_cfg->dma_irqn, 1, 0);
}