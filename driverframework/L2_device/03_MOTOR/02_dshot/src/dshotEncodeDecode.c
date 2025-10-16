/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-25     gg           V1.0 first version
 * 2024-10-01     gg           V2.0 work normal
 * 2025-09-24     gg           V3.0 Refactored for modular design
 */

#include <rtthread.h>
#include <string.h>
#include "dshotConfig.h"
#include "dshotEncodeDecode.h"

#ifndef INVIALID_CHECK_NUM
#define INVIALID_CHECK_NUM 6
#endif

/* decoding helper structures and constants */
typedef struct {
  uint32_t edge[DSHOT_MOTOR_NUMS][RECEIVE_BUFFER_NUMBER];
  uint32_t edge_cnt[DSHOT_MOTOR_NUMS];
} edge_data_str;

typedef struct {
  rt_uint8_t old_hight_low_status;
  rt_uint8_t hight_low_status;
  rt_uint8_t start_flag;
  rt_uint16_t signal_cnt;
  rt_uint16_t vessel_number;
} edge_dispose_str;

/* External variables from main driver */
extern dshot_config_t dshot_config_;
extern rt_bool_t isBusNeedToBeIdle[DSHOT_MOTOR_NUMS];

/* remove unused failure counters to simplify */

/* Convert DShot value to GPIO port bit stream */
static void convertDshotToGpioPortBitStream(rt_uint32_t *maskedGpioBitStream, int pinNumber, uint16_t value) {
  rt_uint32_t middleBit;

  middleBit = (1 << (pinNumber + 0));

  if (dshot_config_.bi_dshot_en) {
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
      maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + pos * MOTOR_DSHOT_STATE_PER_SYMBOL + 2] |= middleBit;
      if (!(value & 0x8000)) {
        maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + pos * MOTOR_DSHOT_STATE_PER_SYMBOL + 1] |= middleBit;
      } else {
        maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + pos * MOTOR_DSHOT_STATE_PER_SYMBOL + 1] &= ~middleBit;
      }
      value <<= 1;
    }
  } else {
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
      maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + pos * MOTOR_DSHOT_STATE_PER_SYMBOL] |= middleBit;

      if (value & 0x8000) {
        maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + pos * MOTOR_DSHOT_STATE_PER_SYMBOL + 1] |= middleBit;
      } else {
        maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + pos * MOTOR_DSHOT_STATE_PER_SYMBOL + 1] &= ~middleBit;
      }
      value <<= 1;
    }
  }

  /*write the value in hold time*/
  if (dshot_config_.bi_dshot_en) {
    for (uint8_t i = 0; i < MOTOR_DSHOT_BIT_HOLD_STATES; i++) {
      maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + MOTOR_DSHOT_FRAME_BITS * MOTOR_DSHOT_STATE_PER_SYMBOL + i] |=
          middleBit;
    }

    for (uint8_t i = 0; i < MOTOR_DSHOT_BIT_BEFORE_STATES; i++) {
      maskedGpioBitStream[i] |= middleBit;
    }
  } else {
    for (uint8_t i = 0; i < MOTOR_DSHOT_BIT_HOLD_STATES; i++) {
      maskedGpioBitStream[MOTOR_DSHOT_BIT_BEFORE_STATES + MOTOR_DSHOT_FRAME_BITS * MOTOR_DSHOT_STATE_PER_SYMBOL + i] &=
          ~middleBit;
    }
  }
}

/* Merge GPIO bit stream for DMA */
static void mergeGpioBitStreamForDma(void) {
  // merge all motor's individual gpio bit stream to a final gpio byte stream
  uint32_t *dshot_send_buf;
  for (rt_uint8_t i = 0; i < MOTOR_DSHOT_BUF_LENGTH; i++) {
    for (rt_uint8_t motor_num = 0; motor_num < DSHOT_MOTOR_NUMS; motor_num++) {
      dshot_send_buf = &dshot_config_.dshot_encode_cache_buf[MOTOR_DSHOT_BUF_LENGTH * motor_num];
      dshot_config_.dshot_dma_send_buf[i] |= dshot_send_buf[i];
    }
  }
}

/* DShot Decode Function */
static uint32_t dshotDecode(uint32_t buffer[], uint32_t count) {
  volatile uint32_t value = 0;    // 差分曼码展开后的21位序列
  uint32_t oldValue = buffer[0];  // 上一个边沿时间
  int bits = 0;
  int len;

  for (uint32_t i = 1; i <= count; i++) {
    if (i < count) {
      int diff = buffer[i] - oldValue;
      if (bits >= 21) {
        return 0xffff;  // 超长，判无效
      }
      len = (diff + 3) / 6;  // 单位约6us
    } else {
      len = 21 - bits;
    }

    value <<= len;
    value |= 1u << (len - 1);
    oldValue = buffer[i];
    bits += len;
  }

  if (bits != 21) {
    return 0xffff;
  }

  static const uint32_t decode[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
                                      0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8,  1,  0, 4,  12, 0};

  uint32_t decodedValue = decode[value & 0x1f];
  decodedValue |= decode[(value >> 5) & 0x1f] << 4;
  decodedValue |= decode[(value >> 10) & 0x1f] << 8;
  decodedValue |= decode[(value >> 15) & 0x1f] << 12;

  uint32_t csum = decodedValue;
  csum = csum ^ (csum >> 8);
  csum = csum ^ (csum >> 4);
  if ((csum & 0xf) != 0xf) {
    return 0xffff;
  }

  decodedValue >>= 4;  // 去掉校验位，保留12位 eee|mmmmmmmmm

  // 0x0FFF 表示电机停止 → 返回 0（有效）
  if (decodedValue == 0x0fff) {
    return 0;
  }

  // 计算周期(μs) = 尾数 << 指数
  uint16_t exponent = (decodedValue & 0xfe00) >> 9;
  uint16_t mantissa = (decodedValue & 0x01ff);
  uint32_t period_us = (uint32_t)mantissa << exponent;
  if (period_us == 0) {
    return 0xffff;  // 防止除0
  }

  // 返回 eRPM/100 = 600000 / period_us （四舍五入）
  return (600000 + period_us / 2) / period_us;
}

/* Identify bits from received data */
static void identifyBits(uint8_t *channel, uint16_t *data, uint16_t *speed_ret) {
  uint32_t speed_temp = 0;              // 临时变量，用于存储计算的速度
  edge_data_str edge_data = {0};        // 初始化边缘数据结构
  edge_dispose_str edge_dispose = {0};  // 初始化边缘处理数据结构
  static uint32_t invalid_check[DSHOT_MOTOR_NUMS] = {0};

  // 遍历4个通道
  for (uint8_t i = 0; i < DSHOT_MOTOR_NUMS; i++) {
    edge_dispose.old_hight_low_status = 0;  // 记录上一个电平状态
    edge_dispose.hight_low_status = 0;      // 当前电平状态
    edge_dispose.start_flag = 0;            // 开始信号标志
    edge_dispose.signal_cnt = 0;            // 信号计数
    edge_dispose.vessel_number = 0;         // 边缘数据索引

    // 遍历数据中的位，最多140位
    for (int j = 0; j < RECEIVE_BUFFER_NUMBER; j++) {
      // 检查当前位是否为低电平
      if (!(data[j] >> channel[i] & 1)) {
        edge_dispose.hight_low_status = 1;  // 设置为高电平
        edge_dispose.start_flag = 1;        // 设置开始信号标志
      } else {
        edge_dispose.hight_low_status = 0;  // 设置为低电平
      }

      // 如果当前位为低电平，增加信号计数
      if (edge_dispose.start_flag) {
        edge_dispose.signal_cnt++;  // 记录信号的发生次数
      }

      // 如果电平状态发生变化
      if (edge_dispose.old_hight_low_status != edge_dispose.hight_low_status) {
        // 记录边缘信号数据，并更新信号计数
        edge_data.edge[i][edge_dispose.vessel_number++] = (edge_dispose.signal_cnt - 1) * 2;  // 计算信号位置
        edge_data.edge_cnt[i]++;                                                              // 增加当前通道的信号计数
      }

      // 更新之前的电平状态
      edge_dispose.old_hight_low_status = edge_dispose.hight_low_status;
    }

    // 调用解码函数，得到电机速度
    speed_temp = dshotDecode(&edge_data.edge[i][0], edge_data.edge_cnt[i]);
    if (speed_temp == 0xffff) {
      invalid_check[i]++;
    } else {
      invalid_check[i] = 0;
    }

    // 当解码无效，速度值先不更新（仍为历史值），当解码无效次数超过阈值，则将该通道的rpm值置为0；
    if (invalid_check[i] > INVIALID_CHECK_NUM) {
      invalid_check[i] = INVIALID_CHECK_NUM;
      speed_ret[i] = 0;
    } else {
      // 如果速度有效，则更新电机速度（单位：eRPM/100）
      if (speed_temp != 0xffff) {
        if (speed_temp < 14285) {  // 简单限幅
          speed_ret[i] = (uint16_t)speed_temp;
        }
      }
    }
  }
}

/* Set motor GPIO bit stream to all 0 */
static void setMotorBitStreamLow(uint8_t motor_id) {
  // Method 1: Fast zero-fill using memset
  memset(&dshot_config_.dshot_encode_cache_buf[MOTOR_DSHOT_BUF_LENGTH * motor_id], 0,
         sizeof(rt_uint32_t) * MOTOR_DSHOT_BUF_LENGTH);
}

/* Set motor GPIO bit stream to all 1 for this motor's pin */
static void setMotorBitStreamHigh(uint8_t motor_id) {
  // Method 2: Specific bit manipulation for this motor's pin
  rt_uint32_t pin_mask = (1 << dshot_config_.pin_index_arr[motor_id]);
  rt_uint32_t *motor_buffer = &dshot_config_.dshot_encode_cache_buf[MOTOR_DSHOT_BUF_LENGTH * motor_id];

  for (uint32_t i = 0; i < MOTOR_DSHOT_BUF_LENGTH; i++) {
    motor_buffer[i] = pin_mask;  // Set only this motor's bit to 1
  }
}

/* Pure DShot encoding for a single motor (no state handling) */
static void encodeDshotSingleMotor(uint8_t motor_id, uint16_t value) {
  // Clear the buffer for this specific motor first
  memset(&dshot_config_.dshot_encode_cache_buf[MOTOR_DSHOT_BUF_LENGTH * motor_id], 0,
         sizeof(rt_uint32_t) * MOTOR_DSHOT_BUF_LENGTH);

  // Normal DShot encoding process
  uint16_t packet = 0;
  uint8_t crc = 0;

  value = (value & 0x07FF) << 1 | 1;

  if (dshot_config_.bi_dshot_en) {
    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
  } else {
    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0f;
  }

  packet = value << 4 | crc;
  convertDshotToGpioPortBitStream(&dshot_config_.dshot_encode_cache_buf[MOTOR_DSHOT_BUF_LENGTH * motor_id],
                                  dshot_config_.pin_index_arr[motor_id], packet);
}

/* Encode DSHOT packets for all motors - unified control logic */
void encodeDshot(void) {
  // Clear all buffers
  memset(dshot_config_.dshot_encode_cache_buf, 0, sizeof(rt_uint32_t) * MOTOR_DSHOT_BUF_LENGTH * DSHOT_MOTOR_NUMS);
  memset(dshot_config_.dshot_dma_send_buf, 0, sizeof(rt_uint32_t) * MOTOR_DSHOT_BUF_LENGTH);

  // Process each motor according to motor index
  for (uint8_t motorIndex = 0; motorIndex < DSHOT_MOTOR_NUMS; motorIndex++) {
    // Priority 1: Check if bus needs to be idle (intelligent idle control)
    if (isBusNeedToBeIdle[motorIndex]) {
      if (dshot_config_.bi_dshot_en) {
        setMotorBitStreamHigh(motorIndex);  // Bi-directional mode: HIGH idle
      } else {
        setMotorBitStreamLow(motorIndex);  // Uni-directional mode: LOW idle
      }
    }
    // Priority 2: Normal DShot encoding
    else {
      encodeDshotSingleMotor(motorIndex, dshot_config_.dshot_output_value[motorIndex]);
    }
  }

  // Final step: merge all motor bit streams into one GPIO byte stream for DMA
  mergeGpioBitStreamForDma();
}

/* Decode DShot data */
void decodeDShot(void) {
  identifyBits(dshot_config_.pin_index_arr, dshot_config_.dshot_dma_rec_buf, dshot_config_.dshot_input_rpm);
}