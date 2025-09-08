/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "driver/rc/sbus.h"

#include <firmament.h>
#include <rtdevice.h>

#include "hal/rc/rc.h"
#include "module/system/systime.h"
#include "module/utils/ringbuffer.h"
#include "rtthread.h"

// SBUS事件定义
#define EVENT_SBUS_DATA_READY (1 << 0)

// 无效通道值指针 - 由rc_data.c设置
static uint16_t* sbus_invalid_channel_values = NULL;

// ==================== 宏定义 ====================
#ifndef min  // mod by prife
#define min(x, y) ((x) < (y) ? (x) : (y))
#endif

// SBUS协议相关宏
#define MAX_SBUS_CHANNEL 16
#define SBUS_FRAME_SIZE 25
#define SBUS_START_SYMBOL 0x0f
#define SBUS_INPUT_CHANNELS 16
#define SBUS_FLAGS_BYTE 23
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2

// SBUS数据范围宏
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

// 线程相关宏
#define SBUS_THREAD_STACK_SIZE 2048
#define SBUS_THREAD_PRIORITY 6

// ==================== 类型定义 ====================
typedef struct {
    uint16_t rc_count;
    uint16_t max_channels;
    bool sbus_failsafe;
    bool sbus_frame_drop;
    uint32_t sbus_frame_drops;
    bool sbus_data_ready;
    uint8_t sbus_lock;
    ringbuffer* sbus_rb;
    uint16_t sbus_val[MAX_SBUS_CHANNEL];
    struct rt_event* sbus_ready_event;
    rt_sem_t sbus_sem;  // 信号量成员
} sbus_decoder_t;

struct sbus_bit_pick {
    uint8_t byte;
    uint8_t rshift;
    uint8_t mask;
    uint8_t lshift;
};

// ==================== 全局变量 ====================
// SBUS解码器矩阵
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
    /*  0 */ {{0, 0, 0xff, 0}, {1, 0, 0x07, 8}, {0, 0, 0x00, 0}},
    /*  1 */ {{1, 3, 0x1f, 0}, {2, 0, 0x3f, 5}, {0, 0, 0x00, 0}},
    /*  2 */ {{2, 6, 0x03, 0}, {3, 0, 0xff, 2}, {4, 0, 0x01, 10}},
    /*  3 */ {{4, 1, 0x7f, 0}, {5, 0, 0x0f, 7}, {0, 0, 0x00, 0}},
    /*  4 */ {{5, 4, 0x0f, 0}, {6, 0, 0x7f, 4}, {0, 0, 0x00, 0}},
    /*  5 */ {{6, 7, 0x01, 0}, {7, 0, 0xff, 1}, {8, 0, 0x03, 9}},
    /*  6 */ {{8, 2, 0x3f, 0}, {9, 0, 0x1f, 6}, {0, 0, 0x00, 0}},
    /*  7 */ {{9, 5, 0x07, 0}, {10, 0, 0xff, 3}, {0, 0, 0x00, 0}},
    /*  8 */ {{11, 0, 0xff, 0}, {12, 0, 0x07, 8}, {0, 0, 0x00, 0}},
    /*  9 */ {{12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, {0, 0, 0x00, 0}},
    /* 10 */ {{13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10}},
    /* 11 */ {{15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, {0, 0, 0x00, 0}},
    /* 12 */ {{16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, {0, 0, 0x00, 0}},
    /* 13 */ {{17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03, 9}},
    /* 14 */ {{19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, {0, 0, 0x00, 0}},
    /* 15 */ {{20, 5, 0x07, 0}, {21, 0, 0xff, 3}, {0, 0, 0x00, 0}}};

// SBUS设备实例
static sbus_decoder_t sbus_decoder_;
static rt_device_t sbus_uart_device = RT_NULL;
static rt_thread_t sbus_thread = RT_NULL;
static rt_bool_t sbus_running = RT_FALSE;

// 线程栈
static rt_uint8_t sbus_thread_stack[SBUS_THREAD_STACK_SIZE];

// ==================== 函数声明 ====================
// 内部函数声明
static bool sbus_parse(sbus_decoder_t* decoder, uint8_t* frame, unsigned len);
static bool sbus_update(sbus_decoder_t* decoder);
static uint32_t sbus_input(sbus_decoder_t* decoder, const uint8_t* values, uint32_t size);
static rt_err_t sbus_decoder_init(sbus_decoder_t* decoder);
static rt_err_t sbus_lowlevel_init(const char* uart_name);
static rt_err_t sbus_uart_rx_ind(rt_device_t dev, rt_size_t size);
static void sbus_thread_entry(void* parameter);
static rt_err_t sbus_rc_control(rc_dev_t rc, int cmd, void* arg);
static rt_int16_t sbus_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val);

// ==================== 内联函数 ====================
rt_inline void sbus_lock(sbus_decoder_t* decoder) { decoder->sbus_lock = 1; }
rt_inline void sbus_unlock(sbus_decoder_t* decoder) { decoder->sbus_lock = 0; }
rt_inline uint8_t sbus_islock(sbus_decoder_t* decoder) { return decoder->sbus_lock; }
rt_inline uint8_t sbus_data_ready(sbus_decoder_t* decoder) { return decoder->sbus_data_ready; }
rt_inline void sbus_data_clear(sbus_decoder_t* decoder) { decoder->sbus_data_ready = 0; }

// ==================== 函数实现 ====================
// SBUS解码相关函数
static bool sbus_parse(sbus_decoder_t* decoder, uint8_t* frame, unsigned len) {
    if (len != SBUS_FRAME_SIZE) {
        decoder->sbus_frame_drops++;
        return false;
    }
    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != SBUS_START_SYMBOL)) {
        decoder->sbus_frame_drops++;
        return false;
    }

    unsigned chancount = (decoder->max_channels > SBUS_INPUT_CHANNELS) ? SBUS_INPUT_CHANNELS : decoder->max_channels;

    /* use the decoder matrix to extract channel data */
    for (unsigned channel = 0; channel < chancount; channel++) {
        unsigned value = 0;

        for (unsigned pick = 0; pick < 3; pick++) {
            const struct sbus_bit_pick* decode = &sbus_decoder[channel][pick];

            if (decode->mask != 0) {
                unsigned piece = frame[1 + decode->byte];
                piece >>= decode->rshift;
                piece &= decode->mask;
                piece <<= decode->lshift;

                value |= piece;
            }
        }

        /* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
        decoder->sbus_val[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    }

    /* decode switch channels if data fields are wide enough */
    if (decoder->max_channels > 17 && chancount > 15) {
        chancount = 18;

        /* channel 17 (index 16) */
        decoder->sbus_val[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
        /* channel 18 (index 17) */
        decoder->sbus_val[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
    }

    /* note the number of channels decoded */
    decoder->rc_count = chancount;

    /* decode the flags byte */
    decoder->sbus_failsafe = (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT));
    decoder->sbus_frame_drop = (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT));

    return true;
}

static bool sbus_update(sbus_decoder_t* decoder) {
    int ret;

    /*
     * 尝试读取一个完整的SBUS帧
     */
    uint8_t buf[SBUS_FRAME_SIZE];
    ret = ringbuffer_get(decoder->sbus_rb, buf, SBUS_FRAME_SIZE);

    /* if the read failed for any reason, just give up here */
    if (ret < SBUS_FRAME_SIZE) {
        return false;
    }

    /*
     * Try to decode something with what we got
     */
    bool sbus_updated = sbus_parse(decoder, buf, ret);
    decoder->sbus_data_ready = sbus_updated && !decoder->sbus_failsafe && !decoder->sbus_frame_drop;

    // 如果数据准备好，发送事件
    if (decoder->sbus_data_ready && decoder->sbus_ready_event != NULL) {
        rt_event_send(decoder->sbus_ready_event, EVENT_SBUS_DATA_READY);
    }

    return decoder->sbus_data_ready;
}

static uint32_t sbus_input(sbus_decoder_t* decoder, const uint8_t* values, uint32_t size) {
    return ringbuffer_put(decoder->sbus_rb, values, size);
}

// 硬件初始化相关函数
static rt_err_t sbus_lowlevel_init(const char* uart_name) {
    // 查找串口设备
    sbus_uart_device = rt_device_find(uart_name);
    if (sbus_uart_device == RT_NULL) {
        rt_kprintf("SBUS: Cannot find UART device: %s\n", uart_name);
        return RT_ERROR;
    }

    // 配置串口参数
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 100000;  // SBUS波特率100000
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_2;  // SBUS使用2个停止位
    config.parity = PARITY_EVEN;     // SBUS使用偶校验

    rt_device_control(sbus_uart_device, RT_DEVICE_CTRL_CONFIG, &config);

    // 设置接收回调
    rt_device_set_rx_indicate(sbus_uart_device, sbus_uart_rx_ind);

    // 打开串口设备
    if (rt_device_open(sbus_uart_device, RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
        rt_kprintf("SBUS: Failed to open UART device: %s\n", uart_name);
        return RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t sbus_decoder_init(sbus_decoder_t* decoder) {
    if (decoder == NULL) {
        return RT_EINVAL;
    }

    rt_memset(decoder, 0, sizeof(sbus_decoder_t));

    decoder->max_channels = MAX_SBUS_CHANNEL;

    decoder->sbus_rb = ringbuffer_create(SBUS_FRAME_SIZE * 2);
    if (decoder->sbus_rb == NULL) {
        rt_kprintf("SBUS: Fail to create sbus ring buffer!\n");
        return RT_ENOMEM;
    }

    // 初始化事件对象
    static struct rt_event event_sbus_data_ready;
    rt_event_init(&event_sbus_data_ready, "event_sbus_data_ready", RT_IPC_FLAG_PRIO);
    decoder->sbus_ready_event = &event_sbus_data_ready;

    // 初始化信号量
    decoder->sbus_sem = rt_sem_create("sbus_sem", 0, RT_IPC_FLAG_FIFO);
    if (decoder->sbus_sem == RT_NULL) {
        rt_kprintf("SBUS: Fail to create sbus semaphore!\n");
        return RT_ENOMEM;
    }

    return RT_EOK;
}

// 串口中断和线程相关函数
static rt_err_t sbus_uart_rx_ind(rt_device_t dev, rt_size_t size) {
    uint8_t ch;
    while (rt_device_read(dev, -1, &ch, 1) > 0) {
        sbus_input(&sbus_decoder_, &ch, 1);
    }
    // 释放信号量，通知解析线程有数据到达
    if (sbus_decoder_.sbus_sem != RT_NULL) {
        rt_sem_release(sbus_decoder_.sbus_sem);
    }
    return RT_EOK;
}

static void sbus_thread_entry(void* parameter) {
    while (sbus_running) {
        // 等待信号量，同步数据到达
        if (rt_sem_take(sbus_decoder_.sbus_sem, RT_WAITING_FOREVER) == RT_EOK) {
            // 检查是否有足够数据进行解析
            if (!sbus_islock(&sbus_decoder_)) {
                uint32_t available_len = ringbuffer_getlen(sbus_decoder_.sbus_rb);
                if (available_len >= SBUS_FRAME_SIZE) {
                    sbus_update(&sbus_decoder_);
                }
            }
        }
    }
}

// RC设备操作函数
static rt_err_t sbus_rc_control(rc_dev_t rc, int cmd, void* arg) {
    switch (cmd) {
        case RC_CMD_CHECK_UPDATE: {
            uint8_t updated = 0;
            if (rc->config.protocol == RC_PROTOCOL_SBUS) {
                updated = sbus_data_ready(&sbus_decoder_);
            }
            *(uint8_t*)arg = updated;
        } break;

        default:
            break;
    }

    return RT_EOK;
}

static rt_int16_t sbus_rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val) {
    rt_int16_t fun_ret = 0;
    uint16_t channl_index = 0;
    rt_err_t event_ret = 0;

    if (rc->config.protocol == RC_PROTOCOL_SBUS) {
        // 等待SBUS数据准备好事件，超时时间100ms
        event_ret = rt_event_recv(sbus_decoder_.sbus_ready_event, EVENT_SBUS_DATA_READY,
                                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 100, NULL);

        if (event_ret != 0) {
            // 超时或事件获取失败，返回无效通道值
            for (uint8_t i = 0; i < min(rc->config.channel_num, sbus_decoder_.rc_count); i++) {
                *(chan_val++) =
                    (sbus_invalid_channel_values != NULL) ? sbus_invalid_channel_values[i] : 1500;  // 默认中位值
                channl_index += 2;
            }
            fun_ret = -1;
        } else {
            // 事件获取成功，返回有效通道值
            sbus_lock(&sbus_decoder_);
            for (uint8_t i = 0; i < min(rc->config.channel_num, sbus_decoder_.rc_count); i++) {
                *(chan_val++) = sbus_decoder_.sbus_val[i];
                channl_index += 2;
            }
            sbus_data_clear(&sbus_decoder_);
            sbus_unlock(&sbus_decoder_);
            fun_ret = channl_index;
        }
    }

    return fun_ret;
}

// RC设备结构体定义
static const struct rc_ops sbus_rc_ops = {
    .rc_config = NULL,
    .rc_control = sbus_rc_control,
    .rc_read = sbus_rc_read,
};

static struct rc_device sbus_rc_dev = {
    .config =
        {
            .protocol = RC_PROTOCOL_SBUS,
            .channel_num = 16,
            .sample_time = 0.05f,
            .rc_min_value = 1000,
            .rc_max_value = 2000,
        },
    .ops = &sbus_rc_ops,
};

// 公共接口函数
rt_err_t drv_rc_sbus_init(const char* uart_name, uint16_t* invalid_channel_values) {
    rt_err_t result;

    // 设置无效通道值
    if (invalid_channel_values != NULL) {
        sbus_invalid_channel_values = invalid_channel_values;
    }

    // 初始化解码器
    result = sbus_decoder_init(&sbus_decoder_);
    if (result != RT_EOK) {
        rt_kprintf("SBUS: Failed to init decoder\n");
        return result;
    }

    // 初始化底层硬件
    result = sbus_lowlevel_init(uart_name);
    if (result != RT_EOK) {
        rt_kprintf("SBUS: Failed to init lowlevel\n");
        rt_sem_delete(sbus_decoder_.sbus_sem);
        return result;
    }

    // 创建数据处理线程
    sbus_running = RT_TRUE;
    sbus_thread = rt_thread_create("sbus_thread", sbus_thread_entry, RT_NULL, sizeof(sbus_thread_stack),
                                   SBUS_THREAD_PRIORITY, 20);
    if (sbus_thread == RT_NULL) {
        rt_kprintf("SBUS: Failed to create thread\n");
        sbus_running = RT_FALSE;
        rt_sem_delete(sbus_decoder_.sbus_sem);
        return RT_ERROR;
    }

    // 启动线程
    rt_thread_startup(sbus_thread);

    // 注册RC设备
    result = hal_rc_register(&sbus_rc_dev, "rc_sbus", RT_DEVICE_FLAG_RDWR, NULL);
    if (result != RT_EOK) {
        rt_kprintf("SBUS: Failed to register RC device\n");
        sbus_running = RT_FALSE;
        rt_thread_delete(sbus_thread);
        rt_sem_delete(sbus_decoder_.sbus_sem);
        return result;
    }

    rt_kprintf("SBUS: Initialized with UART: %s (synchronized mode)\n", uart_name);
    return RT_EOK;
}
