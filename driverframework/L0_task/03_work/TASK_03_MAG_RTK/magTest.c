#include <rtdevice.h>
#include <rtthread.h>
#include <stdarg.h>

#include "I2cInterface.h"
#include "uartConfig.h"
#include "qmc6309_selftest.h"

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

/* 重测机制配置 */
#define MAG_TEST_RETRY_COUNT 3      /* 重测次数 */
#define MAG_TEST_RETRY_INTERVAL 15  /* 每次重测间隔(ms) */

typedef struct {
  rt_device_t dev;
  rt_size_t size;
} mag_uart_rx_msg_t;

static rt_device_t g_mag_uart = RT_NULL;
static struct rt_messagequeue g_rx_mq;
static char g_msg_pool[256];
static struct serial_configure g_uart_cfg;

static I2cInterface_t g_i2c_interface;

/* 简单环形缓冲实现 */
#define RINGBUF_SIZE 256
static uint8_t g_ringbuf[RINGBUF_SIZE];
static volatile uint16_t g_ring_head = 0;  // 写入位置
static volatile uint16_t g_ring_tail = 0;  // 读取位置

static inline uint16_t ringbuf_next(uint16_t v) { return (uint16_t)((v + 1) % RINGBUF_SIZE); }
static inline rt_bool_t ringbuf_is_empty(void) { return g_ring_head == g_ring_tail ? RT_TRUE : RT_FALSE; }
static inline void ringbuf_push(uint8_t b) {
  uint16_t next = ringbuf_next(g_ring_head);
  if (next != g_ring_tail) {
    g_ringbuf[g_ring_head] = b;
    g_ring_head = next;
  }
}
static inline rt_bool_t ringbuf_pop(uint8_t* b) {
  if (ringbuf_is_empty()) return RT_FALSE;
  *b = g_ringbuf[g_ring_tail];
  g_ring_tail = ringbuf_next(g_ring_tail);
  return RT_TRUE;
}

/* UART 回调：向消息队列投递长度消息 */
static rt_err_t mag_uart_rx_ind(rt_device_t dev, rt_size_t size) {
  mag_uart_rx_msg_t msg;
  msg.dev = dev;
  msg.size = size;
  rt_err_t result = rt_mq_send(&g_rx_mq, &msg, sizeof(msg));
  if (result == -RT_EFULL) {
    rt_kprintf("[MAG_TEST] mq full\n");
  }
  return result;
}

/* 简单协议校验：0x55 0x55 0x01 0x00 CRC8(前四字节) */
static uint8_t calc_crc8(const uint8_t* buf, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

/* 命令类型定义 */
#define CMD_TYPE_NONE    0
#define CMD_TYPE_ID_TEST 1  /* ID 测试命令：0x55 0x55 0x01 0x00 */
#define CMD_TYPE_SELFTEST 2 /* 自测命令：0x55 0x55 0x02 0x00 */

/* 解析触发帧，返回命令类型，0 表示未匹配 */
static uint8_t try_parse_command(void) {
  /* 寻找帧头 */
  uint8_t b;
  while (1) {
    /* 至少需要 5 字节 */
    uint16_t available = (g_ring_head >= g_ring_tail) ? (g_ring_head - g_ring_tail)
                                                      : (RINGBUF_SIZE - (g_ring_tail - g_ring_head));
    if (available < 5) return CMD_TYPE_NONE;

    /* 对齐到 0x55 0x55 */
    uint16_t saved_tail = g_ring_tail;
    if (!ringbuf_pop(&b)) return CMD_TYPE_NONE;
    if (b != 0x55) {
      continue;
    }
    if (!ringbuf_pop(&b)) {
      g_ring_tail = saved_tail;
      return CMD_TYPE_NONE;
    }
    if (b != 0x55) {
      /* 第二字节不是 0x55，则从第二字节重新开始 */
      continue;
    }
    /* 读取 cmd, data, crc */
    uint8_t frame[5];
    frame[0] = 0x55; frame[1] = 0x55;
    if (!ringbuf_pop(&frame[2])) { g_ring_tail = saved_tail; return CMD_TYPE_NONE; }
    if (!ringbuf_pop(&frame[3])) { g_ring_tail = saved_tail; return CMD_TYPE_NONE; }
    if (!ringbuf_pop(&frame[4])) { g_ring_tail = saved_tail; return CMD_TYPE_NONE; }

    /* 校验 CRC */
    uint8_t crc = calc_crc8(frame, 4);
    if (crc != frame[4]) {
      /* CRC 不匹配，继续搜索 */
      continue;
    }

    /* 根据命令码返回命令类型 */
    if (frame[2] == 0x01 && frame[3] == 0x00) {
      return CMD_TYPE_ID_TEST;
    } else if (frame[2] == 0x02 && frame[3] == 0x00) {
      return CMD_TYPE_SELFTEST;
    }
    /* 未匹配的命令，继续搜索 */
  }
}

static rt_err_t mag_i2c_init(void) {
  rt_err_t result = get_i2c_interface(WORK_TASK_MAG_TEST_6308_I2C_NAME, WORK_TASK_MAG_TEST_6308_I2C_ADDR, &g_i2c_interface);
  if (result != RT_EOK) {
    rt_kprintf("[MAG_TEST] get i2c interface fail\n");
    return result;
  }
  return RT_EOK;
}

static rt_err_t mag_i2c_read_reg(uint8_t reg, uint8_t* val) {
  uint8_t data = 0;
  int8_t ret = i2c_read_reg8_mult_pack(g_i2c_interface, reg, &data, 1);
  if (ret != 0) return -RT_ERROR;
  *val = data;
  return RT_EOK;
}

/* 自测输出函数：通过 UART 输出自测过程信息 */
static void mag_selftest_output(const char* str) {
  if (g_mag_uart != RT_NULL && str != RT_NULL) {
    rt_device_write(g_mag_uart, 0, str, rt_strlen(str));
  }
}

static void mag_thread_entry(void* parameter) {
  rt_err_t result;
  mag_uart_rx_msg_t msg;

  char* rx_buffer = RT_NULL;
  rx_buffer = rt_malloc(g_uart_cfg.rx_bufsz + 1);
  if (rx_buffer == RT_NULL) {
    rt_kprintf("[MAG_TEST] malloc fail\n");
    return;
  }

  while (1) {
    rt_memset(&msg, 0, sizeof(msg));
    rt_memset(rx_buffer, 0, g_uart_cfg.rx_bufsz + 1);
    result = rt_mq_recv(&g_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
    if (result > 0) {
      rt_size_t rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
      for (rt_size_t i = 0; i < rx_length; i++) {
        ringbuf_push((uint8_t)rx_buffer[i]);
      }

      uint8_t cmd_type = try_parse_command();
      
      if (cmd_type == CMD_TYPE_ID_TEST) {
        /* ID 测试命令：触发 I2C 读取并比对，使用重测机制防止误判 */
        rt_bool_t test_result = RT_FALSE;
        uint8_t id_val = 0;

        /* 进行多次重测，任意一次成功即判定为OK */
        for (uint8_t retry = 0; retry < MAG_TEST_RETRY_COUNT; retry++) {
          rt_err_t read_result = mag_i2c_read_reg(WORK_TASK_MAG_TEST_6308_ID_REG, &id_val);

          if (read_result == RT_EOK && id_val == WORK_TASK_MAG_TEST_6308_ID_EXPECT) {
            test_result = RT_TRUE;
            break;
          }

          /* 本次重测失败，如果还有余下尝试，则延时后继续 */
          if (retry < MAG_TEST_RETRY_COUNT - 1) {
            rt_thread_mdelay(MAG_TEST_RETRY_INTERVAL);
          }
        }

        /* 根据重测结果返回响应 */
        if (test_result == RT_TRUE) {
          const char ok_msg[] = "MAG6308 OK\r\n";
          rt_device_write(g_mag_uart, 0, ok_msg, sizeof(ok_msg) - 1);
        } else {
          const char ng_msg[] = "MAG6308 FAIL\r\n";
          rt_device_write(g_mag_uart, 0, ng_msg, sizeof(ng_msg) - 1);
        }
      } else if (cmd_type == CMD_TYPE_SELFTEST) {
        /* 自测命令：执行 QMC6309 自测功能 */
        /* 通过串口输出自测过程信息 */
        int selftest_result = qmc6309_self_test(&g_i2c_interface, mag_selftest_output);
        
        /* 最终结果已在自测函数中输出，这里不再重复输出 */
        (void)selftest_result;
      }
    }
  }
}

static int mag_test_init(void) {
  rt_err_t ret = RT_EOK;
  char uart_name[RT_NAME_MAX];

  rt_strncpy(uart_name, WORK_TASK_MAG_TEST_6308_UART_NAME, RT_NAME_MAX);

  g_mag_uart = rt_device_find(uart_name);
  if (!g_mag_uart) {
    rt_kprintf("[MAG_TEST] find uart %s fail\n", uart_name);
    return -RT_ERROR;
  }

  ret = rt_mq_init(&g_rx_mq, "mag_rx_mq", g_msg_pool, sizeof(mag_uart_rx_msg_t), sizeof(g_msg_pool), RT_IPC_FLAG_FIFO);
  if (ret != RT_EOK) {
    rt_kprintf("[MAG_TEST] mq init fail\n");
    return ret;
  }

  ret = rt_device_open(g_mag_uart, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
  if (ret != RT_EOK) {
    rt_kprintf("[MAG_TEST] open uart fail\n");
    return ret;
  }

  ret = uart_config_by_device_name(uart_name, WORK_TASK_MAG_TEST_6308_UART_BAUD, &g_uart_cfg);
  if (ret != RT_EOK) {
    rt_kprintf("[MAG_TEST] get uart cfg fail\n");
    return ret;
  }
  ret = rt_device_control(g_mag_uart, RT_DEVICE_CTRL_CONFIG, &g_uart_cfg);
  if (ret != RT_EOK) {
    rt_kprintf("[MAG_TEST] config uart fail\n");
    return ret;
  }

  /* 关闭 UART DMA RX 半/全传输中断，行为对齐 sbus.c */
  rt_device_control(g_mag_uart, RT_DEVICE_CTRL_UART_DMA_RX_DISABLE_HALF_FULL_INT, RT_NULL);

  rt_device_set_rx_indicate(g_mag_uart, mag_uart_rx_ind);

  if (mag_i2c_init() != RT_EOK) {
    rt_kprintf("[MAG_TEST] i2c init fail\n");
    return -RT_ERROR;
  }

  rt_thread_t thread = rt_thread_create("mag_tst", mag_thread_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (thread != RT_NULL) {
    rt_thread_startup(thread);
    rt_kprintf("[MAG_TEST] start on %s, baud:%d, i2c:%s, addr:0x%02X\n",
               uart_name, WORK_TASK_MAG_TEST_6308_UART_BAUD,
               WORK_TASK_MAG_TEST_6308_I2C_NAME, WORK_TASK_MAG_TEST_6308_I2C_ADDR);
  } else {
    rt_kprintf("[MAG_TEST] thread create fail\n");
    ret = -RT_ERROR;
  }

  return ret;
}

#ifdef WORK_TASK_MAG_TEST_6308_EN
INIT_APP_EXPORT(mag_test_init);
#endif

