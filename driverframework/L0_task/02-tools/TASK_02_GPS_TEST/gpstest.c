/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-01     User         GPS test command
 * 2025-07-30     User         GPS power management and test implementation
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drv_gpio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_POWER_PIN GET_PIN(B, 5)
#define GPS_UART_NAME "uart1"
#define GPS_RX_BUFFER_SIZE 512
#define GPS_BAUD_RATE BAUD_RATE_38400

#define DT_POWER_DOWN_MS 3
#define DT_ACK_HIGH_CONFIG_MS 28.0

#define CONFIG_DT_02 ((rt_uint8_t)(DT_ACK_HIGH_CONFIG_MS * 0.2) - DT_POWER_DOWN_MS)
#define CONFIG_DT_04 ((rt_uint8_t)(DT_ACK_HIGH_CONFIG_MS * 0.4) - DT_POWER_DOWN_MS)
#define CONFIG_DT_06 ((rt_uint8_t)(DT_ACK_HIGH_CONFIG_MS * 0.6) - DT_POWER_DOWN_MS)
#define CONFIG_DT_08 ((rt_uint8_t)(DT_ACK_HIGH_CONFIG_MS * 0.8) - DT_POWER_DOWN_MS)

#ifndef RT_MIN
#define RT_MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// GPS状态枚举
typedef enum {
  GPS_STATUS_DEFAULT_MODE = 0,  // 默认模式
  GPS_STATUS_HIGH_PERFORMANCE,  // 高性能模式
  GPS_STATUS_OTHER_MODE,        // 其他模式
  GPS_STATUS_BRICKED            // 变砖状态
} gps_status_t;

// GPS设备相关变量
static rt_device_t gps_uart = RT_NULL;
static rt_uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static rt_event_t rx_event = RT_NULL;
static volatile rt_bool_t data_received = RT_FALSE;

#define GPS_RX_EVENT_FLAG (1 << 0)

// UBX协议命令定义
// GPS版本查询命令 (UBX-MON-VER)
static const rt_uint8_t gps_version_cmd[] = {
    0xB5, 0x62,  // UBX同步字符
    0x0A, 0x04,  // 类别ID和消息ID (MON-VER)
    0x00, 0x00,  // 载荷长度 (0字节)
    0x0E, 0x34   // 校验和
};

// GPS电源模式查询命令 (UBX-CFG-VALGET)
static const rt_uint8_t gps_query_cmd[] = {0xB5, 0x62, 0x06, 0x8B, 0x14, 0x00, 0x00, 0x04, 0x00, 0x00,
                                           0x01, 0x00, 0xA4, 0x40, 0x03, 0x00, 0xA4, 0x40, 0x05, 0x00,
                                           0xA4, 0x40, 0x0A, 0x00, 0xA4, 0x40, 0x4C, 0x15};

// GPS高性能模式配置命令
static const rt_uint8_t gps_config_high_perf_cmd[] = {
    0xB5, 0x62, 0x06, 0x41, 0x10, 0x00, 0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E, 0x79, 0xBF, 0x28,
    0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF, 0x8F, 0x0D, 0xB5, 0x62, 0x06, 0x41, 0x1C, 0x00,
    0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34, 0xF9, 0x12, 0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4,
    0x40, 0x00, 0xB0, 0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xDE, 0xAE};
// 测试数据
static const rt_uint8_t gps_test_data[60] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
                                             0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                                             0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
                                             0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
                                             0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C};

// GPS接收指示回调函数
static void gps_rx_ind(rt_device_t dev, rt_size_t size) {
  data_received = RT_TRUE;
  rt_event_send(rx_event, GPS_RX_EVENT_FLAG);
}

// GPS设备初始化
static int gps_device_init(void) {
  struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

  // 查找GPS UART设备
  gps_uart = rt_device_find(GPS_UART_NAME);
  if (gps_uart == RT_NULL) {
    rt_kprintf("无法找到GPS设备: %s\n", GPS_UART_NAME);
    return -1;
  }

  // 配置波特率
  config.baud_rate = GPS_BAUD_RATE;
  if (rt_device_control(gps_uart, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK) {
    rt_kprintf("设置GPS UART波特率失败!\n");
    return -1;
  }

  // 创建接收事件
  if (rx_event == RT_NULL) {
    rx_event = rt_event_create("gps_rx_event", RT_IPC_FLAG_FIFO);
    if (rx_event == RT_NULL) {
      rt_kprintf("创建GPS接收事件失败!\n");
      return -1;
    }
  }

  // 设置接收回调
  rt_device_set_rx_indicate(gps_uart, gps_rx_ind);

  // 打开设备
  if (rt_device_open(gps_uart, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
    rt_kprintf("打开GPS设备失败!\n");
    return -1;
  }

  rt_kprintf("GPS设备初始化成功\n");
  return 0;
}

// GPS设备清理
static void gps_device_cleanup(void) {
  if (gps_uart) {
    rt_device_close(gps_uart);
    gps_uart = RT_NULL;
  }

  if (rx_event) {
    rt_event_delete(rx_event);
    rx_event = RT_NULL;
  }
}

// GPS命令发送函数
static int gps_send_command(const rt_uint8_t *cmd, rt_size_t len) {
  rt_uint8_t dummy_buffer[64];
  rt_size_t read_count;
  int clear_attempts = 0;

  if (gps_uart == RT_NULL) {
    rt_kprintf("GPS设备未初始化\n");
    return -1;
  }

  // 更彻底地清空接收缓冲区
  do {
    read_count = 0;
    // 多次读取确保清空
    while (rt_device_read(gps_uart, 0, dummy_buffer, sizeof(dummy_buffer)) > 0) {
      read_count++;
      if (read_count > 10) break;  // 防止死循环
    }
    clear_attempts++;
    if (clear_attempts > 3) break;  // 最多尝试3次
    rt_thread_mdelay(10);           // 短暂延迟，让数据完全到达
  } while (read_count > 0);

  // 重置事件标志
  rt_event_control(rx_event, RT_IPC_CMD_RESET, RT_NULL);

  // 再次短暂延迟确保清空完成
  rt_thread_mdelay(20);

  rt_size_t sent = rt_device_write(gps_uart, 0, cmd, len);
  if (sent != len) {
    rt_kprintf("GPS命令发送失败! 期望:%d 实际:%d\n", len, sent);
    return -1;
  }

  // rt_kprintf("GPS命令发送成功，长度: %d 字节\n", len);
  return 0;
}

// GPS响应等待函数
static int gps_wait_response(rt_uint8_t *buffer, rt_size_t max_len, rt_uint32_t timeout_ms, rt_size_t expected_len) {
  rt_uint32_t recv_set = 0;
  rt_size_t total_received = 0;
  rt_size_t read_len;
  rt_size_t target_len = (expected_len > 0) ? expected_len : max_len;

  data_received = RT_FALSE;

  // 等待接收事件，超时时间由参数指定
  if (rt_event_recv(rx_event, GPS_RX_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                    rt_tick_from_millisecond(timeout_ms), &recv_set) != RT_EOK) {
    rt_kprintf("GPS响应超时\n");
    return 0;  // 超时返回0
  }

  // 逐步读取直到达到目标长度
  while (total_received < target_len && total_received < max_len) {
    read_len = rt_device_read(gps_uart, 0, buffer + total_received,
                              RT_MIN(target_len - total_received, max_len - total_received));
    if (read_len <= 0) {
      // 如果没有更多数据，短暂等待后再试
      rt_thread_mdelay(10);
      read_len = rt_device_read(gps_uart, 0, buffer + total_received,
                                RT_MIN(target_len - total_received, max_len - total_received));
      if (read_len <= 0) {
        break;  // 确实没有更多数据
      }
    }
    total_received += read_len;

    // 如果已达到期望长度，退出
    if (expected_len > 0 && total_received >= expected_len) {
      break;
    }
  }

  return total_received;
}

// GPS基础通信检查
static rt_bool_t gps_check_basic_communication(void) {
  rt_uint8_t response[128];
  rt_size_t response_len;

  rt_kprintf("=== GPS基础通信检查 ===\n");

  // 初始化GPS设备
  if (gps_device_init() != 0) {
    rt_kprintf("GPS设备初始化失败\n");
    return RT_FALSE;
  }

  // 发送版本查询命令
  rt_kprintf("发送GPS版本查询命令...\n");
  if (gps_send_command(gps_version_cmd, sizeof(gps_version_cmd)) != 0) {
    rt_kprintf("GPS命令发送失败\n");
    gps_device_cleanup();
    return RT_FALSE;
  }

  // 等待响应 (3秒超时)
  response_len = gps_wait_response(response, sizeof(response), 3000, 0);  // 0表示读取所有可用数据

  if (response_len <= 0) {
    rt_kprintf("GPS无响应 - 通信异常\n");
    gps_device_cleanup();
    return RT_FALSE;
  }

  // 打印接收到的数据（完整输出）
  rt_kprintf("接收到GPS数据(%d字节): ", response_len);
  for (int i = 0; i < response_len; i++) {
    rt_kprintf("%02X ", response[i]);
    // 每16个字节换行，便于阅读
    if ((i + 1) % 16 == 0) {
      rt_kprintf("\n");
    }
  }
  if (response_len % 16 != 0) {
    rt_kprintf("\n");
  }

  // 验证响应格式
  if (response_len >= 8 && response[0] == 0xB5 && response[1] == 0x62) {
    rt_kprintf("GPS基础通信正常\n");
    gps_device_cleanup();
    return RT_TRUE;
  } else {
    rt_kprintf("GPS响应格式错误 - 通信异常\n");
    gps_device_cleanup();
    return RT_FALSE;
  }
}

// GPS模式检查
static gps_status_t gps_check_current_mode(void) {
  rt_uint8_t response[64];
  rt_size_t response_len;

  rt_kprintf("=== GPS模式检查 ===\n");

  // 初始化GPS设备
  if (gps_device_init() != 0) {
    rt_kprintf("GPS设备初始化失败\n");
    return GPS_STATUS_BRICKED;
  }

  // 发送电源模式查询命令
  rt_kprintf("发送GPS电源模式查询命令...\n");
  if (gps_send_command(gps_query_cmd, sizeof(gps_query_cmd)) != 0) {
    gps_device_cleanup();
    return GPS_STATUS_BRICKED;  // 发送失败认为是变砖
  }

  // 先尝试读取高性能模式的响应长度 (38字节)
  response_len = gps_wait_response(response, sizeof(response), 2000, 38);

  if (response_len <= 0) {
    rt_kprintf("GPS无响应，设备已变砖\n");
    gps_device_cleanup();
    return GPS_STATUS_BRICKED;
  }

  // 打印接收到的数据（完整输出）
  rt_kprintf("接收到模式查询响应(%d字节): \n", response_len);
  for (int i = 0; i < response_len; i++) {
    rt_kprintf("%02X ", response[i]);
    // 每16个字节换行，便于阅读
    if ((i + 1) % 16 == 0) {
      rt_kprintf("\n");
    }
  }
  if (response_len % 16 != 0) {
    rt_kprintf("\n");
  }

  // 如果收到了38字节，检查是否为高性能模式
  if (response_len == 38) {
    static const rt_uint8_t high_perf_response[] = {0xB5, 0x62, 0x06, 0x8B, 0x14, 0x00, 0x01, 0x04, 0x00, 0x00,
                                                    0x01, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0x03, 0x00,
                                                    0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0xCE, 0xF7, 0xB5, 0x62,
                                                    0x05, 0x01, 0x02, 0x00, 0x06, 0x8B, 0x99, 0xC2};

    if (memcmp(response, high_perf_response, 38) == 0) {
      rt_kprintf("GPS处于高性能模式\n");
      gps_device_cleanup();
      return GPS_STATUS_HIGH_PERFORMANCE;
    }
  }

  // 如果收到了10字节，检查是否为默认模式
  if (response_len == 10) {
    static const rt_uint8_t default_mode_response[] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x06, 0x8B, 0x98, 0xBD};

    if (memcmp(response, default_mode_response, 10) == 0) {
      rt_kprintf("GPS处于默认模式\n");
      gps_device_cleanup();
      return GPS_STATUS_DEFAULT_MODE;
    }
  }

  // 如果响应长度不是预期的，或者内容不匹配，根据UBX头部判断
  if (response_len >= 8 && response[0] == 0xB5 && response[1] == 0x62) {
    rt_kprintf("GPS处于其他模式\n");
    gps_device_cleanup();
    return GPS_STATUS_OTHER_MODE;
  } else {
    rt_kprintf("GPS响应格式错误，非UBX格式\n");
    gps_device_cleanup();
    return GPS_STATUS_OTHER_MODE;
  }
}

// GPS电源控制
static int gps_power_control(rt_bool_t enable) {
  rt_pin_mode(GPS_POWER_PIN, PIN_MODE_OUTPUT);

  if (enable) {
    rt_pin_write(GPS_POWER_PIN, PIN_HIGH);
    // rt_kprintf("GPS电源已上电\n");
  } else {
    rt_pin_write(GPS_POWER_PIN, PIN_LOW);
    // rt_kprintf("GPS电源已断开\n");
  }

  return 0;
}

// 测试函数：基础通信检查
static int gps_test_communication(void) {
  rt_bool_t result = gps_check_basic_communication();

  if (result) {
    rt_kprintf("测试结果: 通信正常\n");
    return 0;
  } else {
    rt_kprintf("测试结果: 通信异常\n");
    return -1;
  }
}

// 测试函数：模式检查
static int gps_test_mode_check(void) {
  gps_status_t status = gps_check_current_mode();

  switch (status) {
    case GPS_STATUS_DEFAULT_MODE:
      rt_kprintf("当前模式: 默认模式\n");
      break;
    case GPS_STATUS_HIGH_PERFORMANCE:
      rt_kprintf("当前模式: 高性能模式\n");
      break;
    case GPS_STATUS_OTHER_MODE:
      rt_kprintf("当前模式: 其他模式\n");
      break;
    case GPS_STATUS_BRICKED:
      rt_kprintf("当前模式: 变砖状态\n");
      break;
  }

  return 0;
}

// 测试函数：电源控制
static int gps_test_power_control(rt_bool_t enable) {
  rt_kprintf("=== GPS电源控制测试 ===\n");
  return gps_power_control(enable);
}

// 测试函数：GPS高性能模式配置
static int gps_config_high_performance(void) {
  rt_kprintf("=== GPS高性能模式配置 ===\n");

  // 初始化GPS设备
  if (gps_device_init() != 0) {
    rt_kprintf("GPS设备初始化失败\n");
    return -1;
  }

  // 发送高性能模式配置命令
  rt_kprintf("发送高性能模式配置命令...\n");
  if (gps_send_command(gps_config_high_perf_cmd, sizeof(gps_config_high_perf_cmd)) != 0) {
    rt_kprintf("高性能模式配置命令发送失败\n");
    gps_device_cleanup();
    return -1;
  }

  // 等待配置生效
  rt_thread_mdelay(1000);

  // 关闭设备
  gps_device_cleanup();

  rt_kprintf("高性能模式配置完成\n");
  return 0;
}

// 测试函数：延迟测试 (DT测试)
static int gps_config_dt(rt_uint8_t dt_value) {
  gps_power_control(RT_FALSE);  // 开gps电源
  rt_thread_mdelay(3000);
  rt_kprintf("=== GPS D%02d 延迟测试 ===\n", dt_value);

  // 初始化GPS设备
  if (gps_device_init() != 0) {
    rt_kprintf("GPS设备初始化失败\n");
    return -1;
  }

  // 发送测试数据
  rt_kprintf("发送测试数据...\n");
  if (gps_send_command(gps_config_high_perf_cmd, sizeof(gps_config_high_perf_cmd)) != 0) {
    rt_kprintf("测试数据发送失败\n");
    gps_device_cleanup();
    return -1;
  }

  // 延迟指定毫秒数
  // rt_kprintf("延迟 %d 毫秒...\n", dt_value);
  rt_thread_mdelay(dt_value);

  // 电源控制
  // rt_kprintf("执行电源下电...\n");
  gps_power_control(RT_TRUE);

  // 关闭设备
  gps_device_cleanup();

  rt_kprintf("D%02d 测试完成\n", dt_value);
  return 0;
}

// GPS命令行接口
static int cmd_gps(int argc, char **argv) {
  if (argc < 2) {
    rt_kprintf("GPS 测试命令帮助:\n");
    rt_kprintf("  gps comm      - 检查GPS基础通信\n");
    rt_kprintf("  gps mode      - 检查GPS当前模式\n");
    rt_kprintf("  gps config    - 配置GPS高性能模式\n");
    rt_kprintf("  gps power_on  - GPS电源上电\n");
    rt_kprintf("  gps power_off - GPS电源断电\n");
    rt_kprintf("  gps dt02      - D02延迟测试 (2ms)\n");
    rt_kprintf("  gps dt04      - D04延迟测试 (4ms)\n");
    rt_kprintf("  gps dt06      - D06延迟测试 (6ms)\n");
    rt_kprintf("  gps dt08      - D08延迟测试 (8ms)\n");
    return 0;
  }

  if (strcmp(argv[1], "comm") == 0) {
    gps_power_control(RT_FALSE);
    return gps_test_communication();
  } else if (strcmp(argv[1], "mode") == 0) {
    gps_power_control(RT_FALSE);
    return gps_test_mode_check();
  } else if (strcmp(argv[1], "config") == 0) {
    gps_power_control(RT_FALSE);
    return gps_config_high_performance();
  } else if (strcmp(argv[1], "power_on") == 0) {
    return gps_test_power_control(RT_TRUE);
  } else if (strcmp(argv[1], "power_off") == 0) {
    return gps_test_power_control(RT_FALSE);
  } else if (strcmp(argv[1], "dt02") == 0) {
    return gps_config_dt(CONFIG_DT_02);
  } else if (strcmp(argv[1], "dt04") == 0) {
    return gps_config_dt(CONFIG_DT_04);
  } else if (strcmp(argv[1], "dt06") == 0) {
    return gps_config_dt(CONFIG_DT_06);
  } else if (strcmp(argv[1], "dt08") == 0) {
    return gps_config_dt(CONFIG_DT_08);
  } else {
    rt_kprintf("未知命令: %s\n", argv[1]);
    rt_kprintf("使用 'gps' 查看帮助信息\n");
    return -1;
  }
}

MSH_CMD_EXPORT_ALIAS(cmd_gps, gps, GPS power management and test commands);
