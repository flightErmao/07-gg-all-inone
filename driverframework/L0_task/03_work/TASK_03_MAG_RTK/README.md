# 磁力计测试模块使用说明

## 概述

本模块提供磁力计（QMC6308/QMC6309）的测试功能，支持通过串口发送命令进行设备 ID 测试和自测功能。

## 通信协议

### 协议格式

所有命令采用统一的协议格式：

```
帧头1 | 帧头2 | 命令码 | 数据 | CRC8
0x55  | 0x55  | CMD    | DATA | CRC
```

- **帧头**: 固定为 `0x55 0x55`
- **命令码**: 1 字节，定义命令类型
- **数据**: 1 字节，命令参数（通常为 `0x00`）
- **CRC8**: 1 字节，对前 4 字节进行 CRC8 校验

### CRC8 算法

CRC8 多项式：`0x07`

```c
uint8_t crc = 0x00;
for (uint8_t i = 0; i < 4; i++) {
    crc ^= frame[i];
    for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc <<= 1;
        }
    }
}
```

## 支持的命令

### 1. ID 测试命令

**命令格式：**
```
0x55 0x55 0x01 0x00 [CRC8]
```

**完整报文（含 CRC8）：**
```
0x55 0x55 0x01 0x00 0x43
```

**十六进制格式：**
```
55 55 01 00 43
```

**功能说明：**
- 读取磁力计设备 ID 寄存器
- 与期望值进行比对
- 支持重测机制（最多 3 次，每次间隔 15ms）

**响应格式：**
- **成功**: `MAG6308 OK\r\n`
- **失败**: `MAG6308 FAIL\r\n`

**使用示例：**
```
发送: 55 55 01 00 43
接收: MAG6308 OK
```

### 2. 自测命令（新增）

**命令格式：**
```
0x55 0x55 0x02 0x00 [CRC8]
```

**完整报文（含 CRC8）：**
```
0x55 0x55 0x02 0x00 0x7C
```

**十六进制格式：**
```
55 55 02 00 7C
```

**功能说明：**
- 执行 QMC6309 磁力计自测功能
- 自动配置控制寄存器进入自测模式
- 读取自测数据并验证是否在有效范围内
- 支持重试机制（最多 3 次）

**自测流程：**
1. 初始化控制寄存器（CTL_REG_ONE = 0x00, CTL_REG_TWO = 0x00）
2. 启动自测模式（CTL_REG_ONE = 0x03）
3. 触发自测（寄存器 0x0E = 0x80）
4. 等待数据就绪（STATUS_REG DRDY 位）
5. 读取自测数据（X/Y/Z 三轴）
6. 验证数据范围：
   - X 轴: 100 < |X| < 500
   - Y 轴: 100 < |Y| < 500
   - Z 轴: 100 < |Z| < 500

**响应格式：**
- **成功**: `MAG6309 SELFTEST OK\r\n`
- **失败**: `MAG6309 SELFTEST FAIL\r\n`

**使用示例：**
```
发送: 55 55 02 00 7C
接收: MAG6309 SELFTEST OK
```

## CRC8 计算示例

### Python 示例

```python
def calc_crc8(data):
    """计算 CRC8 校验值"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

# ID 测试命令
cmd_id_test = [0x55, 0x55, 0x01, 0x00]
crc = calc_crc8(cmd_id_test)
cmd_id_test.append(crc)
print(f"ID 测试命令: {[hex(x) for x in cmd_id_test]}")

# 自测命令
cmd_selftest = [0x55, 0x55, 0x02, 0x00]
crc = calc_crc8(cmd_selftest)
cmd_selftest.append(crc)
print(f"自测命令: {[hex(x) for x in cmd_selftest]}")
```

### C 示例

```c
uint8_t calc_crc8(const uint8_t* buf, uint8_t len) {
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

// ID 测试命令
uint8_t cmd_id_test[5] = {0x55, 0x55, 0x01, 0x00, 0x00};
cmd_id_test[4] = calc_crc8(cmd_id_test, 4);

// 自测命令
uint8_t cmd_selftest[5] = {0x55, 0x55, 0x02, 0x00, 0x00};
cmd_selftest[4] = calc_crc8(cmd_selftest, 4);
```

## 配置说明

### Kconfig 配置项

模块通过以下配置项控制：

- `WORK_TASK_MAG_TEST_6308_EN`: 使能磁力计测试模块
- `WORK_TASK_MAG_TEST_6308_I2C_NAME`: I2C 总线设备名（默认: "hwi2c1"）
- `WORK_TASK_MAG_TEST_6308_I2C_ADDR`: I2C 设备地址（默认: 0x2C）
- `WORK_TASK_MAG_TEST_6308_ID_REG`: ID 寄存器地址（默认: 0x00）
- `WORK_TASK_MAG_TEST_6308_ID_EXPECT`: 期望的 ID 值（默认: 0x80）
- `WORK_TASK_MAG_TEST_6308_UART_NAME`: UART 设备名（默认: "uart1"）
- `WORK_TASK_MAG_TEST_6308_UART_BAUD`: UART 波特率（默认: 115200）

### 自测参数配置

自测功能的阈值定义在 `qmc6309_selftest.h` 中：

```c
#define QMC6309_SELFTEST_MIN_X     100
#define QMC6309_SELFTEST_MAX_X     500
#define QMC6309_SELFTEST_MIN_Y     100
#define QMC6309_SELFTEST_MAX_Y     500
#define QMC6309_SELFTEST_MIN_Z     100
#define QMC6309_SELFTEST_MAX_Z     500
```

可根据实际硬件特性调整这些阈值。

## 错误处理

### 常见错误

1. **CRC 校验失败**
   - 原因：数据传输错误
   - 处理：重新发送命令

2. **命令未响应**
   - 原因：串口配置错误或设备未初始化
   - 处理：检查串口配置和设备初始化状态

3. **自测失败**
   - 原因：硬件故障或数据超出范围
   - 处理：检查硬件连接和自测阈值配置

### 重试机制

- **ID 测试**: 自动重试 3 次，每次间隔 15ms
- **自测功能**: 自动重试 3 次，每次完整执行自测流程

## 文件结构

```
TASK_03_MAG_RTK/
├── magTest.c              # 主测试模块
├── qmc6309_selftest.c     # 自测功能实现
├── qmc6309_selftest.h     # 自测功能头文件
├── SConscript             # 编译脚本
├── Kconfig                # 配置选项
└── README.md              # 本文档
```

## 使用流程

### 上位机测试流程

1. **配置串口**
   - 打开串口，波特率：115200（或配置值）
   - 数据位：8，停止位：1，校验位：无

2. **发送 ID 测试命令**
   ```
   发送: 55 55 01 00 43
   等待: MAG6308 OK 或 MAG6308 FAIL
   ```

3. **发送自测命令**
   ```
   发送: 55 55 02 00 7C
   等待: MAG6309 SELFTEST OK 或 MAG6309 SELFTEST FAIL
   ```

### 测试脚本示例

```python
import serial
import time

def calc_crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def send_command(ser, cmd_bytes):
    """发送命令并等待响应"""
    crc = calc_crc8(cmd_bytes)
    cmd = bytes(cmd_bytes + [crc])
    ser.write(cmd)
    time.sleep(0.1)
    response = ser.read_all().decode('utf-8', errors='ignore')
    return response.strip()

# 打开串口
ser = serial.Serial('COM3', 115200, timeout=1)

# ID 测试
print("发送 ID 测试命令...")
response = send_command(ser, [0x55, 0x55, 0x01, 0x00])  # CRC8: 0x43
print(f"响应: {response}")

# 自测
print("发送自测命令...")
response = send_command(ser, [0x55, 0x55, 0x02, 0x00])  # CRC8: 0x7C
print(f"响应: {response}")

ser.close()
```

## 注意事项

1. **命令间隔**: 建议两次命令之间间隔至少 100ms，确保前一个命令处理完成
2. **CRC 校验**: 所有命令必须包含正确的 CRC8 校验值，否则会被忽略
3. **响应格式**: 响应以 `\r\n` 结尾，解析时需要注意
4. **超时处理**: 建议设置合理的串口读取超时时间（建议 1-2 秒）
5. **硬件要求**: 确保 I2C 和 UART 硬件连接正确，设备已上电

## 版本历史

- **v1.0** (2024): 初始版本，支持 ID 测试功能
- **v1.1** (2024): 新增自测功能支持

## 技术支持

如有问题，请检查：
1. 串口和 I2C 配置是否正确
2. 硬件连接是否正常
3. 设备地址和寄存器配置是否匹配
4. 自测阈值是否适合当前硬件

