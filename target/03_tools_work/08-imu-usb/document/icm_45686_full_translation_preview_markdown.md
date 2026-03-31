# ICM-45686 数据手册（中文翻译预览）

> 原文：ICM-45686 Datasheet v1.1
> 文档号：DS-000489
> 修订日期：2023-11-25

---

# 1. 简介

## 1.1 目的与范围

本文档为 ICM-45686 的产品规格说明书，描述其功能、性能、接口、寄存器与设计相关信息。

ICM-45686 是一款高性能双接口（UI + AUX）6 轴 MEMS MotionTracking 设备，采用 2.5 × 3.0 × 0.81 mm 的 14 引脚 LGA 封装。

---

## 1.2 产品概述

ICM-45686 集成：

- 三轴陀螺仪
- 三轴加速度计
- 温度传感器
- FIFO 缓冲区
- 可编程中断
- APEX 运动检测引擎

芯片具有两套独立的数据路径：

- UI Path（主接口路径）
- AUX1 Path（辅助接口路径）

两条路径均可独立设置：

- 满量程（FSR）
- 输出数据率（ODR）

### 主接口支持

- I3C
- I2C
- SPI

### AUX 接口支持

- SPI Slave（用于 OIS 控制器）
- I2C Master（用于外接传感器）

芯片内置最高 8 KB FIFO（默认 2 KB，可关闭 APEX 后扩展到 8 KB），用于降低主控读取频率并降低系统功耗。

---

## 1.3 主要特性

### 陀螺仪

- 噪声：3.8 mdps/√Hz
- 量程：
  - ±15.625 dps
  - ±31.25 dps
  - ±62.5 dps
  - ±125 dps
  - ±250 dps
  - ±500 dps
  - ±1000 dps
  - ±2000 dps
  - ±4000 dps

### 加速度计

- 噪声：70 µg/√Hz
- 量程：
  - ±2 g
  - ±4 g
  - ±8 g
  - ±16 g
  - ±32 g

### FIFO

- 默认容量：2 KB
- 最大容量：8 KB
- 支持 16-bit 与 20-bit FIFO 数据格式
- 支持独立 FIFO Data Rate (FDR)

### 接口速度

- SPI：最高 24 MHz
- I2C：最高 1 MHz
- I3C：最高 12.9 MHz

---

# 2. FIFO

## 2.1 FIFO 特点

ICM-45686 的 FIFO 与 ICM-42688 相比更复杂，支持：

- 主路径 UI 数据
- AUX1 路径数据
- 16-bit 模式
- 20-bit 高精度模式
- 时间戳
- 温度
- FSYNC
- Metadata

FIFO 数据以 Packet 的形式存储。

每个 Packet 都以 1 字节 Header 开头。

Header 用于描述本包中包含哪些字段。

---

## 2.2 推荐固定长度 FIFO 配置

若希望每个 FIFO 包都固定包含：

- Accel XYZ
- Gyro XYZ
- Temperature
- Timestamp

推荐关闭：

- 20-bit 模式
- AUX1 数据
- FSYNC
- Metadata

此时每包固定长度为 16 字节：

```text
Byte0      Header
Byte1-2    Accel X
Byte3-4    Accel Y
Byte5-6    Accel Z
Byte7-8    Gyro X
Byte9-10   Gyro Y
Byte11-12  Gyro Z
Byte13     Temperature
Byte14-15  Timestamp
```

总长度：

```text
16 bytes
```

对应配置：

```c
FIFO_CONFIG0      = 0x80;
FIFO_CONFIG1_0    = 0x07;
FIFO_CONFIG1_1    = 0x02;
FIFO_CONFIG2      = 0x00;
FIFO_CONFIG3      = 0x00;
FIFO_CONFIG4      = 0x00;
```

---

## 2.3 FIFO Count

FIFO_COUNT_1 与 FIFO_COUNT_0 构成 16 位计数值。

其含义为：

> 当前 FIFO 中可读取的总字节数

例如：

```text
FIFO_COUNT = 0x0030
```

表示 FIFO 中有 48 字节可读。

如果 packet 固定为 16 字节，则：

```c
packet_count = fifo_count / 16;
```

---

## 2.4 FIFO 读取方式

FIFO_DATA 是一个单字节寄存器。

读取流程：

1. 读取 FIFO_COUNT_1
2. 读取 FIFO_COUNT_0
3. 合成为总字节数
4. 对 FIFO_DATA 进行 burst read

例如：

```c
uint16_t fifo_count;
fifo_count = (read_reg(FIFO_COUNT_1) << 8) |
             read_reg(FIFO_COUNT_0);

spi_read_burst(FIFO_DATA, buf, fifo_count);
```

---

# 3. 温度与时间戳

## 3.1 FIFO 温度格式

FIFO 中的温度仅占 1 字节。

其转换公式为：

```c
Temp(°C) = 25 + temp_raw / 2
```

例如：

```text
temp_raw = 10
```

则：

```text
温度 = 25 + 10/2 = 30°C
```

---

## 3.2 FIFO 时间戳格式

FIFO 时间戳为 16 位。

其范围：

```text
0 ~ 65535
```

随后循环。

默认情况下，时间戳每 tick 对应约 16 µs。

因此：

```c
time_us = timestamp * 16;
```

如果 ODR = 1000 Hz，则相邻两包时间戳大约增加：

```text
1000 µs / 16 µs ≈ 62.5
```

因此你会看到 timestamp 差约为 62 或 63，而不是 1。

---

# 4. 寄存器摘要

| 寄存器 | 功能 |
|---|---|
| FIFO_COUNT_0 | FIFO 字节数低字节 |
| FIFO_COUNT_1 | FIFO 字节数高字节 |
| FIFO_DATA | FIFO 数据读取口 |
| FIFO_CONFIG0 | FIFO 总开关 |
| FIFO_CONFIG1_0 | 选择 accel / gyro / temp |
| FIFO_CONFIG1_1 | 时间戳与额外字段 |
| FIFO_CONFIG2 | 20-bit 与高精度配置 |
| FIFO_CONFIG3 | AUX1 数据配置 |
| FIFO_CONFIG4 | FSYNC / Metadata 配置 |

---

# 后续翻译计划

完整文档预计包含：

1. 电气特性
2. SPI / I2C / I3C 时序
3. FIFO 与 Header 全部位定义
4. 所有寄存器逐项翻译
5. APEX 功能
6. AUX1 与 OIS 接口
7. 20-bit FIFO 格式
8. 中断系统
9. 推荐初始化代码
10. 与 ICM-42688 的差异对比

