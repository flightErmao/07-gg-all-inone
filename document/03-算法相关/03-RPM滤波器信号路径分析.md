# Betaflight RPM滤波器信号路径分析文档

## 目录
- [一、RPM滤波器概述](#一rpm滤波器概述)
- [二、DShot遥测接收与解码](#二dshot遥测接收与解码)
- [三、eRPM到频率的转换](#三erpm到频率的转换)
- [四、RPM滤波器初始化](#四rpm滤波器初始化)
- [五、RPM滤波器实时更新](#五rpm滤波器实时更新)
- [六、陀螺仪滤波应用](#六陀螺仪滤波应用)
- [七、完整信号流程图](#七完整信号流程图)
- [八、代码文件索引](#八代码文件索引)

---

## 一、RPM滤波器概述

### 1.1 什么是RPM滤波器？

RPM滤波器（RPM Filter）是Betaflight中最精确的噪声消除技术，它基于**电机的实时转速**创建动态陷波滤波器，精确消除电机和桨叶产生的振动噪声。

### 1.2 工作原理

```
电调(ESC) → 双向DShot遥测 → eRPM数据 → 转换为频率 → 计算谐波 → 
创建陷波滤波器 → 应用于陀螺仪数据 → 消除电机振动噪声
```

### 1.3 为什么RPM滤波器效果最好？

| 特性 | RPM滤波器 | 动态陷波 | 静态陷波 |
|-----|----------|---------|---------|
| **数据源** | 电机转速（直接） | FFT分析（间接） | 手动配置 |
| **精度** | 极高 | 高 | 中等 |
| **跟踪速度** | 实时 | 稍有延迟 | 固定频率 |
| **频率范围** | 全范围（悬停-全油门） | 受FFT限制 | 固定 |
| **CPU开销** | 低 | 中高（FFT） | 很低 |
| **配置** | 自动 | 自动 | 需要手动 |

### 1.4 前提条件

要使用RPM滤波器，必须满足：
1. **硬件支持**：电调（ESC）必须支持双向DShot协议
2. **协议配置**：使用DShot150、DShot300或DShot600
3. **固件支持**：电调固件支持遥测反馈（如BLHeli_32、BlueJay等）
4. **飞控配置**：在Betaflight中启用双向DShot（`set dshot_bidir = ON`）

### 1.5 关键概念

#### eRPM (Electrical RPM)
- **定义**：电机的电气转速，即电机磁极的旋转速度
- **关系**：`实际RPM = eRPM / (磁极对数)`
- **例如**：14极电机（7对磁极），eRPM = 7000，则实际RPM = 1000

#### 谐波 (Harmonics)
- **基频**：电机旋转产生的基本频率 = 实际转速 × (叶片数/2)
- **谐波**：基频的整数倍（2倍、3倍等）
- **原因**：桨叶产生的气动扰动不是完美正弦波，包含多个频率成分

---

## 二、DShot遥测接收与解码

### 2.1 双向DShot协议概述

#### 2.1.1 传统DShot vs 双向DShot

**传统DShot（单向）**：
```
飞控 ─────DShot指令────→ 电调
```

**双向DShot（Bidirectional）**：
```
飞控 ─────DShot指令────→ 电调
飞控 ←────遥测数据─────  电调
```

#### 2.1.2 双向DShot时序

```
时间线（以DShot600为例，位周期 = 1.67μs）
──────────────────────────────────────────────────────────→

├─ 飞控发送DShot指令 (16位 = 26.7μs)
│  └─ 包含油门值 + 遥测请求位 + 校验和
│
├─ 等待30μs（电调处理时间）
│
├─ 电调发送遥测数据 (21位 @ 5/4x速度 = 28μs)
│  └─ GCR编码的eRPM数据
│
└─ 飞控准备接收下一帧（总周期约100μs）
```

### 2.2 遥测数据格式

#### 2.2.1 原始数据格式（16位）

```
位15-12: eee (指数部分，3位)
位11-3:  mmmmmmmmm (尾数部分，9位)
位2-0:   校验和的补码
```

**eRPM计算公式**：
```
周期(μs) = 尾数 << 指数
函数返回值 = (60 × 1,000,000 / 100) / 周期 = 600,000 / 周期
真实eRPM = 函数返回值 × 100 = 60,000,000 / 周期

注意：dshot_decode_eRPM_telemetry_value() 返回的是 eRPM/100（以100为单位）
```

#### 2.2.2 GCR编码

为了确保信号可靠性，16位数据被GCR（Group Code Recording）编码为20位：

**GCR编码表**：
```
十六进制 → GCR编码
0 → 0x19    1 → 0x1B    2 → 0x12    3 → 0x13
4 → 0x1D    5 → 0x15    6 → 0x16    7 → 0x17
8 → 0x1A    9 → 0x09    A → 0x0A    B → 0x0B
C → 0x1E    D → 0x0D    E → 0x0E    F → 0x0F
```

**特点**：GCR编码确保最多只有2个连续的0，提高解码可靠性。

#### 2.2.3 差分曼彻斯特编码

20位GCR数据再编码为21位差分信号：
```
原始位: 1 0 1 1 1 0 0 1 1 0
编码后: 0 1 1 0 1 0 0 0 1 0 0
规则：
  - 起始位总是0
  - 如果当前位是1，则翻转输出
  - 如果当前位是0，则保持输出
```

### 2.3 硬件层接收

#### 2.3.1 硬件接收机制

根据不同的MCU平台，有不同的实现方式：

**STM32平台** (使用DMA + 定时器):
- **代码位置**：`src/platform/STM32/dshot_bitbang.c`
- **机制**：
  1. 发送DShot指令后，GPIO切换为输入模式
  2. 使用定时器捕获引脚电平变化
  3. DMA自动将捕获数据存储到缓冲区
  4. 中断触发后进行解码

**RP2040 (Pico)平台** (使用PIO):
- **代码位置**：`src/platform/PICO/dshot_bidir_pico.c`、`src/platform/PICO/dshot.pio`
- **机制**：
  1. PIO状态机自动处理发送和接收
  2. 3倍过采样确保可靠性
  3. 数据自动存入FIFO
  4. CPU从FIFO读取并解码

#### 2.3.2 PIO接收实现（RP2040示例）

**PIO程序** (`src/platform/PICO/dshot.pio` 第97-119行):
```pio
receive:
    set    pindirs, 0     side 0        ; 切换为输入模式
    set    x, 19          side 0        ; 等待30μs
delay_loop:
    nop                   side 0  [14]
    jmp    x--, delay_loop side 0 [14]
    
    wait   0 pin, 0       side 1  [2]   ; 等待起始位（0）
    nop                   side 0  [15]
    set    x, 1           side 0  [15]  ; 接收2个字（每个30位）
    
in_wordloop:
    set    y, 9           side 0  [1]   ; 每个字10组×3位采样
in_bitloop:
    in     pins, 1        side 1  [9]   ; 第1次采样
    in     pins, 1        side 0  [9]   ; 第2次采样
    in     pins, 1        side 1  [5]   ; 第3次采样
    jmp    y--, in_bitloop side 0
    
    push   noblock        side 0        ; 推送到RX FIFO
    jmp    x--, in_wordloop side 0      ; 接收第2个字
```

**要点**：
- **3倍过采样**：每个位采样3次，使用多数表决来确定真实值
- **2个字**：21位数据被分成2个30位字（包括填充位）
- **自动化**：PIO硬件自动完成，不占用CPU

### 2.4 遥测数据解码

#### 2.4.1 解码函数调用链

```
硬件层（中断/DMA/PIO）
    ↓
dshotDecodeTelemetry()        ← 从硬件FIFO读取原始数据
    ↓
decodeTelemetryRaw()          ← GCR和差分曼彻斯特解码
    ↓
dshotTelemetryState.motorState[motorIndex].rawValue  ← 存储原始16位值
    ↓
updateDshotTelemetry()        ← 主循环调用
    ↓
dshot_decode_telemetry_value() ← 解析遥测类型
    ↓
dshot_decode_eRPM_telemetry_value() ← 解码eRPM值
```

#### 2.4.2 详细解码过程

##### 第1步：GCR解码

**代码位置**：`src/platform/PICO/dshot_bidir_pico.c` 第148-296行

```c
static uint32_t decodeTelemetryRaw(int motorIndex, const uint32_t first, const uint32_t second)
{
    // 3倍采样的多数表决表
    const uint8_t majorityVerdict[] = {0, 1, 1, 1, 1, 1, 1, 0};
    // 000 -> 0, 001-110 -> 1, 111 -> 0
    
    // GCR反向查找表
    const int8_t gcrs[] = {
        -1, -1, -1, -1, -1, -1, -1, -1,
        -1,  9, 10, 11, -1, 13, 14, 15,
        -1, -1,  2,  3, -1,  5,  6,  7,
        -1,  0,  8,  1, -1,  4, 12, -1
    };
    
    // 合并两个32位字为60位数据（20×3采样）
    uint64_t bits60 = first >> 2 | (((uint64_t)second >> 2) << 30);
    
    // 多数表决：3个采样决定1个位
    uint32_t bits21 = 0;
    for (int i = 0; i < 20; i++) {
        bits21 <<= 1;
        int pattern = bits60 & 7;  // 取3位
        bits21 += majorityVerdict[pattern] ? 1 : 0;
        bits60 >>= 3;
    }
    
    // 差分曼彻斯特解码：XOR相邻位
    bits21 ^= bits21 >> 1;
    
    // GCR解码：20位 -> 16位
    uint16_t value = 0;
    for (int i = 0; i < 4; i++) {
        int gcrNibble = (bits21 >> (i * 5)) & 0x1f;
        int8_t decoded = gcrs[gcrNibble];
        if (decoded < 0) {
            // GCR解码失败
            return DSHOT_TELEMETRY_INVALID;
        }
        value |= decoded << (i * 4);
    }
    
    // 验证校验和
    uint16_t csum = value;
    csum = csum ^ (csum >> 4);
    csum = csum ^ (csum >> 8);
    if ((csum & 0xf) != 0xf) {
        // 校验和错误
        return DSHOT_TELEMETRY_INVALID;
    }
    
    return value >> 4;  // 去掉校验和，返回12位数据
}
```

##### 第2步：eRPM值解码

**代码位置**：`src/main/drivers/dshot.c` 第198-213行

```c
static uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value)
{
    // 特殊值：0x0FFF表示电机停止
    if (value == 0x0fff) {
        return 0;
    }
    
    // 格式：eee mmmmmmmmm (3位指数 + 9位尾数)
    // 提取指数和尾数
    uint16_t exponent = (value & 0xfe00) >> 9;  // 高7位中的低3位
    uint16_t mantissa = (value & 0x01ff);       // 低9位
    
    // 计算周期（微秒）= 尾数 << 指数
    value = mantissa << exponent;
    
    if (!value) {
        return DSHOT_TELEMETRY_INVALID;
    }
    
    // 转换为 eRPM / 100（以100为单位的eRPM）
    // 真实eRPM = 返回值 × 100 = 60,000,000 / 周期(μs)
    return (1000000 * 60 / 100 + value / 2) / value;
}
```

**重要说明**：
此函数返回的是 **eRPM / 100**，而不是 eRPM × 100！

**公式推导**：
```
电气旋转频率(Hz) = 1 / 周期(s)
                 = 1,000,000 / 周期(μs)

eRPM = 电气旋转频率 × 60
     = (1,000,000 / 周期_μs) × 60
     = 60,000,000 / 周期_μs

函数返回值 = eRPM / 100
          = 60,000,000 / (周期_μs × 100)
          = 600,000 / 周期_μs

真实eRPM = 返回值 × ERPM_PER_LSB
        = 返回值 × 100

原因：为了节省传输带宽和存储空间，使用较小的数值范围（0-4095）
     实际 eRPM 可达 0-409,500，满足使用需求
```

##### 第3步：遥测数据更新

**代码位置**：`src/main/drivers/dshot.c` 第256-303行

```c
FAST_CODE_NOINLINE void updateDshotTelemetry(void)
{
    if (!useDshotTelemetry) {
        return;
    }
    
    // 检查是否有新数据需要处理
    if (dshotTelemetryState.rawValueState != DSHOT_RAW_VALUE_STATE_NOT_PROCESSED) {
        return;
    }
    
    const unsigned motorCount = MIN(MAX_SUPPORTED_MOTORS, dshotMotorCount);
    uint32_t erpmTotal = 0;
    uint32_t rpmSamples = 0;
    
    // 解码所有电机的遥测数据
    for (uint8_t k = 0; k < motorCount; k++) {
        dshotTelemetryType_t type;
        uint32_t value;
        
        // 解码遥测值
        dshot_decode_telemetry_value(k, &value, &type);
        
        if (value != DSHOT_TELEMETRY_INVALID) {
            // 更新遥测数据存储
            dshotUpdateTelemetryData(k, type, value);
            
            if (type == DSHOT_TELEMETRY_TYPE_eRPM) {
                // 转换eRPM为实际RPM
                dshotRpm[k] = erpmToRpm(value);
                erpmTotal += value;
                rpmSamples++;
            }
        }
    }
    
    // 计算平均RPM
    if (rpmSamples > 0) {
        dshotRpmAverage = erpmToRpm(erpmTotal) / (float)rpmSamples;
    }
    
    // 更新滤波后的电机频率（用于RPM滤波器）
    minMotorFrequencyHz = FLT_MAX;
    for (unsigned motor = 0; motor < dshotMotorCount; motor++) {
        // 应用PT1低通滤波平滑RPM数据
        motorFrequencyHz[motor] = pt1FilterApply(
            &motorFreqLpf[motor], 
            erpmToHz * getDshotErpm(motor)
        );
        minMotorFrequencyHz = MIN(minMotorFrequencyHz, motorFrequencyHz[motor]);
    }
    
    // 标记为已处理
    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_PROCESSED;
}
```

---

## 三、eRPM到频率的转换

### 3.1 转换公式

#### 3.1.1 eRPM到实际RPM

**代码位置**：`src/main/drivers/dshot.c` 第381-385行

```c
float erpmToRpm(uint32_t erpm)
{
    // rpm = (erpm × ERPM_PER_LSB) / (磁极对数)
    return erpm * erpmToHz * SECONDS_PER_MINUTE;
}
```

**其中**：
- `ERPM_PER_LSB = 100`（eRPM的精度因子）
- `erpmToHz`：在初始化时计算

#### 3.1.2 eRPM到频率（Hz）

**初始化代码位置**：`src/main/drivers/dshot.c` 第175-196行

```c
void initDshotTelemetry(const timeUs_t looptimeUs)
{
    // 计算eRPM到频率的转换系数
    // erpmToHz = (ERPM_PER_LSB / 60) / (磁极对数)
    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.0f);
    
#ifdef USE_RPM_FILTER
    if (motorConfig()->dev.useDshotTelemetry) {
        // 为每个电机初始化PT1滤波器
        for (unsigned i = 0; i < dshotMotorCount; i++) {
            pt1FilterInit(&motorFreqLpf[i], 
                         pt1FilterGain(rpmFilterConfig()->rpm_filter_lpf_hz, 
                                      looptimeUs * 1e-6f));
        }
    }
#endif
}
```

**公式详解**：
```
已知：
- dshot_decode_eRPM_telemetry_value() 返回值（这是 eRPM/100）
- 电机磁极数 (配置参数，如14极)

计算：
磁极对数 = 磁极数 / 2 = 14 / 2 = 7

真实eRPM = 返回值 × ERPM_PER_LSB
         = 返回值 × 100

电气旋转频率(Hz) = 真实eRPM / 60
                 = (返回值 × 100) / 60

实际转速(RPM) = 真实eRPM / 磁极对数
              = (返回值 × 100) / 7

实际旋转频率(Hz) = 实际转速 / 60
                 = (返回值 × 100) / (7 × 60)
```

**erpmToHz 计算**：
```c
// 代码第183行
erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.0f);
         = 100 / 60 / 磁极对数
```

**转换系数示例**：
```
14极电机：
磁极对数 = 7
erpmToHz = 100 / 60 / 7 = 0.238

如果 dshot_decode_eRPM_telemetry_value() 返回值 = 700:
- 真实eRPM = 700 × 100 = 70,000
- 实际RPM = 70,000 / 7 = 10,000
- 旋转频率 = 10,000 / 60 = 166.7 Hz

通过 erpmToRpm() 计算：
- erpmToRpm(700) = 700 × 0.238 × 60 = 10,000 RPM ✓
通过频率计算：
- motorFrequencyHz = 700 × 0.238 = 166.7 Hz ✓
```

### 3.2 频率平滑滤波

#### 3.2.1 为什么需要平滑？

eRPM数据可能存在：
- **量化误差**：GCR编码的精度限制
- **测量噪声**：电调测量的不确定性
- **更新率抖动**：遥测数据包的不规则到达

#### 3.2.2 PT1低通滤波器

**代码位置**：`src/main/drivers/dshot.c` 第296-299行

```c
for (unsigned motor = 0; motor < dshotMotorCount; motor++) {
    motorFrequencyHz[motor] = pt1FilterApply(
        &motorFreqLpf[motor], 
        erpmToHz * getDshotErpm(motor)
    );
}
```

**PT1滤波器原理**：
```c
// 一阶低通滤波器（指数移动平均）
y[n] = y[n-1] + k × (x[n] - y[n-1])

其中：
- x[n]: 当前输入（原始频率）
- y[n-1]: 上一次输出（滤波后频率）
- k: 滤波增益（0-1之间）
```

**增益计算**：
```c
k = dT / (dT + 1/(2πfc))

其中：
- dT: 采样周期
- fc: 截止频率（典型值：150Hz）
```

**效果**：
- **截止频率150Hz**：允许快速的转速变化通过
- **衰减高频噪声**：平滑量化和测量误差
- **延迟很小**：约1-2ms，不影响跟踪性能

### 3.3 桨叶频率计算

电机旋转产生的振动频率不是转速本身，而是**桨叶通过频率**：

```
桨叶频率 = 旋转频率 × (叶片数 / 2)
```

**原因**：
- 4叶桨：每转产生2个完整的压力波（对称结构）
- 3叶桨：每转产生1.5个压力波（但谐波复杂）

**在RPM滤波器中**：
- 不需要显式乘以叶片数
- 陷波滤波器直接跟踪电机频率及其谐波
- 谐波自然包含了桨叶效应

---

## 四、RPM滤波器初始化

### 4.1 初始化函数

**代码位置**：`src/main/flight/rpm_filter.c` 第69-108行

```c
void rpmFilterInit(const rpmFilterConfig_t *config, const timeUs_t looptimeUs)
{
    motorIndex = 0;
    harmonicIndex = 0;
    rpmFilter.numHarmonics = 0; // 默认禁用RPM滤波
    
    // 检查前提条件1：双向DShot是否可用
    if (!useDshotTelemetry) {
        return;
    }
    
    // 检查前提条件2：RPM滤波是否启用
    if (!config->rpm_filter_harmonics) {
        return;
    }
    
    // 启用并初始化RPM滤波器
    rpmFilter.numHarmonics = config->rpm_filter_harmonics;     // 谐波数量(1-3)
    rpmFilter.minHz = config->rpm_filter_min_hz;               // 最小频率(100Hz)
    rpmFilter.maxHz = 0.48f * 1e6f / looptimeUs;              // 最大频率(接近奈奎斯特)
    rpmFilter.fadeRangeHz = config->rpm_filter_fade_range_hz;  // 淡出范围(50Hz)
    rpmFilter.q = config->rpm_filter_q / 100.0f;              // Q值(5.0)
    rpmFilter.looptimeUs = looptimeUs;
    
    // 加载每个谐波的权重
    for (int n = 0; n < RPM_FILTER_HARMONICS_MAX; n++) {
        rpmFilter.weights[n] = constrainf(config->rpm_filter_weights[n] / 100.0f, 0.0f, 1.0f);
    }
    
    // 为每个轴、每个电机、每个谐波创建陷波滤波器
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < rpmFilter.numHarmonics; i++) {
                // 初始化为最小频率（稍后会更新）
                biquadFilterInit(&rpmFilter.notch[axis][motor][i], 
                                rpmFilter.minHz * i,     // 初始频率
                                rpmFilter.looptimeUs,     // 循环时间
                                rpmFilter.q,              // Q值
                                FILTER_NOTCH,             // 陷波类型
                                0.0f);                    // 初始权重0
            }
        }
    }
    
    // 计算每次PID循环需要更新多少个陷波滤波器
    const float loopIterationsPerUpdate = RPM_FILTER_DURATION_S / (looptimeUs * 1e-6f);
    const float numNotchesPerAxis = getMotorCount() * rpmFilter.numHarmonics;
    notchUpdatesPerIteration = ceilf(numNotchesPerAxis / loopIterationsPerUpdate);
}
```

### 4.2 配置参数详解

#### 4.2.1 参数组定义

**代码位置**：`src/main/pg/rpm_filter.h` 第29-39行

```c
typedef struct rpmFilterConfig_s
{
    uint8_t  rpm_filter_harmonics;      // 谐波数量：0=关闭, 1-3
    uint8_t  rpm_filter_weights[3];     // 每个谐波的权重：0-100%
    uint8_t  rpm_filter_min_hz;         // 最小频率：100Hz
    uint16_t rpm_filter_fade_range_hz;  // 淡出范围：50Hz
    uint16_t rpm_filter_q;              // Q值：500 (实际5.0)
    uint16_t rpm_filter_lpf_hz;         // eRPM数据LPF：150Hz
} rpmFilterConfig_t;
```

#### 4.2.2 默认配置

**代码位置**：`src/main/pg/rpm_filter.c` 第32-39行

```c
PG_RESET_TEMPLATE(rpmFilterConfig_t, rpmFilterConfig,
    .rpm_filter_harmonics = 3,              // 3个谐波
    .rpm_filter_min_hz = 100,               // 最小100Hz
    .rpm_filter_fade_range_hz = 50,         // 50Hz淡出范围
    .rpm_filter_q = 500,                    // Q=5.0
    .rpm_filter_lpf_hz = 150,               // 150Hz LPF
    .rpm_filter_weights = { 100, 100, 100 }, // 全部100%
);
```

#### 4.2.3 参数说明

| 参数 | 默认值 | 说明 | 影响 |
|-----|-------|------|-----|
| **rpm_filter_harmonics** | 3 | 跟踪的谐波数量 | 越多越全面，但CPU开销越大 |
| **rpm_filter_min_hz** | 100 | 最低跟踪频率 | 低于此频率陷波会淡出 |
| **rpm_filter_max_hz** | 自动计算 | 最高跟踪频率 | 接近奈奎斯特频率（~48% PID频率） |
| **rpm_filter_fade_range_hz** | 50 | 淡出过渡范围 | 避免在minHz附近突变 |
| **rpm_filter_q** | 500 (5.0) | 陷波Q值 | 越高陷波越窄，越精确 |
| **rpm_filter_lpf_hz** | 150 | eRPM数据LPF截止频率 | 平滑RPM数据，减少抖动 |
| **rpm_filter_weights** | [100,100,100] | 每个谐波的强度 | 可以单独调整每个谐波的效果 |

### 4.3 滤波器数据结构

#### 4.3.1 RPM滤波器结构

**代码位置**：`src/main/flight/rpm_filter.c` 第47-59行

```c
typedef struct rpmFilter_s {
    int numHarmonics;                      // 谐波数量(1-3)
    float weights[RPM_FILTER_HARMONICS_MAX]; // 每个谐波的权重
    float minHz;                           // 最小频率
    float maxHz;                           // 最大频率
    float fadeRangeHz;                     // 淡出范围
    float q;                               // Q值
    
    timeUs_t looptimeUs;                   // PID循环时间
    
    // 3D陷波滤波器数组：
    // [3轴][最多8电机][最多3谐波]
    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_HARMONICS_MAX];
} rpmFilter_t;
```

#### 4.3.2 滤波器数量计算

**示例：4电机、3谐波、3轴**

```
每个电机：3个谐波 = 3个陷波滤波器
总陷波数量 = 4电机 × 3谐波 = 12个陷波/轴
全部3轴 = 12 × 3 = 36个陷波滤波器

每个陷波滤波器包含：
- 5个浮点系数（b0, b1, b2, a1, a2）
- 2个状态变量（x1, x2 或 y1, y2）
- 1个权重值
总计：约32字节/滤波器 × 36 = 1152字节
```

### 4.4 批量更新策略

#### 4.4.1 为什么需要批量更新？

**问题**：如果每个PID循环更新所有陷波滤波器：
- 4电机 × 3谐波 × 3轴 = 36次Biquad系数更新
- 每次更新涉及三角函数计算（sin, cos）
- 在8kHz PID频率下，CPU负担过重

**解决方案**：分散更新
- 每个PID循环只更新部分陷波滤波器
- 在1ms内完成所有陷波的一轮更新

#### 4.4.2 更新速率计算

**代码位置**：`src/main/flight/rpm_filter.c` 第105-107行

```c
const float loopIterationsPerUpdate = RPM_FILTER_DURATION_S / (looptimeUs * 1e-6f);
const float numNotchesPerAxis = getMotorCount() * rpmFilter.numHarmonics;
notchUpdatesPerIteration = ceilf(numNotchesPerAxis / loopIterationsPerUpdate);
```

**示例计算**（8kHz PID，4电机，3谐波）：
```
RPM_FILTER_DURATION_S = 0.001s (1ms内完成一轮更新)
looptimeUs = 125μs (8kHz = 1/8000)

loopIterationsPerUpdate = 0.001 / 0.000125 = 8次循环
numNotchesPerAxis = 4 × 3 = 12个陷波
notchUpdatesPerIteration = ceil(12 / 8) = 2

结果：每个PID循环更新2个陷波滤波器
```

**更新循环**：
```
PID循环1: 更新电机0谐波0, 电机0谐波1
PID循环2: 更新电机0谐波2, 电机1谐波0
PID循环3: 更新电机1谐波1, 电机1谐波2
PID循环4: 更新电机2谐波0, 电机2谐波1
PID循环5: 更新电机2谐波2, 电机3谐波0
PID循环6: 更新电机3谐波1, 电机3谐波2
PID循环7-8: (部分或跳过，因为已更新完毕)

然后循环重新开始...
```

---

## 五、RPM滤波器实时更新

### 5.1 更新函数

**代码位置**：`src/main/flight/rpm_filter.c` 第110-169行

```c
FAST_CODE_NOINLINE void rpmFilterUpdate(void)
{
    if (!useDshotTelemetry) {
        return;
    }
    
    // 调试：输出每个电机的频率
    for (int motor = 0; motor < getMotorCount() && motor < DEBUG16_VALUE_COUNT; motor++) {
        DEBUG_SET(DEBUG_RPM_FILTER, motor, lrintf(getMotorFrequencyHz(motor)));
    }
    
    if (!isRpmFilterEnabled()) {
        return;
    }
    
    // 获取循环时间补偿（处理PID循环抖动）
    const float dtCompensation = schedulerGetCycleTimeMultiplier();
    const float correctedLooptime = rpmFilter.looptimeUs * dtCompensation;
    
    // 批量更新陷波滤波器
    for (int i = 0; i < notchUpdatesPerIteration; i++) {
        
        // 只更新有效的谐波（权重>0）
        if (rpmFilter.weights[harmonicIndex] > 0.0f) {
            
            // 选择ROLL轴的当前陷波作为模板
            biquadFilter_t *template = &rpmFilter.notch[0][motorIndex][harmonicIndex];
            
            // 计算陷波中心频率
            const float frequencyHz = constrainf(
                (harmonicIndex + 1) * getMotorFrequencyHz(motorIndex),  // 谐波倍数
                rpmFilter.minHz,                                         // 下限
                rpmFilter.maxHz                                          // 上限
            );
            
            // 计算权重（淡出 + 谐波权重）
            const float marginHz = frequencyHz - rpmFilter.minHz;
            float weight = 1.0f;
            
            // 淡出：接近minHz时逐渐关闭陷波
            if (marginHz < rpmFilter.fadeRangeHz) {
                weight *= marginHz / rpmFilter.fadeRangeHz;
            }
            
            // 应用谐波组权重
            weight *= rpmFilter.weights[harmonicIndex];
            
            // 更新陷波滤波器系数
            biquadFilterUpdate(template, frequencyHz, correctedLooptime, 
                              rpmFilter.q, FILTER_NOTCH, weight);
            
            // 复制系数到PITCH和YAW轴（所有轴使用相同频率）
            for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilter_t *dest = &rpmFilter.notch[axis][motorIndex][harmonicIndex];
                dest->b0 = template->b0;
                dest->b1 = template->b1;
                dest->b2 = template->b2;
                dest->a1 = template->a1;
                dest->a2 = template->a2;
                dest->weight = template->weight;
            }
        }
        
        // 循环遍历所有陷波
        harmonicIndex = (harmonicIndex + 1) % rpmFilter.numHarmonics;
        if (harmonicIndex == 0) {
            motorIndex = (motorIndex + 1) % getMotorCount();
        }
    }
}
```

### 5.2 频率和谐波计算

#### 5.2.1 基频和谐波

```c
// 第n个谐波的频率
harmonicFrequency = (harmonicIndex + 1) × motorFrequency

示例：电机频率 = 200Hz
- 谐波0（基频）: 1 × 200 = 200Hz
- 谐波1（2倍频）: 2 × 200 = 400Hz
- 谐波2（3倍频）: 3 × 200 = 600Hz
```

#### 5.2.2 为什么需要谐波？

**物理原因**：
1. **非线性效应**：电机和桨叶的振动不是完美正弦波
2. **多重来源**：
   - 电机电磁力：基频 + 谐波
   - 桨叶气动力：叶片通过频率及其谐波
   - 机械共振：可能在谐波频率激发

**频谱示例**：
```
振动幅度
  ↑
  │     ●                    电机基频（1x）
  │     │
  │     │  ●                 2倍谐波（2x）
  │     │  │
  │     │  │ ●              3倍谐波（3x）
  │     │  │ │
  └─────┴──┴─┴───────────→ 频率
       1x  2x 3x
```

### 5.3 淡出机制

#### 5.3.1 为什么需要淡出？

**问题**：
- 电机怠速或低速时，频率可能低于`minHz`（如100Hz）
- 如果硬性截断，会在minHz处产生突变
- 突变可能引起滤波效果不连续

**解决方案**：渐进淡出
```c
if (marginHz < rpmFilter.fadeRangeHz) {
    weight *= marginHz / rpmFilter.fadeRangeHz;
}
```

#### 5.3.2 淡出曲线

```
权重
  ↑
  │                      ┌────────────  weight = 1.0
  │                     ╱
  │                    ╱
  │                   ╱  线性淡入
  │                  ╱
  │                 ╱
  │                ╱
  │_______________╱____________________________→ 频率
  │              │     │
 0Hz          minHz  minHz+fadeRange
             (100)   (150)
```

**效果**：
- `频率 < minHz`：权重 = 0，陷波完全关闭
- `minHz < 频率 < minHz+fadeRange`：权重线性增加
- `频率 > minHz+fadeRange`：权重 = 1.0，陷波全效

### 5.4 陷波滤波器更新

#### 5.4.1 Biquad陷波滤波器

**更新函数**（简化说明）：
```c
void biquadFilterUpdate(biquadFilter_t *filter, 
                       float centerFreq,      // 中心频率
                       float looptimeUs,      // 循环时间
                       float Q,               // Q值
                       biquadFilterType_e type, // FILTER_NOTCH
                       float weight)          // 权重(0-1)
{
    // 计算归一化频率
    float omega = 2.0f * M_PIf * centerFreq * looptimeUs * 1e-6f;
    float sn = sin_approx(omega);
    float cs = cos_approx(omega);
    float alpha = sn / (2.0f * Q);
    
    // 陷波滤波器传递函数系数
    float b0 =  1;
    float b1 = -2 * cs;
    float b2 =  1;
    float a0 =  1 + alpha;
    float a1 = -2 * cs;
    float a2 =  1 - alpha;
    
    // 归一化
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
    
    // 存储权重（用于加权滤波）
    filter->weight = weight;
}
```

#### 5.4.2 陷波滤波器频率响应

```
幅度(dB)
  ↑
  │
 0│─────┐                    ┌─────
  │     │                    │
  │     │                    │
  │     │                    │
  │     │                    │
-40│     │      陷波深度      │
  │     │    (-40 to -60dB)  │
-60│     └────────────────────┘
  │           ↑
  └───────────┴───────────────────→ 频率
           centerFreq
           
带宽 = centerFreq / Q
Q=5.0 → 带宽约为中心频率的20%
```

### 5.5 系数复制优化

#### 5.5.1 为什么只计算ROLL轴？

**原因**：
- 所有3个轴（Roll/Pitch/Yaw）的电机振动频率相同
- Biquad系数计算涉及三角函数，CPU开销较大
- 系数与轴无关，只与频率、Q值相关

**优化策略**：
1. 只为ROLL轴计算系数（使用`sin`/`cos`）
2. 将系数直接复制到PITCH和YAW轴
3. CPU开销减少2/3

**代码**：
```c
// 计算ROLL轴（模板）
biquadFilter_t *template = &rpmFilter.notch[0][motorIndex][harmonicIndex];
biquadFilterUpdate(template, ...);

// 复制到其他轴
for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
    biquadFilter_t *dest = &rpmFilter.notch[axis][motorIndex][harmonicIndex];
    dest->b0 = template->b0;
    dest->b1 = template->b1;
    dest->b2 = template->b2;
    dest->a1 = template->a1;
    dest->a2 = template->a2;
    dest->weight = template->weight;
}
```

---

## 六、陀螺仪滤波应用

### 6.1 应用函数

**代码位置**：`src/main/flight/rpm_filter.c` 第171-187行

```c
FAST_CODE float rpmFilterApply(const int axis, float value)
{
    // 遍历所有谐波
    for (int i = 0; i < rpmFilter.numHarmonics; i++) {
        
        // 跳过无效谐波（权重<=0）
        if (rpmFilter.weights[i] <= 0.0f) {
            continue;
        }
        
        // 遍历所有电机
        for (int motor = 0; motor < getMotorCount(); motor++) {
            // 串联应用陷波滤波器
            value = biquadFilterApplyDF1Weighted(&rpmFilter.notch[axis][motor][i], value);
        }
    }
    
    return value;
}
```

### 6.2 在陀螺仪滤波链中的位置

**代码位置**：`src/main/sensors/gyro_filter_impl.c` 第48-50行

```c
#ifdef USE_RPM_FILTER
gyroADCf = rpmFilterApply(axis, gyroADCf);
#endif
```

**在完整滤波链中的位置**：
```
陀螺仪原始数据 (gyro.gyroADC)
    ↓
降采样滤波 (lowpass2或平均)
    ↓
【RPM滤波器】 ← 在这里应用！
    ↓
静态陷波1 (notchFilter1)
    ↓
静态陷波2 (notchFilter2)
    ↓
陀螺仪低通滤波器 (lowpassFilter)
    ↓
动态陷波滤波器 (dynNotchFilter)
    ↓
最终输出 (gyro.gyroADCf)
```

### 6.3 为什么放在滤波链早期？

#### 6.3.1 位置原因

**优势**：
1. **精确频率**：RPM数据精确，应尽早消除振动
2. **避免污染**：防止振动噪声影响后续滤波器
3. **保护动态陷波**：减少动态陷波的负担

**如果放在后期**：
- 振动噪声会通过其他滤波器
- 可能混叠或产生非线性效应
- 降低整体滤波效率

#### 6.3.2 与其他滤波器的配合

```
滤波器层次：
┌──────────────────────────────────┐
│  RPM滤波器                        │ ← 精确消除已知频率（电机）
│  - 基于真实RPM                    │
│  - 极窄陷波（Q=5）                │
│  - 多个谐波                       │
└──────────────────────────────────┘
          ↓
┌──────────────────────────────────┐
│  静态陷波滤波器                   │ ← 消除固定共振（框架）
│  - 手动配置                       │
│  - 框架共振点                     │
└──────────────────────────────────┘
          ↓
┌──────────────────────────────────┐
│  陀螺仪低通滤波器                 │ ← 衰减高频噪声
│  - PT1/PT2/Biquad                │
│  - 截止频率100-250Hz             │
└──────────────────────────────────┘
          ↓
┌──────────────────────────────────┐
│  动态陷波滤波器                   │ ← 清理剩余振动
│  - FFT分析                        │
│  - 自适应跟踪                     │
└──────────────────────────────────┘
```

### 6.4 加权Biquad滤波

#### 6.4.1 为什么需要加权？

**问题**：
- 标准Biquad要么完全开启，要么完全关闭
- 在频率边界（minHz附近）会产生突变

**解决方案**：加权滤波
```c
output = weight × filtered + (1 - weight) × input
```

#### 6.4.2 加权滤波实现

**代码位置**：`src/main/common/filter.c`

```c
FAST_CODE float biquadFilterApplyDF1Weighted(biquadFilter_t* filter, float input)
{
    // 标准Biquad滤波
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 
                       - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    // 更新状态
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = result;
    
    // 加权混合：根据weight在滤波输出和原始输入之间插值
    return filter->weight * result + (1.0f - filter->weight) * input;
}
```

**效果**：
- `weight = 0.0`：完全绕过滤波器（输出=输入）
- `weight = 0.5`：50%滤波效果
- `weight = 1.0`：完全滤波

**在淡出区域**：
```
频率 = 110Hz (minHz=100, fadeRange=50)
marginHz = 110 - 100 = 10Hz
weight = 10 / 50 = 0.2

结果：20%的陷波效果，平滑过渡
```

### 6.5 串联滤波

#### 6.5.1 多陷波串联

**代码示例**（4电机、3谐波）：
```c
// 串联应用12个陷波滤波器（4×3）
float gyroData = originalValue;

// 谐波0（基频）：
gyroData = biquad_motor0_harmonic0(gyroData);
gyroData = biquad_motor1_harmonic0(gyroData);
gyroData = biquad_motor2_harmonic0(gyroData);
gyroData = biquad_motor3_harmonic0(gyroData);

// 谐波1（2倍频）：
gyroData = biquad_motor0_harmonic1(gyroData);
gyroData = biquad_motor1_harmonic1(gyroData);
gyroData = biquad_motor2_harmonic1(gyroData);
gyroData = biquad_motor3_harmonic1(gyroData);

// 谐波2（3倍频）：
gyroData = biquad_motor0_harmonic2(gyroData);
gyroData = biquad_motor1_harmonic2(gyroData);
gyroData = biquad_motor2_harmonic2(gyroData);
gyroData = biquad_motor3_harmonic2(gyroData);

return gyroData;
```

#### 6.5.2 为什么可以串联？

**线性时不变系统（LTI）性质**：
- Biquad滤波器是线性的
- 多个线性滤波器串联，顺序不影响结果
- 总频率响应 = 各滤波器频率响应的乘积

**数学表示**：
```
H_total(f) = H_1(f) × H_2(f) × ... × H_n(f)

每个陷波在其中心频率产生衰减：
- 中心频率处：-40dB到-60dB
- 其他频率：接近0dB（不衰减）
```

#### 6.5.3 CPU性能考虑

**每个Biquad滤波操作**：
```
- 5次浮点乘法
- 4次浮点加法
- 2次状态更新（赋值）
- 1次加权混合（3次乘法 + 1次加法）

总计：约15-20个浮点运算
```

**完整RPM滤波**（4电机、3谐波、3轴）：
```
每轴：4 × 3 = 12次Biquad
全部：12 × 3 = 36次Biquad
总运算：36 × 15 = 540次浮点运算/PID循环

在F7处理器（216MHz）：
- 约0.5-1.0μs per Biquad
- 总耗时：约10-15μs
- 占用：10-15μs / 125μs (8kHz) = 8-12%
```

---

## 七、完整信号流程图

### 7.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                   RPM滤波器完整信号流程                       │
└─────────────────────────────────────────────────────────────┘

硬件层                  DShot层               数据处理层
────────                ───────               ──────────
  ESC                                         
   │                                          
   │ 双向DShot                                
   │ 遥测                                     
   ↓                                          
┌──────┐             ┌──────────┐           ┌──────────┐
│ GPIO │────DMA/───→│  解码     │──────────→│ eRPM数据 │
│ 引脚 │    PIO     │  GCR      │           │ 存储     │
└──────┘            │  差分曼码  │           └──────────┘
                    └──────────┘                  ↓
                                           ┌──────────┐
                                           │ eRPM→Hz  │
滤波器层                                    │ PT1平滑  │
────────                                    └──────────┘
                                                  ↓
┌──────────────────────────────────────┐   ┌──────────┐
│     RPM滤波器更新                     │←──│电机频率Hz│
│  - 计算谐波频率                       │   └──────────┘
│  - 更新Biquad系数                    │
│  - 淡出处理                          │
│  - 批量更新（分散CPU负载）            │
└──────────────────────────────────────┘
                ↓
        ┌──────────────┐
        │ 陷波滤波器阵列 │
        │ 3轴×4电机×3谐波│
        └──────────────┘
                ↓
        【应用于陀螺仪数据】
                ↓
          精确消除电机噪声
```

### 7.2 时序流程图

```
时间线（单个PID循环，8kHz = 125μs）
────────────────────────────────────────────────────────────→

0μs    │← PID循环开始
       │
5μs    ├─ 陀螺仪采样完成
       │  └─ 读取传感器ADC值
       │
10μs   ├─ 降采样滤波
       │  └─ gyro.sampleSum[axis]
       │
15μs   ├─【RPM滤波器应用】
       │  │
       │  ├─ rpmFilterApply(axis, gyroADCf)
       │  │  └─ 串联应用12个陷波（4电机×3谐波）
       │  │     └─ 每个Biquad: 0.5-1.0μs
       │  │
       │  └─ 完成：15-25μs
       │
25μs   ├─ 静态陷波滤波器
       │
30μs   ├─ 陀螺仪低通滤波器
       │
35μs   ├─ 动态陷波滤波器
       │
45μs   ├─【RPM滤波器更新】（每N个循环）
       │  │
       │  ├─ rpmFilterUpdate()
       │  │  └─ 更新2-4个陷波系数（批量）
       │  │     └─ Biquad系数计算（sin/cos）
       │  │
       │  └─ 完成：45-50μs
       │
50μs   ├─ PID计算
       │  ├─ D-term滤波
       │  ├─ P/I/D/F计算
       │  └─ 混控
       │
70μs   ├─ 电机输出
       │  └─ DShot协议发送
       │
75μs   ├─ DShot遥测接收开始（硬件自动）
       │  └─ 等待30μs后ESC响应
       │
       │  [硬件层自动接收遥测数据]
       │  [PIO/DMA捕获21位数据]
       │
120μs  ├─ updateDshotTelemetry()（异步）
       │  ├─ 解码GCR和差分曼彻斯特
       │  ├─ 提取eRPM值
       │  ├─ 转换为频率
       │  └─ PT1平滑
       │
125μs  │← 下一个PID循环开始
       │
       └─ [循环继续...]
```

### 7.3 数据流详细图

```
┌──────────────────────────────────────────────────────────┐
│ 步骤1: ESC遥测发送                                        │
└──────────────────────────────────────────────────────────┘
  ESC内部: 测量电机eRPM
      ↓
  周期(μs) = 尾数 << 指数
      ↓
  eRPM × 100 = (60 × 10^6 / 100) / 周期
      ↓
  16位数据: [eee][mmmmmmmmm][校验]
      ↓
  GCR编码: 16位 → 20位
      ↓
  差分曼彻斯特: 20位 → 21位
      ↓
  发送到飞控（5/4倍DShot速率）

┌──────────────────────────────────────────────────────────┐
│ 步骤2: 飞控接收和解码                                     │
└──────────────────────────────────────────────────────────┘
  GPIO/DMA/PIO: 接收21位数据（3倍过采样）
      ↓
  多数表决: 60位采样 → 20位GCR
      ↓
  差分曼彻斯特解码: 20位 → 16位原始
      ↓
  GCR解码: 4×5位 → 4×4位 (16位)
      ↓
  校验和验证
      ↓
  提取12位eRPM数据
      ↓
  dshotTelemetryState.motorState[i].rawValue

┌──────────────────────────────────────────────────────────┐
│ 步骤3: 数据转换                                           │
└──────────────────────────────────────────────────────────┘
  eRPM × 100 (原始值)
      ↓
  实际RPM = (eRPM × 100) / (磁极对数)
      ↓
  电机频率(Hz) = RPM / 60
      ↓
  PT1低通滤波 (fc=150Hz)
      ↓
  motorFrequencyHz[motor] (平滑的频率)

┌──────────────────────────────────────────────────────────┐
│ 步骤4: RPM滤波器更新                                      │
└──────────────────────────────────────────────────────────┘
  对于每个[电机×谐波]:
      ↓
  中心频率 = (谐波序号+1) × motorFrequencyHz
      ↓
  限制范围: [minHz, maxHz]
      ↓
  计算权重:
    - 淡出因子 (接近minHz时)
    - 谐波权重 (配置参数)
      ↓
  更新Biquad系数:
    - 计算ω = 2πf × dT
    - 计算sin(ω), cos(ω)
    - 计算b0, b1, b2, a1, a2
      ↓
  复制系数到其他轴
      ↓
  rpmFilter.notch[axis][motor][harmonic]

┌──────────────────────────────────────────────────────────┐
│ 步骤5: 陀螺仪滤波应用                                     │
└──────────────────────────────────────────────────────────┘
  陀螺仪原始数据: gyro.gyroADC[axis]
      ↓
  降采样: gyroADCf
      ↓
  【RPM滤波器应用】
    │
    ├─ 谐波0（基频）:
    │  ├─ 陷波[motor0][harm0] → gyroADCf
    │  ├─ 陷波[motor1][harm0] → gyroADCf
    │  ├─ 陷波[motor2][harm0] → gyroADCf
    │  └─ 陷波[motor3][harm0] → gyroADCf
    │
    ├─ 谐波1（2倍频）:
    │  ├─ 陷波[motor0][harm1] → gyroADCf
    │  ├─ 陷波[motor1][harm1] → gyroADCf
    │  ├─ 陷波[motor2][harm1] → gyroADCf
    │  └─ 陷波[motor3][harm1] → gyroADCf
    │
    └─ 谐波2（3倍频）:
       ├─ 陷波[motor0][harm2] → gyroADCf
       ├─ 陷波[motor1][harm2] → gyroADCf
       ├─ 陷波[motor2][harm2] → gyroADCf
       └─ 陷波[motor3][harm2] → gyroADCf
      ↓
  精确消除电机振动的陀螺仪数据
      ↓
  继续后续滤波...
```

### 7.4 频率跟踪示例

```
场景：从悬停到全油门

时间  油门   电机RPM  基频(Hz)  2倍(Hz)  3倍(Hz)  RPM滤波器陷波中心
────  ────  ────────  ────────  ───────  ───────  ─────────────────
0s    20%   5,000     167       334      501      167, 334, 501
1s    30%   7,000     233       467      700      233, 467, 700
2s    50%   10,000    333       667      1000     333, 667, 1000
3s    70%   13,000    433       867      1300     433, 867, 1300
4s    90%   15,000    500       1000     1500     500, 1000, 1500

说明：
- 电机14极（7对），桨叶4片
- 基频 = RPM / 60 (电机旋转频率)
- 谐波自动跟踪基频变化
- 每个频率都有对应的陷波滤波器精确消除
```

---

## 八、代码文件索引

### 8.1 核心文件

#### 8.1.1 DShot遥测接收和解码

**`src/main/drivers/dshot.c`**
- **功能**：DShot协议主实现，遥测数据处理
- **关键函数**：
  - `initDshotTelemetry()` (第175-196行)
    - 初始化遥测系统
    - 计算eRPM到Hz的转换系数
    - 初始化电机频率LPF
  
  - `dshot_decode_eRPM_telemetry_value()` (第198-213行)
    - 解码GCR格式的eRPM值
    - 提取指数和尾数
    - 计算eRPM × 100
  
  - `dshot_decode_telemetry_value()` (第215-242行)
    - 判断遥测类型（eRPM或扩展遥测）
    - 调用相应解码函数
  
  - `dshotUpdateTelemetryData()` (第244-254行)
    - 更新遥测数据存储
    - 处理温度、电压、电流等
  
  - `updateDshotTelemetry()` (第256-303行)
    - **主遥测处理函数**
    - 批量解码所有电机
    - 转换eRPM为频率
    - 应用PT1平滑
    - 更新`motorFrequencyHz[]`
  
  - `getMotorFrequencyHz()` (第320-323行)
    - 返回平滑后的电机频率
    - RPM滤波器从这里获取频率数据
  
  - `erpmToRpm()` (第381-385行)
    - eRPM转换为实际RPM

**关键数据结构**：
```c
// 第147-152行
FAST_DATA_ZERO_INIT static pt1Filter_t motorFreqLpf[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float motorFrequencyHz[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float minMotorFrequencyHz;
FAST_DATA_ZERO_INIT static float erpmToHz;
FAST_DATA_ZERO_INIT static float dshotRpmAverage;
FAST_DATA_ZERO_INIT static float dshotRpm[MAX_SUPPORTED_MOTORS];
```

---

#### 8.1.2 硬件层遥测接收（平台相关）

**STM32平台：**
- **`src/platform/STM32/dshot_bitbang.c`**
  - 使用DMA + 定时器捕获
  - 中断驱动的解码
  
- **`src/platform/common/stm32/pwm_output_dshot_shared.c`**
  - 共享的DShot输出代码

**RP2040 (Pico)平台：**
- **`src/platform/PICO/dshot_bidir_pico.c`**
  - **关键函数**：
    - `decodeTelemetryRaw()` (第148-296行)
      - 3倍过采样多数表决
      - 差分曼彻斯特解码
      - GCR解码
      - 校验和验证
    
    - `dshotDecodeTelemetry()` (第300-376行)
      - 从PIO FIFO读取数据
      - 调用`decodeTelemetryRaw()`
      - 存储解码结果

- **`src/platform/PICO/dshot.pio`**
  - PIO状态机程序
  - 第97-119行：遥测接收状态机
  - 自动3倍过采样

**AT32平台：**
- **`src/platform/AT32/dshot_bitbang.c`**
  - AT32特定的DMA实现

**APM32平台：**
- **`src/platform/APM32/dshot_bitbang.c`**
  - APM32特定的DMA实现

---

#### 8.1.3 RPM滤波器核心

**`src/main/flight/rpm_filter.c`**
- **功能**：RPM滤波器主实现
- **关键函数**：
  - `rpmFilterInit()` (第69-108行)
    - 初始化RPM滤波器
    - 创建陷波滤波器阵列
    - 计算批量更新参数
  
  - `rpmFilterUpdate()` (第110-169行)
    - **实时更新陷波频率**
    - 批量更新Biquad系数
    - 淡出处理
    - 系数复制优化
  
  - `rpmFilterApply()` (第171-187行)
    - **应用RPM滤波到陀螺仪数据**
    - 串联所有陷波滤波器
    - 加权Biquad应用
  
  - `isRpmFilterEnabled()` (第189-192行)
    - 检查RPM滤波器是否启用

**关键数据结构**：
```c
// 第47-59行
typedef struct rpmFilter_s {
    int numHarmonics;              // 谐波数量
    float weights[3];              // 谐波权重
    float minHz;                   // 最小频率
    float maxHz;                   // 最大频率
    float fadeRangeHz;             // 淡出范围
    float q;                       // Q值
    timeUs_t looptimeUs;           // 循环时间
    // 3轴 × 最多8电机 × 最多3谐波
    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_HARMONICS_MAX];
} rpmFilter_t;

// 第62行：单例
FAST_DATA_ZERO_INIT static rpmFilter_t rpmFilter;
```

---

#### 8.1.4 RPM滤波器配置

**`src/main/pg/rpm_filter.h`**
- **功能**：参数定义和接口
- **配置结构** (第29-39行)：
  ```c
  typedef struct rpmFilterConfig_s {
      uint8_t  rpm_filter_harmonics;      // 谐波数量
      uint8_t  rpm_filter_weights[3];     // 谐波权重
      uint8_t  rpm_filter_min_hz;         // 最小频率
      uint16_t rpm_filter_fade_range_hz;  // 淡出范围
      uint16_t rpm_filter_q;              // Q值
      uint16_t rpm_filter_lpf_hz;         // LPF截止频率
  } rpmFilterConfig_t;
  ```

**`src/main/pg/rpm_filter.c`**
- **功能**：参数组注册和默认值
- **默认配置** (第32-39行)：
  ```c
  PG_RESET_TEMPLATE(rpmFilterConfig_t, rpmFilterConfig,
      .rpm_filter_harmonics = 3,
      .rpm_filter_min_hz = 100,
      .rpm_filter_fade_range_hz = 50,
      .rpm_filter_q = 500,
      .rpm_filter_lpf_hz = 150,
      .rpm_filter_weights = { 100, 100, 100 },
  );
  ```

**`src/main/flight/rpm_filter.h`**
- **功能**：外部接口声明
- **函数接口** (第29-32行)：
  ```c
  void rpmFilterInit(const rpmFilterConfig_t *config, const timeUs_t looptimeUs);
  void rpmFilterUpdate(void);
  float rpmFilterApply(const int axis, float value);
  bool isRpmFilterEnabled(void);
  ```

---

#### 8.1.5 陀螺仪滤波集成

**`src/main/sensors/gyro_filter_impl.c`**
- **功能**：陀螺仪滤波链实现
- **RPM滤波器应用位置** (第48-50行)：
  ```c
  #ifdef USE_RPM_FILTER
  gyroADCf = rpmFilterApply(axis, gyroADCf);
  #endif
  ```
  
**完整滤波链** (第32-84行)：
  1. 降采样 (第32-43行)
  2. **RPM滤波器** (第48-50行) ← 这里！
  3. 静态陷波1 (第56行)
  4. 静态陷波2 (第57行)
  5. 陀螺仪低通 (第58行)
  6. 动态陷波 (第64-78行)
  7. 最终输出 (第84行)

**`src/main/sensors/gyro.c`**
- **功能**：陀螺仪数据处理主文件
- **关键函数**：
  - `gyroUpdate()` (第408-445行) - 采样和累积
  - `gyroFiltering()` (第463-520行) - 调用滤波链

---

### 8.2 辅助文件

#### 8.2.1 滤波器算法库

**`src/main/common/filter.c`**
- **Biquad滤波器实现**：
  - `biquadFilterInit()` - 初始化
  - `biquadFilterUpdate()` - 更新系数
  - `biquadFilterApply()` - DF2实现
  - `biquadFilterApplyDF1()` - DF1实现
  - `biquadFilterApplyDF1Weighted()` - 加权DF1（RPM滤波器使用）

- **PT1滤波器实现**：
  - `pt1FilterInit()`
  - `pt1FilterApply()` - eRPM数据平滑使用

**`src/main/common/filter.h`**
- **数据结构定义**：
  - `biquadFilter_t` - Biquad滤波器状态
  - `pt1Filter_t` - PT1滤波器状态
  - `filter_t` - 通用滤波器联合体

---

#### 8.2.2 DShot命令和配置

**`src/main/drivers/dshot_command.h`**
- DShot命令定义
- 包括扩展遥测启用命令

**`src/main/pg/motor.c`** / **`motor.h`**
- 电机配置参数
- 包括：
  - `motorPoleCount` - 磁极数
  - `useDshotTelemetry` - 启用双向DShot
  - `useDshotEdt` - 扩展遥测模式

---

#### 8.2.3 调度和时序

**`src/main/scheduler/scheduler.c`**
- **`schedulerGetCycleTimeMultiplier()`**
  - RPM滤波器更新时使用
  - 补偿PID循环抖动

**`src/main/fc/core.c`**
- 主循环
- 调用`updateDshotTelemetry()`

---

### 8.3 代码导航地图

```
RPM滤波器代码结构
├── 硬件层（平台相关）
│   ├── STM32/dshot_bitbang.c          ← DMA接收
│   ├── PICO/dshot_bidir_pico.c        ← PIO接收和解码
│   ├── PICO/dshot.pio                 ← PIO状态机
│   ├── AT32/dshot_bitbang.c           ← AT32 DMA
│   └── APM32/dshot_bitbang.c          ← APM32 DMA
│
├── DShot协议层
│   ├── drivers/dshot.c                ← 主实现（遥测解码、频率转换）
│   ├── drivers/dshot_command.h        ← 命令定义
│   └── drivers/dshot_bitbang_decode.c ← 解码辅助
│
├── RPM滤波器层
│   ├── flight/rpm_filter.c            ← 核心实现（初始化、更新、应用）
│   ├── flight/rpm_filter.h            ← 接口声明
│   ├── pg/rpm_filter.c                ← 参数组和默认值
│   └── pg/rpm_filter.h                ← 配置结构
│
├── 陀螺仪滤波集成
│   ├── sensors/gyro_filter_impl.c     ← 滤波链（RPM滤波器应用点）
│   ├── sensors/gyro.c                 ← 陀螺仪数据处理
│   └── sensors/gyro_init.c            ← 初始化
│
├── 滤波器算法库
│   ├── common/filter.c                ← Biquad和PT1实现
│   └── common/filter.h                ← 数据结构
│
└── 配置和调度
    ├── pg/motor.c/h                   ← 电机配置
    ├── scheduler/scheduler.c          ← 时序管理
    └── fc/core.c                      ← 主循环
```

---

### 8.4 调试相关

#### 8.4.1 Debug模式

**RPM滤波器调试**：
- `DEBUG_RPM_FILTER` - 显示每个电机的频率
- **代码位置**：`rpm_filter.c` 第116-118行

**DShot遥测调试**：
- `DEBUG_DSHOT_RPM_TELEMETRY` - 显示解码的eRPM
- **代码位置**：`dshot.c` 第230-232行

**使用方法**：
```c
// 在Betaflight CLI中设置
set debug_mode = RPM_FILTER
// 或
set debug_mode = DSHOT_RPM_TELEMETRY

// 在黑盒日志中查看debug[0-3]字段
```

#### 8.4.2 遥测质量统计

**`src/main/drivers/dshot_telemetry_stats.c`**（如果启用）
- 跟踪遥测数据包成功率
- 检测丢包和错误

---

## 九、系统频率关系详解

### 9.1 频率层次结构

Betaflight系统中存在多个不同的频率，它们相互关联但又各司其职：

```
硬件采样层
    ↓
┌─────────────────────────────────────────┐
│ 陀螺仪采样频率 (Gyro Sample Rate)       │
│ - 典型值：8000 Hz (8kHz)                │
│ - 其他：3200, 6400, 9000 Hz             │
│ - 决定因素：陀螺仪芯片型号               │
└─────────────────────────────────────────┘
    ↓ (可能降采样)
┌─────────────────────────────────────────┐
│ PID 循环频率 (PID Loop Rate)            │
│ - 典型值：8000, 4000, 2000 Hz           │
│ - 关系：= 陀螺仪频率 / pid_process_denom │
│ - 所有滤波和控制在此频率执行             │
└─────────────────────────────────────────┘
    ↓ (并行)
┌─────────────────────────────────────────┐
│ DShot 输出频率 (Motor Update Rate)      │
│ - DShot150: 最高 4000 Hz                │
│ - DShot300: 最高 8000 Hz                │
│ - DShot600: 最高 16000 Hz               │
│ - 实际频率 ≤ PID循环频率                │
└─────────────────────────────────────────┘
    ↓ (遥测反馈)
┌─────────────────────────────────────────┐
│ DShot 遥测接收频率                       │
│ - 等于 DShot 输出频率                   │
│ - 每发送一次命令，接收一次遥测            │
│ - 实时解码 eRPM 数据                    │
└─────────────────────────────────────────┘
    ↓ (数据处理)
┌─────────────────────────────────────────┐
│ eRPM 数据更新频率                        │
│ - 等于 PID 循环频率                     │
│ - updateDshotTelemetry() 每循环调用     │
└─────────────────────────────────────────┘
    ↓ (低通滤波)
┌─────────────────────────────────────────┐
│ RPM 低通滤波器 (motorFreqLpf)            │
│ - 截止频率：150 Hz (默认)               │
│ - 作用：平滑 eRPM 数据                  │
│ - 输出：motorFrequencyHz[]              │
└─────────────────────────────────────────┘
    ↓ (滤波器更新)
┌─────────────────────────────────────────┐
│ RPM 滤波器更新频率                       │
│ - 批量更新，分散到多个 PID 循环          │
│ - 完整更新周期：约 1ms                  │
│ - 每个陷波在 1ms 内完成一次更新          │
└─────────────────────────────────────────┘
    ↓ (应用滤波)
┌─────────────────────────────────────────┐
│ RPM 滤波器应用频率                       │
│ - 等于 PID 循环频率                     │
│ - 每个陀螺仪样本都经过 RPM 滤波          │
└─────────────────────────────────────────┘

独立采样
    ↓
┌─────────────────────────────────────────┐
│ 加速度计采样频率 (Accelerometer)        │
│ - 典型值：1000 Hz                       │
│ - 用于姿态解算                          │
└─────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────┐
│ IMU 更新频率 (Attitude Update)          │
│ - 典型值：100 Hz                        │
│ - imuUpdateAttitude() 任务              │
│ - 使用平滑后的陀螺仪数据（50Hz LPF）     │
└─────────────────────────────────────────┘
```

### 9.2 频率之间的关系

#### 9.2.1 陀螺仪采样 vs PID循环

```
关系：PID循环频率 = 陀螺仪采样频率 / pid_process_denom

示例：
- 陀螺仪：8000 Hz
- pid_process_denom = 1 → PID = 8000 Hz (1:1)
- pid_process_denom = 2 → PID = 4000 Hz (2:1)
- pid_process_denom = 4 → PID = 2000 Hz (4:1)

代码位置：src/main/sensors/gyro.c
```

#### 9.2.2 PID循环 vs DShot遥测

```
关系：DShot 遥测频率 = PID 循环频率（在协议允许的范围内）

DShot 协议限制：
- DShot150: 最大 4kHz 电机更新
- DShot300: 最大 8kHz 电机更新  
- DShot600: 最大 16kHz 电机更新

实际频率：
如果 PID = 8kHz:
  - DShot300 或 DShot600: 遥测频率 = 8kHz
  - DShot150: 遥测频率 = 4kHz (自动降频)

代码位置：src/main/config/config.c 第611-617行
```

#### 9.2.3 DShot遥测 vs eRPM数据

```
关系：eRPM 数据更新 = DShot 遥测接收频率

流程：
1. 每个 PID 循环发送 DShot 指令
2. 30μs 后 ESC 发送遥测
3. 硬件自动接收（DMA/PIO）
4. updateDshotTelemetry() 解码数据

频率：
- 8kHz PID + DShot300: 每 125μs 更新一次 eRPM
- 4kHz PID + DShot150: 每 250μs 更新一次 eRPM

代码位置：src/main/drivers/dshot.c 第256-303行
```

#### 9.2.4 eRPM数据 vs RPM低通滤波

```
关系：RPM 低通滤波器每个 PID 循环应用一次

PT1 滤波器：
- 截止频率：150 Hz (默认)
- 采样频率：= PID 循环频率
- 群延迟：约 1 / (2π × 150) ≈ 1.06 ms

效果：
- 平滑 eRPM 数据的抖动
- 保留快速的转速变化（< 150Hz）
- 衰减测量噪声和量化误差

代码位置：src/main/drivers/dshot.c 第297行
```

#### 9.2.5 RPM滤波器更新 vs 应用

```
更新频率（批量）：
- 所有陷波在 1ms 内完成一次更新
- 每个 PID 循环更新 2-4 个陷波（分散负载）

应用频率：
- 等于 PID 循环频率
- 每个陀螺仪样本都经过完整的 RPM 滤波

示例（8kHz PID，4电机，3谐波）：
- 总陷波数：4 × 3 = 12 个/轴
- 每次 PID 循环更新：2 个陷波
- 完整更新周期：12 / 2 = 6 个 PID 循环 = 0.75ms

代码位置：
- 更新：src/main/flight/rpm_filter.c 第110-169行
- 应用：src/main/flight/rpm_filter.c 第171-187行
```

#### 9.2.6 陀螺仪采样 vs IMU更新

```
关系：IMU 独立于 PID 循环，频率低得多

陀螺仪数据路径：
1. 原始陀螺仪 (8kHz) → PID 使用
2. 平滑陀螺仪 (50Hz LPF) → IMU 使用

IMU 更新：
- 频率：100 Hz (固定任务周期)
- 使用 gyroFilteredDownsampled[]
- PT1 滤波器截止频率：50 Hz

原因：
- 姿态解算不需要高频数据
- 降低 CPU 负担
- 提供平滑的角度估计

代码位置：
- IMU 滤波：src/main/sensors/gyro.c 第513行
- IMU 更新：src/main/flight/imu.c
- 任务配置：src/main/fc/tasks.c 第378行
```

### 9.3 频率配置表

| 参数 | 典型值 | 配置位置 | 说明 |
|-----|-------|---------|-----|
| **陀螺仪采样** | 8000 Hz | 硬件自动 | 由陀螺仪芯片决定 |
| **PID 循环** | 8000 Hz | `pid_process_denom=1` | gyro_freq / denom |
| **DShot 输出** | 8000 Hz | `motor_protocol=DSHOT300` | 协议决定最大值 |
| **DShot 遥测** | 8000 Hz | 自动 | = DShot 输出频率 |
| **eRPM 更新** | 8000 Hz | 自动 | 每个 PID 循环 |
| **RPM LPF** | 150 Hz | `rpm_filter_lpf_hz=150` | PT1 截止频率 |
| **RPM 滤波更新** | ~1333 Hz | 自动计算 | 分散到 6 个循环 |
| **RPM 滤波应用** | 8000 Hz | 自动 | 每个 PID 循环 |
| **加速度计** | 1000 Hz | 硬件自动 | 独立采样 |
| **IMU 更新** | 100 Hz | 固定任务 | 姿态解算 |
| **IMU 陀螺 LPF** | 50 Hz | 固定 | PT1 滤波器 |

### 9.4 时序关系示意图

```
时间轴（8kHz PID，125μs 周期）
────────────────────────────────────────────────────────────→

PID循环 #1 (0μs)
├─ 陀螺仪采样 (8kHz)
├─ RPM滤波应用 (8kHz) ← 使用上次更新的陷波系数
├─ 其他陀螺仪滤波
├─ PID计算
├─ DShot输出 (8kHz)
│  └─ 30μs后 ESC发送遥测
├─ 遥测接收（硬件自动）
├─ eRPM数据解码 (8kHz)
├─ RPM低通滤波 (150Hz截止)
└─ RPM滤波器更新 (更新2个陷波)

PID循环 #2 (125μs)
├─ 陀螺仪采样
├─ RPM滤波应用
├─ ...
└─ RPM滤波器更新 (更新另外2个陷波)

PID循环 #3-6 (250-750μs)
└─ 继续批量更新陷波...

PID循环 #7 (750μs)
└─ 所有陷波完成一轮更新

── 独立时间线 ──

每 10ms (100Hz)
└─ IMU姿态更新
   ├─ 使用50Hz LPF的陀螺仪数据
   ├─ 使用1000Hz的加速度计数据
   └─ 四元数更新
```

### 9.5 频率选择建议

#### 对于 F4 / F7 处理器：
```
推荐配置：
- 陀螺仪：8000 Hz
- PID 循环：4000 Hz (pid_process_denom=2)
- DShot：DShot300
- 遥测频率：4000 Hz
- RPM LPF：150 Hz

原因：
- F4/F7 在 8kHz 可能 CPU 吃紧
- 4kHz 足够精确，留有余量
- DShot300 匹配 4kHz 更新
```

#### 对于 H7 处理器：
```
推荐配置：
- 陀螺仪：8000 Hz
- PID 循环：8000 Hz (pid_process_denom=1)
- DShot：DShot600
- 遥测频率：8000 Hz
- RPM LPF：150 Hz

原因：
- H7 性能强大，支持 8kHz
- 更高频率 = 更低延迟
- DShot600 充分利用带宽
```

### 9.6 频率对性能的影响

| 频率参数 | 提高的影响 | 降低的影响 |
|---------|----------|----------|
| **PID循环** | 延迟减少，响应更快 | CPU负载降低，更稳定 |
| **DShot遥测** | eRPM更新更快 | 可能丢包增加 |
| **RPM LPF** | 跟踪速度更快，延迟小 | 噪声抑制更好 |
| **IMU更新** | 姿态更精确（意义不大）| CPU负载减少 |

---

## 总结

### RPM滤波器的优势

1. **极高精度**：直接基于电机实际转速，误差极小
2. **全范围跟踪**：从悬停到全油门，实时跟踪频率变化
3. **多谐波覆盖**：消除基频及2倍、3倍谐波
4. **低CPU开销**：陷波滤波器计算量小，批量更新分散负载
5. **零配置**：完全自动，无需手动调整

### 关键技术点

1. **双向DShot**：高速、可靠的遥测反馈机制
2. **GCR编码**：确保数据传输可靠性
3. **PT1平滑**：消除eRPM数据的抖动和噪声
4. **批量更新**：分散Biquad系数计算的CPU负担
5. **淡出机制**：避免低频时的滤波突变
6. **系数复制**：优化多轴滤波器的计算效率
7. **加权滤波**：平滑的滤波器开关，避免瞬态

### 性能参数（典型值）

- **遥测更新率**：约2-8kHz（取决于DShot协议）
- **频率跟踪延迟**：约1-2ms（PT1滤波150Hz）
- **陷波Q值**：5.0（极窄，精确消除）
- **谐波覆盖**：1x、2x、3x（基频到3倍频）
- **CPU开销**：8-12%（F7处理器@8kHz PID）

### 数据精度说明

**重要**：`dshot_decode_eRPM_telemetry_value()` 返回的是 **eRPM / 100**
- 返回值范围：0-4095（12位）
- 真实 eRPM = 返回值 × 100
- 最大可表示：409,500 eRPM
- 设计目的：节省传输带宽和存储空间

---

**文档版本**：2.0  
**更新日期**：2024年10月  
**基于代码版本**：Betaflight 4.5+  
**最后修订**：修正 eRPM × 100 错误，添加系统频率关系详解

