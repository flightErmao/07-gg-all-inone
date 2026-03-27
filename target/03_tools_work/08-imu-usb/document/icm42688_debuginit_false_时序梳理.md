# ICM42688 `DebugInit(false)` 时序梳理

## 说明

- 本文基于 `D:\58-imu-verify-solution\03-firmware\ref_driver\icm42688.c` 中的 `int ICM42688::DebugInit(bool clkin_enable)` 分析。
- 本文只分析 `clkin_enable = false` 这条执行路径。
- `DebugInit()` 这个无参重载实际调用的是 `DebugInit(true)`，所以 `DebugInit(false)` 必须由外部显式传 `false` 才会走到本文这条路径。
- 代码里大量使用 `WriteMask()`，它的行为是“先读寄存器，再按 mask 改位，最后写回”。因此“写入数据”一栏分成两种：
  - `reg_value`：函数传给 `WriteMask()` 的目标位值。
  - `最终写回值`：只有在当前函数内部已经确定了同一寄存器其它相关位时，才写成固定字节；否则写成“保留原值某些位”的表达式。
- 对于 `clkin_enable = false`，`RTC mode`、`timestamp register`、`nflt gyro` 这几项不会实际写寄存器。

## 1. 按时间顺序的功能设置

| 序号 | 调用/步骤 | 功能说明 | 是否实际写寄存器 |
| --- | --- | --- | --- |
| 1 | 读取 `WHO_AM_I` | 读取芯片 ID，确认设备是 ICM42688，期望值 `0x47` | 否，仅读 |
| 2 | `icm4x6xx_config_ui_intf(SPI_INTF)` | 选择 UI 接口为 SPI，`SPI_INTF = 3` | 是 |
| 3 | `icm4x6xx_enable_rtc_mode(false)` | 因为参数是 `false`，函数直接返回，不开启 `CLKIN/RTC mode` | 否 |
| 4 | `icm4x6xx_enable_tmst(true)` | 只有 `clkin_enable = true` 才执行；当前路径跳过 | 否 |
| 5 | `icm4x6xx_en_big_endian_mode(true)` | FIFO 计数和传感器数据使用大端模式 | 是 |
| 6 | `icm4x6xx_config_fsync(0)` | 清除 `FSYNC_CONFIG[6:4]`，不让 FSYNC 去标记温度分辨率路径 | 是 |
| 7 | `icm4x6xx_set_accel_fsr(ACC_RANGE_16G)` | 加速度量程设置为 `16g` | 是 |
| 8 | `icm4x6xx_set_gyro_fsr(GYRO_RANGE_2000DPS)` | 陀螺量程设置为 `2000 dps` | 是 |
| 9 | `icm4x6xx_set_accel_filter_order(THIRD_ORDER)` | 加速度滤波器阶数设置为三阶 | 是 |
| 10 | `icm4x6xx_set_gyro_filter_order(THIRD_ORDER)` | 陀螺滤波器阶数设置为三阶 | 是 |
| 11 | `icm4x6xx_set_fifo_mode(STREAM)` | FIFO 工作模式设置为 `STREAM` | 是 |
| 12 | `icm4x6xx_enable_nflt_gyro(true)` | 只有 `clkin_enable = true` 才执行；当前路径跳过 | 否 |
| 13 | `icm4x6xx_disable_aux_pins()` | 切到 bank 2，依次关闭 AUX 相关 pad，然后切回 bank 0 | 是 |
| 14 | `icm4x6xx_set_accel_odr(ICM4X6XX_ODR_1000)` | 加速度 ODR 设置为 `1000 Hz`，转换后的寄存器值是 `ODR_1KHZ = 6` | 是 |
| 15 | `icm4x6xx_set_gyro_odr(ICM4X6XX_ODR_1000)` | 陀螺 ODR 设置为 `1000 Hz`，转换后的寄存器值是 `ODR_1KHZ = 6` | 是 |
| 16 | `icm4x6xx_set_accel_bandwidth(BW_ODR_DIV_5)` | 加速度带宽设置为 `ODR/5`，枚举值 `2` | 是 |
| 17 | `icm4x6xx_set_gyro_bandwidth(BW_ODR_DIV_2)` | 陀螺带宽设置为 `ODR/2`，枚举值 `0` | 是 |
| 18 | `icm4x6xx_en_fifo(true, true)` | 使能 accel FIFO、gyro FIFO，同时带上温度数据，hi-res 关闭 | 是 |
| 19 | `icm4x6xx_accel_gyro_powerup(ACCEL_GYRO_POWERUP)` | 同时拉起 accel 和 gyro 到 low noise mode，`ACCEL_GYRO_POWERUP = 0x0F` | 是 |
| 20 | `icm4x6xx_disable_afsr()` | 对 `0x4D` 寄存器执行 `mask=0xC0` 的写掩码操作，目标值 `0x40` | 是 |

## 2. 按时间顺序的寄存器/数据表

| 序号 | 步骤 | Bank | 地址寄存器 | 写入方式 | reg_value / 直接写值 | mask | 最终写回值 | 说明 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 读取 `WHO_AM_I` | 0 | `0x75` | 读 | 期望读到 `0x47` | - | - | 设备识别，不写寄存器 |
| 2 | 配置 UI 为 SPI | 0 | `0x4C` (`REG_INTF_CONFIG0`) | `WriteMask()` | `0x03` | `0x03` | `0x03` | 只改 bit[1:0]，`SPI_INTF = 3` |
| 3 | `RTC mode = false` | - | - | 无写入 | - | - | - | 该函数直接返回 |
| 4 | `TMST enable` 跳过 | - | - | 无写入 | - | - | - | 只有 `clkin_enable = true` 才执行 |
| 5 | 使能大端模式 | 0 | `0x4C` (`REG_INTF_CONFIG0`) | `WriteMask()` | `0x30` | `0x30` | `0x33` | 在第 2 步基础上叠加，置位 `FIFO_COUNT_BIG_ENDIAN_MASK(0x20)` 和 `SENSOR_DATA_BIG_ENDIAN_MASK(0x10)` |
| 6 | 配置 FSYNC | 0 | `0x62` (`REG_FSYNC_CONFIG`) | `WriteMask()` | `0x00` | `0x70` | `0x00` | 清 bit[6:4] |
| 7 | 配置 accel FSR | 0 | `0x50` (`REG_ACCEL_CONFIG0`) | `WriteMask()` | `0x00` | `0xE0` | `0x00` | `ACC_RANGE_16G = 0`，写入 `0 << 5` |
| 8 | 配置 gyro FSR | 0 | `0x4F` (`REG_GYRO_CONFIG0`) | `WriteMask()` | `0x00` | `0xE0` | `0x00` | `GYRO_RANGE_2000DPS = 0`，写入 `0 << 5` |
| 9 | accel 三阶滤波 | 0 | `0x53` (`REG_ACC_CONFIG1`) | `WriteMask()` | `0x10` | `0x18` | `0x10` | `THIRD_ORDER = 2`，写入 `2 << 3` |
| 10 | gyro 三阶滤波 | 0 | `0x51` (`REG_GYRO_CONFIG1`) | `WriteMask()` | `0x08` | `0x0C` | `0x08` | `THIRD_ORDER = 2`，写入 `2 << 2` |
| 11 | FIFO 模式设为 STREAM | 0 | `0x16` (`REG_FIFO_CONFIG`) | `WriteMask()` | `0x40` | `0xC0` | `0x40` | `STREAM = 1`，写入 `1 << 6` |
| 12 | 切换到 bank 2 | bank select | `0x76` (`REG_BANK_SEL`) | 直接写 | `0x02` | - | `0x02` | 为 AUX pin 配置做准备 |
| 13 | 关闭 AUX pin 0x70 | 2 | `0x70` | 直接写 | `0x01` | - | `0x01` | `icm4x6xx_disable_aux_pins()` 内第一笔直接写 |
| 14 | 关闭 AUX pin 0x71 | 2 | `0x71` | 直接写 | `0x01` | - | `0x01` | 同上 |
| 15 | 关闭 AUX pin 0x72 | 2 | `0x72` | 直接写 | `0x01` | - | `0x01` | 同上 |
| 16 | 关闭 AUX pin 0x73 | 2 | `0x73` | 直接写 | `0x01` | - | `0x01` | 同上 |
| 17 | 切回 bank 0 | bank select | `0x76` (`REG_BANK_SEL`) | 直接写 | `0x00` | - | `0x00` | 恢复 bank 0 |
| 18 | accel ODR = 1000 Hz | 0 | `0x50` (`REG_ACCEL_CONFIG0`) | `WriteMask()` | `0x06` | `0x0F` | `0x06` | `ICM4X6XX_ODR_1000 -> ODR_1KHZ = 6`，和第 7 步合并后寄存器仍为 `0x06` |
| 19 | gyro ODR = 1000 Hz | 0 | `0x4F` (`REG_GYRO_CONFIG0`) | `WriteMask()` | `0x06` | `0x0F` | `0x06` | `ICM4X6XX_ODR_1000 -> ODR_1KHZ = 6`，和第 8 步合并后寄存器仍为 `0x06` |
| 20 | accel BW = ODR/5 | 0 | `0x52` (`REG_GYRO_ACCEL_CONFIG0`) | `WriteMask()` | `0x20` | `0xF0` | `0x20` | `BW_ODR_DIV_5 = 2`，写入 `2 << 4` |
| 21 | gyro BW = ODR/2 | 0 | `0x52` (`REG_GYRO_ACCEL_CONFIG0`) | `WriteMask()` | `0x00` | `0x0F` | `0x20` | `BW_ODR_DIV_2 = 0`，低四位清零，保留第 20 步的高四位 |
| 22 | 使能 FIFO 数据项 | 0 | `0x5F` (`REG_FIFO_CONFIG_1`) | `WriteMask()` | `0x07` | `0x1F` | `0x07` | 使能 accel(`0x01`) + gyro(`0x02`) + temp(`0x04`)，不使能 timestamp/fsync(`0x08`) 和 hires(`0x10`) |
| 23 | accel/gyro 上电 | 0 | `0x4E` (`REG_PWR_MGMT_0`) | `WriteMask()` | `0x0F` | `0x0F` | `0x0F` | accel 和 gyro 都进入 LNM，随后延时 `20 ms` |
| 24 | disable AFSR | 0 | `0x4D` | `WriteMask()` | `0x40` | `0xC0` | `0x40 \| (原值 & 0x3F)` | `0x4D` 在 bank 0 对应 `INTF_CONFIG1`；这一步只改 bit[7:6]，其余低 6 位保持原值 |

## 3. 最终可以直接记住的结果

对于 `DebugInit(false)`，最后真正落到芯片上的关键配置可以概括成下面这些：

| 序号 | 项目 | 最终结果 |
| --- | --- | --- |
| 1 | UI 接口 | SPI |
| 2 | RTC / CLKIN | 不开启 |
| 3 | Timestamp 到寄存器 | 不开启 |
| 4 | FIFO/传感器数据字节序 | 大端 |
| 5 | Accel FSR | `16g` |
| 6 | Gyro FSR | `2000 dps` |
| 7 | Accel ODR | `1000 Hz` |
| 8 | Gyro ODR | `1000 Hz` |
| 9 | Accel Filter Order | 三阶 |
| 10 | Gyro Filter Order | 三阶 |
| 11 | Accel Bandwidth | `ODR/5` |
| 12 | Gyro Bandwidth | `ODR/2` |
| 13 | FIFO 模式 | `STREAM` |
| 14 | FIFO 内容 | accel + gyro + temp |
| 15 | Accel/Gyro Power | 同时上电到 LNM |
| 16 | AUX pin | bank 2 下 `0x70~0x73` 全部写 `0x01` |

## 4. 对文档里“最终写回值”的一个提醒

- 本文表格中的“最终写回值”是按当前函数内部的前后顺序推出来的，适合你做代码梳理和调试对照。
- 因为 `WriteMask()` 不是盲写，而是“读改写”，如果芯片在调用 `DebugInit(false)` 之前某些寄存器已经被别处改过，那么未被 mask 覆盖的位会保留下来。
- 这也是为什么像 `0x4D` 这种寄存器，最稳妥的理解方式是同时看 `reg_value` 和 `mask`。
