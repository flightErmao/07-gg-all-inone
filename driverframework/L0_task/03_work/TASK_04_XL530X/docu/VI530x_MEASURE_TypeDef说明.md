# VI530x_MEASURE_TypeDef 字段说明

本文基于 `VI530x_API.c` 中的 `VI530x_Get_Measure_Data()` 实现，梳理 VI530x 测距结果结构体 `VI530x_MEASURE_TypeDef` 各成员的含义及其计算流程。

## 原始数据来源

- 测距完成后，芯片通过寄存器 `0x0C`（Scratch Pad）返回 32 字节数据帧。
- SDK 每次中断到来时读取该数据帧，并按顺序解析 TOF、峰值、背景噪声、积分次数等信息。
- 多回波（共 3 组）会返回 3 组 TOF/Peak/AC 数据，SDK 根据反射率筛选出最可信的一组作为 `raw_tof`。

代码参考：

```280:428:driverframework/L0_task/03_work/TASK_04_XL530X/src/VI530x_API.c
VI530x_Status VI530x_Get_Measure_Data(VI530x_MEASURE_TypeDef *result, uint8_t wait_mode)
{
    ...
    ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);
    memcpy(&tof[0], &data_buff[1], 2);
    ...
    reflectivity[i] = (a * b) / 100000;
    ...
    confidence = (321000 * (float)raw_peak / intecounts - lower_co * raw_ac) * 100 /
                 (upper_co * raw_ac + 1 - lower_co * raw_ac);
    bias = VI530x_Calculate_Pileup_Bias_V40_LR(...);
    result->correction_tof = raw_tof + (int16_t)bias - VI530x_Cali_Data.VI530x_Cali_Offset;
    ...
}
```

## 字段速览

| 成员 | 单位/范围 | 数据来源 | 计算或解析方式 | 含义说明 |
| --- | --- | --- | --- | --- |
| `correction_tof` | mm，带符号 16 位 | 选定回波的 TOF + pile-up 偏差 – 标定偏移 | 见下文公式 | 输出距离（经 pile-up 校正、offset 去偏） |
| `confidence` | 0～100 % | 选定回波的峰值/AC/积分次数 | 经验公式并限幅 | 可信度评分，越高越可靠 |
| `intecounts` | 24 bit，无单位 | 原始帧 `data_buff[26:28]` | 取 24 位有效值 | 本次测距的积分次数（曝光时间累积值） |
| `peak` | 24 bit 左移 8 位 | 原始帧中对应回波的 Peak | 取 24 位有效值后左移 8 位 | 目标回波峰值强度（脉冲累积计数） |
| `noise` | 24 bit，无单位 | 原始帧 `data_buff[29:31]` | 取 24 位有效值 | 背景噪声累积计数，用于可信度与 pile-up 校正 |
| `xtalk_count` | 目前恒为 0 | 占位字段 | SDK 当前未解析 | 预留，用于表示 Xtalk 触发次数/计数 |
| `ts` | 原始字节 | `data_buff[25]` | 直接保存 | 芯片返回的温度/状态字节，SDK 暂未解码 |

## 详细说明

### `correction_tof`

- 首先依据反射率阈值 `re_th = 3`，在 3 组回波中选择其中一组 `raw_tof`：
  - 反射率计算：`reflectivity = ((peak - peak_th) / intecounts) * (tof + offset1)^2 / 100000`，其中 `peak_th = 0`、`offset1 = 100`。
  - 从 0、1、2 号回波依次判断，一旦反射率不低于阈值即采纳该组数据，否则输出 0。
- 基于选定回波，调用 `VI530x_Calculate_Pileup_Bias_V40_LR()` 估算 pile-up 偏差：
  - 该函数根据 `VI530x_Cali_Data.VI530X_MA_Sum`、`raw_peak`、`noise`、`intecounts` 计算插值偏差值。
- 将计算得到的偏差加到 `raw_tof` 上，再减去出厂或用户标定得到的 `VI530x_Cali_Data.VI530x_Cali_Offset`（单位 mm），得到最终的 `correction_tof`。
- 意义：这是 SDK 输出给上层的距离值，包含 pile-up 校正和 offset 去偏。

相关算法：

```121:157:driverframework/L0_task/03_work/TASK_04_XL530X/src/VI530x_Algorithm.c
float VI530x_Calculate_Pileup_Bias_V40_LR(uint16_t vi530x_ma_sum,uint32_t peak, uint32_t noise,uint32_t integral_times)
{
    ...
    if (peak > noise * vi530x_ma_sum)
    {
        peak_tmp = (peak - noise * vi530x_ma_sum) * 16 / integral_times;
    }
    ... // 根据查表插值返回 pile-up 偏差
}
```

### `confidence`

- 使用经验公式评估可信度：
  - `confidence = (321000 * raw_peak / intecounts - lower_co * raw_ac) * 100 / (upper_co * raw_ac + 1 - lower_co * raw_ac)`，其中 `lower_co = 0.07`，`upper_co = 0.13`。
  - 结果被限制在 0～100 之间。
- `raw_ac` 来自原始帧 AC 字段（Ambient Count，环境光计数），与 `raw_peak` 搭配计算信噪比。
- 意义：值越高代表此次测距更可信；若计算结果不为 100，标定流程会认为采样异常。

### `intecounts`

- 从 `data_buff[26:29]` 拼接 32 位再取低 24 位有效数据。
- 代表本次测距的积分次数（Integration Counts），与曝光时间相关，不过 SDK 未换算成时间单位。
- 在 `confidence` 计算和 pile-up 校正中均作为归一化因子。

### `peak`

- 对应选中回波的峰值数据，先取 24 位有效值再左移 8 位，与芯片内部 bit 对齐方式有关。
- 反映回波能量大小，直接影响反射率、可信度及 pile-up 校正。

### `noise`

- 解析自 `data_buff[29:31]`，同样只保留 24 位有效值。
- 表征环境噪声，既参与 pile-up 校正，也决定 confidence 上下阈值。

### `xtalk_count`

- 字段已在结构体中预留，但当前 SDK 读取流程并未从数据帧中解析对应值，默认输出 0。
- 建议后续若芯片固件提供该信息，可在读取 32 字节帧时补充解析逻辑。

### `ts`

- 保存自 `data_buff[25]` 的原始字节。
- 官方手册通常定义该字节为 Temperature State 或 Time Stamp。当前 SDK 仅缓存，未参与任何算法，可按需在上层解码。

## 使用建议

- 在上层应用中记录 `intecounts`、`peak`、`noise` 可协助分析异常测距（例如环境光过强）。
- 若需要更精准的温度补偿，可结合 `ts` 与 `VI530x_Set_Sys_Temperature_Enable()` 控制逻辑自行扩展。
- 如果计划启用 Xtalk 统计，应在固件返回帧中确认相关字段位置并补齐解析，再更新 `xtalk_count`。
