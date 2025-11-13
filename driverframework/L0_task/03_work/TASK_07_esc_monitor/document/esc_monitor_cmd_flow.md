# `cmdEscMonitor` 电压/电流采集流程

## 概述

- 负责在 ESC 上电流程中操控 `PB.14`（DroneOnOff）并采样 `PB.15`（PhoneOnV1P8），同时驱动 INA226 完成两阶段电压、电流采集。
- 定时器仅以 10 ms 周期触发事件，由后台工作线程完成实际 INA226 读取与 100 组样本累计，产出均值/中值结果。
- 采样结果写入 `esc_monitor_detection_`，随后经 Modbus 保持寄存器同步给上位机；上位机基于寄存器值执行阈值判定。

## 流程图

```mermaid
flowchart TD
    A[cmdEscMonitor 调用] --> B{IO 初始化}
    B -->|失败| X[记录错误并返回]
    B -->|成功| C{INA226 初始化}
    C -->|失败| X
    C -->|成功| D[重置检测数组<br/>esc_monitor_reset_detection_results]
    D --> E[重置采样器<br/>esc_monitor_ina226_reset]
    E --> F[启动阶段1采样]
    F --> G[延时 1200 ms 等待样本]
    G --> H[PB.14 ↓（Drone 断电）]
    H --> I[采集 PB.15 电平 1000 ms<br/>state_1 = esc_monitor_phone_v1p8]
    I --> J[写 STATE1_HIGH 标志]
    J --> K[延时 500 ms]
    K --> L[PB.14 ↑（Drone 上电）]
    L --> M[采集 PB.15 电平 1200 ms<br/>state_2 = esc_monitor_phone_v1p8]
    M --> N[写 STATE2_LOW 标志]
    N --> O[延时 1000 ms]
    O --> P[启动阶段2采样]
    P --> Q[采集 PB.15 电平 1200 ms<br/>state_3 = esc_monitor_phone_v1p8]
    Q --> R[写 STATE3_HIGH 标志]
    R --> S{读取阶段1结果}
    S -->|有效| S1[写 Stage1 Bus_V×10<br/>写 Stage1 Current×1000]
    S -->|无效| S0[写 0]
    S1 --> T
    S0 --> T
    T{读取阶段2结果} -->|有效| T1[写 Stage2 Bus_V×10<br/>写 Stage2 Current×1000]
    T -->|无效| T0[写 0]
    T1 --> U[FLOW_DONE = 1]
    T0 --> U
    U --> V[调用 cmdEscMonResult 打印]
    V --> W[流程结束]

    subgraph INA226 后台采样
        Y1[10 ms 定时器回调] --> Y2[发送 SAMPLE 事件]
        Y2 --> Y3[工作线程等待事件]
        Y3 --> Y4[读取 INA226 电压/电流]
        Y4 --> Y5[写入当前阶段缓存]
        Y5 -->|样本计数=100| Y6[阶段结束→result.valid=RT_TRUE]
    end
```

## 关键时序

- 阶段1：启动采样后先等待 1200 ms，以便线程积累足够样本，再进行 PB.15 电平统计与标志登记。
- 阶段2：PB.14 再次拉高后，先延时 1000 ms 以确保阶段1收尾，再启动第二阶段采样并同步统计电平。
- 每个阶段结束后，如果采样有效，则将
  - `bus_voltage_median × 10` 写入 `ESC_MONITOR_DETECTION_INDEX_STAGE{1,2}_BUS_VOLTAGE_X10`
  - `shunt_current_median × 1000` 写入 `ESC_MONITOR_DETECTION_INDEX_STAGE{1,2}_CURRENT_MA`
    若结果无效则填 0。
- 最后置位 `FLOW_DONE` 并打印概览，Modbus 线程会周期性读取 `esc_monitor_detection_` 写入保持寄存器。

## 上位机解析

- 电压寄存器除以 10 得到伏特，电流寄存器除以 1000 得到安培。
- `STATE1/2/3` 与 `FLOW_DONE` 为布尔位（0/1），可用来判定流程是否顺利执行以及 PB.15 的电平状态。

![1763022480705](images/esc_monitor_cmd_flow/1763022480705.png)
