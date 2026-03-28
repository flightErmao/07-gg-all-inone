# applications 文件夹架构梳理

## 1. 设计依据

本文档依据 `02-各方案选项备注.md` 第 3 章所列五项测试需求，结合当前 `applications/` 已有结构，
给出系统化的文件夹分层方案和新增建议。

整体链路固定为：

```
MCU -> SD 卡本地记录 -> 测试结束后导出 CSV -> PC 分析脚本
```

USB 串口定位为：命令下发 / 状态查看 / 采集控制，不承担主数据记录。

---

## 2. 当前结构（已有）

```
applications/
├── app/
│   └── main.c
├── sensor/
│   └── imu_reader_thread.cpp
├── storage/
│   ├── fatfs_sdcard_port.c
│   └── fatfs_sdcard_port.h
├── usb/
│   ├── usb_mode_manager.c
│   └── usb_mode_manager.h
└── SConscript
```

---

## 3. 目标结构（整理后 + 新增）

```
applications/
├── app/                          # 应用入口层
│   ├── main.c                    # 已有：系统入口，启动各子系统
│   └── app_init.c / .h           # 新增：统一初始化编排
│
├── cmd/                          # 命令解析层（新增）
│   ├── cmd_parser.c / .h         # USB 串口命令解析
│   └── cmd_handler.c / .h        # 命令分发与响应
│
├── sensor/                       # IMU 读取层（现有，补全）
│   ├── imu_reader_thread.cpp     # 已有：IMU FIFO 轮询主线程
│   ├── imu_config.h              # 新增：IMU 参数配置（ODR、量程、FIFO 深度）
│   ├── imu_data_types.h          # 新增：原始数据结构定义（帧、包格式）
│
├── storage/                      # 存储层（现有，扩展）
│   ├── fatfs_sdcard_port.c / .h  # 已有：FatFS 底层挂载与读写接口
│   ├── csv_writer.c / .h         # 新增：CSV 格式化写入（各测试共用）
│   └── file_naming.c / .h        # 新增：文件命名规则（测试类型 + 时间戳）
│
├── session/                      # 测试会话管理层（新增，核心）
│   ├── session_manager.c / .h    # 会话状态机：IDLE / RUNNING / STOPPING
│   ├── test_bias.c / .h          # 测试 1：零偏及零偏稳定性采集会话
│   ├── test_temp_drift.c / .h    # 测试 2：温漂采集会话
│   ├── test_arw.c / .h           # 测试 3：ARW / 噪声采集会话
│   ├── test_vibration.c / .h     # 测试 4：振动条件下输出连续性采集会话
│   └── test_shock.c / .h         # 测试 5：冲击后数据恢复采集会话
│
├── control/                      # 硬件控制层（新增）
│   ├── relay_ctrl.c / .h         # 继电器控制（测试 1 断电重上电使用）
│   └── led_status.c / .h         # LED 状态指示
│
├── usb/                          # USB 层（现有，扩展）
│   ├── usb_mode_manager.c / .h   # 已有：CDC / MSC 模式切换
│   └── usb_report.c / .h         # 新增：状态上报与预览数据输出
│
└── SConscript                    # 构建脚本（已有，按新增文件同步更新）
```

---

## 4. 各层职责详解

### 4.1 `app/` — 应用入口层

| 文件 | 职责 |
|------|------|
| `main.c` | 已有。LED 心跳、启动 `usb_mode_manager`；后续补充调用 `app_init` |
| `app_init.c/.h` | 新增。统一编排各子系统启动顺序：SD 挂载 → USB 初始化 → 命令监听 → 会话就绪 |

**原则：** `main.c` 只做最小入口，复杂初始化逻辑收入 `app_init`，便于测试和调试。

---

### 4.2 `cmd/` — 命令解析层（新增）

| 文件 | 职责 |
|------|------|
| `cmd_parser.c/.h` | 从 USB CDC 接收串口字符串，按协议解析为命令结构体 |
| `cmd_handler.c/.h` | 根据命令类型分发：调用 `session_manager` 的启停接口、查询状态、切换 USB 模式 |

**支持的命令示例：**
```
start bias         # 启动测试 1：零偏采集
start temp         # 启动测试 2：温漂采集
start arw          # 启动测试 3：ARW/噪声采集
start vibration    # 启动测试 4：振动连续性采集
start shock        # 启动测试 5：冲击恢复采集
stop               # 停止当前会话
status             # 查询当前会话状态
usb msc            # 切换为 MSC 模式（导出文件）
usb cdc            # 切换回 CDC 模式
```

**原则：** 命令解析与业务逻辑完全分离，`cmd_parser` 只负责字符串 → 结构体；`cmd_handler` 只负责路由，不含任何采集逻辑。

---

### 4.3 `sensor/` — IMU 读取层

| 文件 | 职责 |
|------|------|
| `imu_reader_thread.cpp` | 已有骨架。IMU FIFO 轮询线程，定时读取四颗 IMU 原始数据 |
| `imu_config.h` | 新增。集中管理 IMU 参数：ODR、加速度量程 `FS_acc`、陀螺量程、FIFO 深度、轮询周期 |
| `imu_data_types.h` | 新增。定义数据结构：原始六轴帧 `imu_raw_frame_t`、FIFO 批次包 `imu_fifo_batch_t` |

**数据结构示例（`imu_data_types.h`）：**
```c
typedef struct {
    uint32_t timestamp_ms;
    int16_t  acc_x, acc_y, acc_z;
    int16_t  gyro_x, gyro_y, gyro_z;
    int16_t  temp_raw;
} imu_raw_frame_t;

typedef struct {
    uint32_t poll_timestamp_ms;
    uint16_t pkt_cnt;              // 本次轮询读出的包数，对应 fifo_pkt_cnt
    imu_raw_frame_t frames[64];    // 最大 64 包，视 FIFO 深度调整
} imu_fifo_batch_t;
```

**原则：** `sensor/` 只负责读硬件、填数据结构，不做任何测试判断或文件写入。

---

### 4.4 `storage/` — 存储层

| 文件 | 职责 |
|------|------|
| `fatfs_sdcard_port.c/.h` | 已有。FatFS 挂载/卸载、底层追加行、原始文件读写等接口 |
| `csv_writer.c/.h` | 新增。各测试项 CSV 行格式化函数，输出符合分析脚本约定的表头和数据行 |
| `file_naming.c/.h` | 新增。生成文件路径，规则：`/测试类型/YYYYMMDD_HHMMSS.csv` |

**CSV 表头约定（按测试项）：**

| 测试 | CSV 表头 |
|------|----------|
| 测试 1 零偏 | `time,temp,A1_acc_x,A1_acc_y,A1_acc_z,A1_gyro_x,A1_gyro_y,A1_gyro_z,...` |
| 测试 2 温漂 | 同测试 1（含 `temp`） |
| 测试 3 ARW | `time,temp,A1_gyro_x,A1_gyro_y,A1_gyro_z,A2_gyro_x,...` |
| 测试 4 振动 | `time,fifo_pkt_cnt,A1_acc_z,A2_acc_z,B1_acc_z,B2_acc_z` |
| 测试 5 冲击 | `time,A1_acc_x,A1_acc_y,A1_acc_z,A1_gyro_x,A1_gyro_y,A1_gyro_z,...` |

**SD 卡目录结构：**
```
/
├── bias/
│   └── 20250101_120000.csv
├── temp/
│   └── 20250101_140000.csv
├── arw/
│   └── 20250101_160000.csv
├── vibration/
│   └── 20250101_180000.csv
└── shock/
    └── 20250101_200000.csv
```

**原则：** `csv_writer` 只做格式化，不持有文件句柄；底层 IO 仍通过 `fatfs_sdcard_port` 完成。

---

### 4.5 `session/` — 测试会话管理层（新增核心层）

这是五项测试的核心业务逻辑所在，负责采集控制、状态管理和向 `storage` 写入数据。

#### 4.5.1 `session_manager.c/.h` — 会话状态机

```
IDLE  -->  RUNNING  -->  STOPPING  -->  IDLE
            |                ^
            +--- stop 命令 --+
```

- 维护当前活跃的测试类型和状态
- 提供 `session_start(test_type)` / `session_stop()` / `session_get_status()` 接口
- 保证同一时刻只有一个测试会话运行

#### 4.5.2 `test_bias.c/.h` — 测试 1：零偏及零偏稳定性

- 采集 `1 min` 连续静置数据
- 记录列：`time, temp, A1_acc_x ~ B2_gyro_z`（全六轴）
- 支持通过 `relay_ctrl` 控制 IMU 断电 / 重上电，实现多轮上电重复测试
- 每轮上电生成独立 CSV 文件，文件名含轮次编号

#### 4.5.3 `test_temp_drift.c/.h` — 测试 2：温漂

- 持续采集，等待外部温箱温度稳定
- 记录列：同测试 1（`time, temp, 全六轴`）
- 支持标注当前温度点（通过 USB 命令传入：`mark temp 25`）
- 按温度升降程序连续记录，后续由 PC 脚本按温度点分段统计

#### 4.5.4 `test_arw.c/.h` — 测试 3：ARW / 噪声

- 连续静置采集，推荐 `1 h`（最低 `30 min`）
- 记录列：`time, temp, A1_gyro_x ~ B2_gyro_z`
- 采集期间不做任何计算，原始数据直写 SD
- 可通过 USB 查询已记录时长

#### 4.5.5 `test_vibration.c/.h` — 测试 4：振动输出连续性

- IMU 工作在 FIFO 模式，MCU 按 `500 ms` 轮询
- 每次轮询写一行：`time, fifo_pkt_cnt, A1_acc_z, A2_acc_z, B1_acc_z, B2_acc_z`
- `fifo_pkt_cnt` 即本次轮询读出的包数，对应分析脚本中的 `fifo_abnormal_ratio_high` 计算输入
- 不在 MCU 端做高幅判断，全部由 PC 分析脚本离线识别

#### 4.5.6 `test_shock.c/.h` — 测试 5：冲击后数据恢复

- 连续记录全六轴原始数据
- 记录列：`time, A1_acc_x ~ B2_gyro_z`
- MCU 不做冲击检测，只保证持续无中断记录
- PC 分析脚本负责识别冲击时刻 `t_shock`，计算 `recovery_time`、`post_shock_shift` 等指标

**原则：** 每个 `test_xxx` 模块只负责本测试项的采集逻辑，通过 `csv_writer` 和 `fatfs_sdcard_port` 写入，不直接操作硬件。

---

### 4.6 `control/` — 硬件控制层（新增）

| 文件 | 职责 |
|------|------|
| `relay_ctrl.c/.h` | 继电器控制：`relay_power_off()` / `relay_power_on()`，配置 GPIO，含续流保护延时 |
| `led_status.c/.h` | LED 状态指示：IDLE（慢闪）/ RUNNING（常亮）/ ERROR（快闪） |

**原则：** 硬件控制逻辑集中管理，`session/` 层通过接口调用，不直接操作 GPIO。

---

### 4.7 `usb/` — USB 层

| 文件 | 职责 |
|------|------|
| `usb_mode_manager.c/.h` | 已有。CDC / MSC 模式切换，测试结束后切 MSC 可直接拷贝 CSV 文件 |
| `usb_report.c/.h` | 新增。向 CDC 串口上报：当前会话状态、已采集行数、简化预览数据（可选） |

---

## 5. 层间依赖关系

```
┌────────────────────────────────────────────────┐
│                    app/                         │  入口层
│         main.c  ←→  app_init.c                 │
└───────────────────┬────────────────────────────┘
                    │ 启动
┌───────────────────▼────────────────────────────┐
│                    cmd/                         │  命令层
│       cmd_parser  →  cmd_handler               │
└───────────────────┬────────────────────────────┘
                    │ 调用
┌───────────────────▼────────────────────────────┐
│                  session/                       │  业务逻辑层（核心）
│  session_manager → test_bias / temp / arw /    │
│                    vibration / shock            │
└──────┬───────────────────────┬─────────────────┘
       │ 读数据                 │ 写文件
┌──────▼──────┐        ┌───────▼──────────┐
│   sensor/   │        │    storage/      │
│ imu_reader  │        │ fatfs + csv +    │
│ imu_config  │        │ file_naming      │
└─────────────┘        └──────────────────┘
       │ 硬件控制
┌──────▼──────┐
│  control/   │
│ relay_ctrl  │
│ led_status  │
└─────────────┘
       │ USB 交互
┌──────▼──────┐
│    usb/     │
│ mode_mgr    │
│ usb_report  │
└─────────────┘
```

**单向依赖原则：**
- 上层依赖下层，下层不感知上层
- `session/` 是唯一既依赖 `sensor/` 又依赖 `storage/` 的层
- `control/` 和 `usb/` 由 `session/` 或 `cmd/` 调用，不主动访问其他层

---

## 6. 各测试项与文件夹对应总览

| 测试项 | 主要涉及模块 | SD 卡目录 |
|--------|-------------|-----------|
| 测试 1：零偏稳定性 | `test_bias` + `relay_ctrl` + `csv_writer` | `/bias/` |
| 测试 2：温漂 | `test_temp_drift` + `csv_writer` | `/temp/` |
| 测试 3：ARW / 噪声 | `test_arw` + `csv_writer` | `/arw/` |
| 测试 4：振动连续性 | `test_vibration` + `csv_writer` | `/vibration/` |
| 测试 5：冲击恢复 | `test_shock` + `csv_writer` | `/shock/` |

---

## 7. 新增文件清单（需要创建）

按优先级排列：

**P1 — 采集运行必需：**
- `session/session_manager.c/.h`
- `session/test_vibration.c/.h`（测试 4，振动项目）
- `session/test_shock.c/.h`（测试 5，冲击项目）
- `storage/csv_writer.c/.h`
- `storage/file_naming.c/.h`
- `sensor/imu_config.h`
- `sensor/imu_data_types.h`

**P2 — 控制和观测：**
- `cmd/cmd_parser.c/.h`
- `cmd/cmd_handler.c/.h`
- `usb/usb_report.c/.h`
- `control/led_status.c/.h`

**P3 — 测试 1/2/3 专用会话：**
- `session/test_bias.c/.h`
- `session/test_temp_drift.c/.h`
- `session/test_arw.c/.h`
- `control/relay_ctrl.c/.h`（测试 1 使用）

**已有文件保持不动，在现有基础上扩展：**
- `app/main.c`
- `sensor/imu_reader_thread.cpp`
- `storage/fatfs_sdcard_port.c/.h`
- `usb/usb_mode_manager.c/.h`
- `SConscript`
