# 08-imu-usb Script

这个目录提供一组与当前 `08-imu-usb` 固件对齐的 Python 脚本，供上位机通过 CDC 串口直接调用。

当前固件已经落实以下协议要求：

- 上位机发送的命令名和 MCU 实际支持的 shell 命令一致
- MCU 收到命令后，会先返回一行 `ACK ...`
- 命令处理完成后，会返回一行 `RESULT ...`
- 状态查询类命令会额外返回一行 `STATUS ...`

这样上位机可以明确区分：

- 命令是否已经送达 MCU
- 命令是否执行成功
- 命令执行后的关键状态

## 当前支持的固件命令

### `status`

用途：查询当前 USB/文件系统/日志状态。

示例：

```text
status
```

典型返回：

```text
ACK cmd=status received
STATUS mode=cdc fs=1 usb_ready=1 logger=1 log_fd=1 target=cdc last_result=0 last_stage=23
RESULT cmd=status status=ok
```

### `enter_msc`

用途：切换到 `CDC + MSC` 导出模式。

示例：

```text
enter_msc
```

典型返回：

```text
ACK cmd=enter_msc received current=cdc target=cdc+msc
RESULT cmd=enter_msc status=accepted action=reboot_to_cdc_msc
```

说明：

- 返回 `accepted` 代表 MCU 已收到命令并准备重启切换
- 之后设备会重新枚举，CDC 串口会短暂断开
- 新模式下 PC 会看到新的 CDC 串口和 U 盘

### `enter_cdc`

用途：切回 `CDC-only` 模式。

示例：

```text
enter_cdc
```

典型返回：

```text
ACK cmd=enter_cdc received current=cdc+msc target=cdc
RESULT cmd=enter_cdc status=accepted action=reboot_to_cdc
```

## 依赖

```powershell
pip install pyserial
```

## GUI 启动

```powershell
python .\main.py
```

GUI 当前支持：

- 刷新/连接 CDC 串口
- 查询 `status`，并显示 `ACK/STATUS/RESULT`
- 发送 `enter_msc` 切到 `CDC + MSC`
- 发送 `enter_cdc` 切回 `CDC-only`
- 自动等待 MSC 盘符，同时尽量保留 CDC 命令通道
- 自动导出 `IMU_LOG.CSV`
- 通过 J-Link 复位回 CDC

## CLI 常用命令

列出串口：

```powershell
py -3 .\imu_usb_tool.py ports
```

查询固件状态：

```powershell
py -3 .\imu_usb_tool.py status --port COM78
```

切到 CDC + MSC：

```powershell
py -3 .\imu_usb_tool.py enter-msc --port COM78
```

切回 CDC-only：

```powershell
py -3 .\imu_usb_tool.py enter-cdc --port COM78
```

切到 CDC + MSC 并导出日志：

```powershell
py -3 .\imu_usb_tool.py export-log --port COM78 --reset-to-cdc
```

仅通过 J-Link 复位回 CDC：

```powershell
py -3 .\imu_usb_tool.py reset-to-cdc --port COM78
```

零偏多次上电批量分析：

```powershell
py -3 .\imu_usb_tool.py bias-batch `
  ..\data\01_bias\007
```

也可以直接传入多个 BIN：

```powershell
py -3 .\imu_usb_tool.py bias-batch `
  ..\data\01_bias\007\001.BIN `
  ..\data\01_bias\007\002.BIN `
  ..\data\01_bias\007\003.BIN `
  ..\data\01_bias\007\004.BIN `
  ..\data\01_bias\007\005.BIN `
  ..\data\01_bias\007\006.BIN
```

说明：

- 每个 BIN 视为一次重新上电后的静置采集段，适配 `document/02-各方案选项备注.md` 中零偏测试要求。
- 脚本会先给每个 BIN 输出一次单轮分析，再在顶层生成 `*_bias_runs.csv`、`*_bias_aggregate.csv`、`*_bias_batch_analysis.md` 和 6 张上电轮次趋势图。
- `*_bias_runs.csv` 是每轮上电的段均值/段内标准差/段内极差；`*_bias_aggregate.csv` 是按多次上电段均值计算的最终均值、样本标准差和上电间偏差。
- GUI 中选择 `测试项目 1：零偏及零偏稳定性` 后点击“选择 BIN 分析”支持一次多选 BIN，输出同一套批量结果。

振动 CSV 额外积分分析：

```powershell
py -3 .\vibration_accel_integrate_plot.py `
  ..\data\04_vibration\161_20260410_150946_waveform\161_42688_A.csv `
  --dt 0.01
```

说明：

- 这个脚本会优先自动识别 `*_accel_z_g` 列，先自动识别前静止段，并用这段静止数据估计 bias，再做 `x[k] - bias` 去偏置，最后按离散公式 `y[k] = y[k-1] + (x[k] - bias) * dt` 做积分。
- 输出 CSV 会同时给出原始 `g`、去偏置后的 `g`、积分后的 `g*s`，以及换算后的 `m/s`。
- 输出文件默认放在输入 CSV 同目录下，分别是独立的 `.csv` 和 `.png`。

振动目录级静止段 bias 去除 + 6 轴批量积分分析：

```powershell
py -3 .\vibration_static_bias_batch_report.py `
  ..\data\04_vibration\161_20260410_150946_waveform `
  --dt 0.01
```

说明：

- 这个脚本会先用每颗 IMU 的 `accel_z_g` 自动识别前静止段，再用这段静止数据给该 IMU 的 `3` 个加速度轴和 `3` 个陀螺轴分别估计 bias。
- 加速度轴会输出去偏置后的积分速度 `m/s`，陀螺轴会输出去偏置后的积分角度 `deg`。
- 会为每颗 IMU 生成一份独立 `6` 轴 CSV 和一张总览图，并把结果章节补充进同目录下的 `*_vibe_analysis.md`。

冲击恢复批量分析：

```powershell
py -3 .\run_shock_bin_batch_analysis.py `
  ..\data\98-raw-bin\04-敲击测试\01-第一个轻敲-第二个是3个敲击-由轻到重
```

说明：

- 这个脚本会对目录下每个 `BIN` 单独创建一个输出文件夹，并按 `document/02-各方案选项备注.md` 中 `3.5` 的方案生成冲击恢复报告。
- 指标包括 `peak_ratio_shock`、`dropout_flag_shock`、`t_first_valid`、`recovery_time`、`post_shock_shift`。
- GUI 里选择 `测试项目 5：冲击后数据恢复` 再点“选择 BIN 分析”时，也会走同一套解析逻辑并输出对应报告。

## 推荐上位机测试流程

### 1. 确认命令链路正常

```powershell
py -3 .\imu_usb_tool.py status --port COM78
```

预期：

- 收到 `ACK cmd=status received`
- 收到 `RESULT cmd=status status=ok`

### 2. 测试 MSC 导出

```powershell
py -3 .\imu_usb_tool.py enter-msc --port COM78
```

预期：

- 先收到 `ACK cmd=enter_msc ...`
- 再收到 `RESULT cmd=enter_msc status=accepted ...`
- 设备重枚举后出现新的 CDC 和 U 盘
- U 盘中可看到 `IMU_LOG.CSV`

### 3. 测试恢复 CDC

```powershell
py -3 .\imu_usb_tool.py enter-cdc --port COMxx
```

预期：

- 收到 `ACK cmd=enter_cdc ...`
- 收到 `RESULT cmd=enter_cdc status=accepted ...`
- 设备重新枚举回 CDC-only
