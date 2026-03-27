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

### `fake_imu_sd_test [lines] [overwrite]`

用途：在 SD 卡上生成假 IMU 数据文件，方便上位机验证 SD 卡写入、MSC 导出和文件读取流程。

参数说明：

- `lines`：写入的假数据行数，默认 `64`
- `overwrite`：是否覆盖写入
- `1` 表示覆盖 `IMU_TEST.CSV`
- `0` 表示追加到 `IMU_TEST.CSV`

示例：

```text
fake_imu_sd_test 128 1
```

典型返回：

```text
ACK cmd=fake_imu_sd_test received lines=128 overwrite=1
RESULT cmd=fake_imu_sd_test status=ok file=IMU_TEST.CSV lines=128 overwrite=1
```

失败示例：

```text
RESULT cmd=fake_imu_sd_test status=error reason=mount_failed
```

或：

```text
RESULT cmd=fake_imu_sd_test status=error reason=write_failed written=32 file=IMU_TEST.CSV
```

说明：

- 该命令会向 SD 卡写入 `IMU_TEST.CSV`
- 每行格式与假 IMU 日志一致：`seq,tick,ax,ay,az`
- 推荐先执行这个命令，再切到 `enter_msc`，由上位机读取 U 盘中的 `IMU_TEST.CSV`

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
- 发送 `fake_imu_sd_test`，执行假数据写 SD 测试
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

向 SD 卡写入 128 行假 IMU 数据，并覆盖旧测试文件：

```powershell
py -3 .\imu_usb_tool.py fake-sd-test --port COM78 --lines 128 --overwrite
```

向 SD 卡追加 256 行假 IMU 数据：

```powershell
py -3 .\imu_usb_tool.py fake-sd-test --port COM78 --lines 256
```

切到 CDC + MSC 并导出日志：

```powershell
py -3 .\imu_usb_tool.py export-log --port COM78 --reset-to-cdc
```

仅通过 J-Link 复位回 CDC：

```powershell
py -3 .\imu_usb_tool.py reset-to-cdc --port COM78
```

## 推荐上位机测试流程

### 1. 确认命令链路正常

```powershell
py -3 .\imu_usb_tool.py status --port COM78
```

预期：

- 收到 `ACK cmd=status received`
- 收到 `RESULT cmd=status status=ok`

### 2. 测试 SD 卡写入

```powershell
py -3 .\imu_usb_tool.py fake-sd-test --port COM78 --lines 128 --overwrite
```

预期：

- 收到 `ACK cmd=fake_imu_sd_test ...`
- 收到 `RESULT cmd=fake_imu_sd_test status=ok ...`

### 3. 测试 MSC 导出

```powershell
py -3 .\imu_usb_tool.py enter-msc --port COM78
```

预期：

- 先收到 `ACK cmd=enter_msc ...`
- 再收到 `RESULT cmd=enter_msc status=accepted ...`
- 设备重枚举后出现新的 CDC 和 U 盘
- U 盘中可看到 `IMU_LOG.CSV` 与 `IMU_TEST.CSV`

### 4. 测试读回

上位机直接读取 MSC 盘符中的 `IMU_TEST.CSV`，检查：

- 文件是否存在
- 行数是否符合预期
- 内容是否为 `seq,tick,ax,ay,az` 五列

### 5. 测试恢复 CDC

```powershell
py -3 .\imu_usb_tool.py enter-cdc --port COMxx
```

预期：

- 收到 `ACK cmd=enter_cdc ...`
- 收到 `RESULT cmd=enter_cdc status=accepted ...`
- 设备重新枚举回 CDC-only
