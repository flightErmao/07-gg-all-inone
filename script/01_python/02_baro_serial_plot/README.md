# BARO 串口实时曲线工具

该工具用于读取 `02_baro_show` 固件的串口输出，并实时绘制 SPA06-003 的气压和温度数据。

支持的固件日志格式：

```text
BARO[a06_03_1]: P=101325.00Pa T=25.31deg timestamp=1234ms
```

## 启动

直接双击：

```text
run_baro_plot.bat
```

也可以在 PowerShell 中运行：

```powershell
cd D:\08-rtt\04-new-create\07-gg-all-inone\script\01_python\02_baro_serial_plot
py -3.12 baro_serial_plot.py
```

本机的 Python 3.12 已具备运行所需的 `tkinter`、`pyserial` 和 `matplotlib`。如果换到其他电脑，先安装依赖：

```powershell
py -3.12 -m pip install -r requirements.txt
```

## 使用方法

1. 给开发板供电并连接输出日志的串口。
2. 启动 GUI，点击“刷新”。
3. 选择开发板对应的 COM 口，波特率默认 `115200`。
4. 点击“连接”。
5. 固件输出 BARO 数据后，气压和温度曲线会自动刷新。
6. 如需记录数据，点击“开始保存 CSV”。

如果串口能连接但传感器没有接好，日志区域仍会显示固件输出，状态栏会提示“未检测到 BARO”。

## 解析器自测

```powershell
py -3.12 baro_serial_plot.py --self-test
```
