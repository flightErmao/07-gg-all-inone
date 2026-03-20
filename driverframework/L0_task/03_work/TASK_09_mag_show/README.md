# TASK_09_mag_show

## 目标

本任务用于评估：

- 当磁力计与 SD 卡之间距离固定时
- 在测试全过程中设备姿态、线束状态、周围磁环境都保持不变
- 唯一变化条件只有 `SD 是否工作`

此时 SD 工作会让磁力计输出的航向角最多偏移多少度。

这里评估的是“相对偏移量”，不是“绝对航向精度”。

## 本次新增内容

本目录已新增以下能力：

- MCU 端新增 `mag_sd_*` shell 命令
- MCU 端可将未校准的磁力计原始数据通过 `uart1` 按固定文本协议输出
- `script/mag_sd_eval_tool.py` 提供双串口上位机测试工具
- 可记录磁力计与 SD 卡距离
- 可通过按钮控制开始测试、结束测试、导出评估结果
- 导出原始数据、事件点、每轮评估结果、Markdown 报告、偏移曲线图

## 串口分工

测试使用两个串口：

- `shell串口`
  - 用于给 MCU 发送 shell 命令
  - 比如通过 USB VCOM / 调试串口进入 msh
- `data串口`
  - 用于接收 MCU 在 `uart1` 上输出的原始磁力计数据
  - 上位机脚本连接这个口并实时采集

默认配置：

- 评估数据输出设备：`uart1`
- 评估数据波特率：`115200`
- 输出周期：`20ms`

对应 Kconfig：

- `TASK_MAG_SD_EVAL_UART_NAME`
- `TASK_MAG_SD_EVAL_UART_BAUD`
- `TASK_MAG_SD_EVAL_STREAM_PERIOD_MS`

## MCU 端命令

新增 shell 命令如下：

- `mag_sd_stream_start`
  - 开启评估数据流
  - MCU 开始向 `uart1` 输出原始磁力计数据
- `mag_sd_stream_stop`
  - 关闭评估数据流
- `mag_sd_test_start`
  - 标记测试开始
  - 表示从这一刻起进入“SD 工作影响段”
- `mag_sd_test_stop`
  - 标记测试结束
- `mag_sd_status`
  - 查看评估输出串口、波特率、周期、当前状态

## 数据协议

MCU 在 `uart1` 输出的是纯文本 CSV 风格协议，便于 Python 脚本直接解析。

### 原始磁力计数据

```text
MAG_RAW,timestamp_ms,x_raw,y_raw,z_raw
```

示例：

```text
MAG_RAW,123456,102.000,-53.000,287.000
```

说明：

- 这里的 `x_raw/y_raw/z_raw` 是未校准原始值
- 当前用于相对偏移评估，不做硬铁/软铁校准

### 事件标记

```text
MAG_EVENT,timestamp_ms,event_name,detail
```

示例：

```text
MAG_EVENT,122000,STREAM_START,uart=uart1,baud=115200,period_ms=20
MAG_EVENT,125500,TEST_START
MAG_EVENT,130800,TEST_STOP
MAG_EVENT,131000,STREAM_STOP
```

## 上位机脚本

脚本路径：

- [script/mag_sd_eval_tool.py](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/script/mag_sd_eval_tool.py)

依赖安装：

```bash
py -3 -m pip install -r script/requirements.txt
```

启动：

```bash
py -3 script/mag_sd_eval_tool.py
```

## 推荐测试流程

1. 确认硬件固定
   - 固定磁力计与 SD 卡距离
   - 固定姿态
   - 固定线束位置
   - 保证周围磁环境不变化

2. 打开上位机工具
   - 选择 `shell串口`
   - 选择 `data串口`
   - 填写距离，例如 `5mm`、`10mm`、`15mm`
   - 如有需要填写备注

3. 点击“连接”
   - 工具会自动发送 `mag_sd_stream_start`
   - MCU 开始在 `uart1` 输出 `MAG_RAW`

4. 保持设备静止 3 秒以上
   - 这段数据作为“基线段”
   - 即 `SD 未工作` 或影响尚未开始时的参考磁场向量

5. 点击“开始测试”
   - 工具发送 `mag_sd_test_start`
   - 从这一刻起开始记录完整测试段
   - 阶段1：先保持 `SD 不工作`
   - 阶段2：再让 `SD 开始工作`
   - 阶段3：最后让 `SD 停止工作`
   - 测试期间不要改变设备姿态和外界条件

6. 点击“结束测试”
   - 工具发送 `mag_sd_test_stop`
   - 表示本轮三阶段测试结束

7. 点击“导出评估结果”
   - 工具会导出原始数据、事件、结果图和报告

8. 更换距离后重复测试
   - 建议一个距离导出一个结果目录
   - 最后对比不同距离下的最大航向偏移

## 评估方法

### 为什么未校准原始数据也可以用于本测试

这个测试的核心不是求“真实地磁航向”，而是求：

- 同一块板子
- 同一姿态
- 同一环境
- 同一原始磁力计
- 唯一变量只有 `SD 是否工作`

在这个前提下，磁力计的固定零偏、固定比例误差虽然存在，但它们在同一轮测试里基本不变，因此可以用于比较“SD 工作前后”的相对变化。

也就是说，本方法可以评估：

- `SD 工作引入了多大的附加偏转`

但不能直接代表：

- `设备绝对航向是否准确`
- `经过校准后真实航向误差是多少`

### 脚本的计算逻辑

每轮测试按下面方式评估：

1. `TEST_START` 前最近 3 秒的数据作为基线段
2. 对基线段的 `X/Y/Z` 原始值求平均
3. 对 `TEST_START ~ TEST_STOP` 之间的整段数据，自动识别三个阶段：
   - 阶段1：SD 未工作
   - 阶段2：SD 工作中
   - 阶段3：SD 停止后恢复
4. 以基线段的 `X/Y` 平均向量作为整轮参考航向向量
5. 以阶段1的 `X/Y` 平均向量作为三阶段对比参考
6. 分别统计三个阶段的：
   - 各轴均值、方差
   - 磁场强度均值、方差
   - 航向角均值、方差
   - 相对阶段1的航向偏移
7. 重点输出：
   - SD 启动后相对启动前的航向变化
   - SD 关闭后相对工作中的恢复变化
   - SD 关闭后相对启动前的残余偏移

核心上是在算：

- `heading_offset = angle(current_xy, baseline_xy)`

这样做的优点：

- 不怕 0~360 度跳变
- 不依赖绝对校准参数
- 适合当前“唯一变量控制”为 SD 工作状态的测试目标

### SD 干扰进入/退出识别方式

当脚本尝试自动拆分三阶段时，识别逻辑如下：

1. 在 `TEST_START ~ TEST_STOP` 内先取最前面一小段样本作为稳定种子窗口
2. 计算该窗口的平均参考向量 `ref_x/ref_y/ref_z`
3. 对每个测试样本计算磁场变化量：
   - `delta = sqrt((x-ref_x)^2 + (y-ref_y)^2 + (z-ref_z)^2)`
4. 对 `delta` 做滑动平均，降低单点噪声影响
5. 根据稳定种子窗口的均值、方差和 95 分位，构造进入阈值 `high_threshold`
6. 当平滑后的 `delta` 连续多点高于 `high_threshold` 时，判定 `SD 干扰进入`
7. 再根据稳定段统计量构造退出阈值 `low_threshold`
8. 当平滑后的 `delta` 在阶段2之后连续多点低于 `low_threshold` 时，判定 `SD 干扰退出`

如果没有检测到足够明显且持续的扰动抬升/回落，脚本不会强行分段，而是输出整段测试的一般性统计报告。

## 导出结果文件

导出目录中会包含：

- `raw_samples.csv`
  - 全部原始磁力计数据
- `events.csv`
  - STREAM/TEST 事件
- `session_metadata.json`
  - 距离、备注、串口配置等元数据
- `evaluation_summary.csv`
  - 每轮测试的汇总结果
- `cycle_XX_series.csv`
  - 每轮测试逐样本计算结果
- `cycle_XX_heading_offset.png`
  - 每轮测试原始数据和航向偏移曲线图
- `evaluation_report.md`
  - 结果说明报告

重点关注字段：

- `max_abs_heading_offset_deg`
  - 该轮测试中的最大绝对航向偏移
- `p95_abs_heading_offset_deg`
  - 95 分位绝对偏移
- `max_field_delta_lsb`
  - 原始磁场向量最大变化量

## 结果解读建议

- 如果不同距离下 `max_abs_heading_offset_deg` 明显下降，说明拉开 SD 与磁力计距离有效
- 如果同一距离重复测试离散很大，优先排查姿态是否变化、周围金属件是否移动、供电/线束状态是否一致
- 如果结果接近 0°，说明在该距离下 SD 工作对磁力计水平向量影响很小

## 注意事项

- 该方法前提是“唯一变量控制”
- 测试过程中不能手持晃动设备
- 周边不要有移动的铁磁材料
- 线束摆放要固定
- 供电模式尽量一致
- 若 SD 工作模式不一致，结果不可直接横向比较

## 相关文件

- [Kconfig](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/Kconfig)
- [src/cmdMagShow.c](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/src/cmdMagShow.c)
- [src/taskMagShow.c](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/src/taskMagShow.c)
- [src/magSdEval.c](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/src/magSdEval.c)
- [inc/magSdEval.h](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/inc/magSdEval.h)
- [script/mag_sd_eval_tool.py](/d:/48-gg-all-inone-copy/07-gg-all-inone/driverframework/L0_task/03_work/TASK_09_mag_show/script/mag_sd_eval_tool.py)
