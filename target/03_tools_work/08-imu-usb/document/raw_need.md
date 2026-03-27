| **日期** **Data** <br/> | **修订版本** **Revision****Version** | **修改描述****Change Description** | **作者****Author** |
| ----------------------------------- | ------------------------------------------------------ | ---------------------------------------------- | ------------------------------ |
| 2023-05-19                          | V1.0                                                   | 草稿                                           | 周翩                           |
| <br/>                               | <br/>                                                  | <br/>                                          | <br/>                          |
| <br/>                               | <br/>                                                  | <br/>                                          | <br/>                          |
| <br/>                               | <br/>                                                  | <br/>                                          | <br/>                          |
| <br/>                               | <br/>                                                  | <br/>                                          | <br/>                          |
| <br/>                               | <br/>                                                  | <br/>                                          | <br/>                          |

# 一、测量原理及特点

## 原理

不介绍了。

## 特点

消费量的无人机基本都使用MEMS的IMU，优点是成本低廉，体积小，集成度高，缺点也很明显，就是精度低，稳定性差。

当然我们对精度的追求并不是无上限的，对于目前的使用需求，主流的MEMS IMU是指标是够用的，但是他们在不同工况下的表现依然有些差异。

### imu环境敏感因素

* 温度变化
* 高强度冲击、振动

# 二、选型要求

## 基本要求

* 工作温度范围（operating temperature range）
  * -40\~85°
* 量程（full scale range)
  * gyroscope: ≮2000dps
  * accelerometer: ≮ 8g，
* 分辨率(resolution）
  * 不低于 16bit
* 带宽
  * 大于400Hz，group delay < 4ms
* 采样率(sample rate)
  * 不低于1KHz
* 正交度(cross-axis sensitivity)
  * 小于±2%

## 性能要求

**由于很多imu的数据手册标称的指标和真实指标差异比较大，所以手册中的指标仅作为参考**。

### 核心指标（强制）

* 零偏(zero-rate output)
  * accelerometer：80mg
  * gyroscope: 1°/s
* 温漂(zero-rate output vs temperature)
  * accelerometer：小于0.2mg/℃
  * gyroscope: 0.01(°/s)/℃
* 角度随机游走(ARW, Angle Random Walk)
  * 0.005 °/Hz

ARW、Rate Noise Density、RMS之间的换算

* ARW = Rate Noise Density
* ARW = RMS / √BW， BW, bandwitdh，即带宽

### 关键指标（强制）

### 一般指标（推荐）

# 三、测试要求

## IC级别测试

* 零偏及零偏稳定性

  * 常温，间隔1Hour以上上电，记录零偏变化(每隔一小时，记录一分钟)
* 温漂

  * 温度冲击比较小时（小于0.5℃/s)，不同温度条件下，记录不同温度下的零偏
    （温度范围：0到50）
  * 温度冲击比较大时（大于于5℃/s)，不同温度条件下，记录不同温度下的零偏
* 角度随机游走

  * 后面补充测试方法和程序
* 震动条件下的输出，数据更新是否连续

  * 振动的峰值约在90%的量程
    需要提供振动台装置，不同周期，

    持续1s的大的加速度，看峰值是否90%的量程；

    在振动大的时候，数据输出是否连续，符合物理具体，

    看会不会突然不出数据；

  峰值保持1s，且大于90%

  定性摸底，不是搞标定

  测试时间：1s

  * 振动的幅值超过量程
* 冲击后数据恢复

  * 冲击不超过加计量程
  * 冲击冲过加计量程
    拿东西敲一下，模拟跌落的情况，炸机的情况；

    看imu的数据是否连续是否正常，

    冲击结束之后，数据是否恢复正常

    机械结构是否振坏

## 整机级别测试

暂无。

# 参考

[Department of Computer Science and Technology – Technical reports: UCAM-CL-TR-696](http://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.html)

[武汉大学多源智能导航实验室](http://i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=d4fc52fc6eff4be5859e18406b62d46d)
