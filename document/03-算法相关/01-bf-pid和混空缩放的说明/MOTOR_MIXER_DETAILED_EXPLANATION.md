# 电机混控器详细原理说明

## 1. 为什么PID sum的单位是deg/s？

### 1.1 PID控制器的工作模式

从代码中可以看到，PID控制器工作在**Rate模式（角速度模式）**：

```cpp
// Rate mode (角速度模式): process rate PID controller
float gyroRate[XYZ_AXIS_COUNT];  // 陀螺仪测量的角速度 (deg/s)
const float currentSetpoint = getSetpointRate(axis);  // 期望角速度 (deg/s)
const float errorRate = currentSetpoint - gyroRate[axis];  // 角速度误差 (deg/s)
```

### 1.2 PID各项的计算

```cpp
// P项：比例项
float pTerm = pid_runtime_.pidCoefficient[axis].Kp * errorRate;
pid_data_[axis].P = pTerm * PTERM_SCALE;  // PTERM_SCALE = 0.032029

// I项：积分项
const float iTermChange = pid_runtime_.pidCoefficient[axis].Ki * pid_runtime_.dT * errorRate;
pid_data_[axis].I = CLAMPF(pid_data_[axis].I + iTermChange, -itermLimit, itermLimit);
pid_data_[axis].I *= ITERM_SCALE;  // ITERM_SCALE = 0.244381

// D项：微分项
float delta = -(gyroForD - previousGyroRateDterm[axis]) * pid_runtime_.pidFrequency;
pid_data_[axis].D = pid_runtime_.pidCoefficient[axis].Kd * delta * DTERM_SCALE;  // DTERM_SCALE = 0.000529

// F项：前馈项
pid_data_[axis].F = pid_runtime_.pidCoefficient[axis].Kf * pidSetpointDelta * FEEDFORWARD_SCALE;  // FEEDFORWARD_SCALE = 0.013754

// Sum：总和
pid_data_[axis].Sum = P + I + D + F;
```

### 1.3 单位分析

- **输入**：`errorRate` 的单位是 **deg/s**（角速度误差）
- **P项**：`Kp * errorRate * PTERM_SCALE`，单位仍然是 **deg/s**
- **I项**：`Ki * dT * errorRate * ITERM_SCALE`，单位是 **deg/s**（积分后仍然是角速度）
- **D项**：`Kd * delta * DTERM_SCALE`，单位是 **deg/s**（微分后仍然是角速度）
- **F项**：`Kf * setpointDelta * FEEDFORWARD_SCALE`，单位是 **deg/s**

**结论**：PID sum = P + I + D + F，单位是 **deg/s**（度/秒），表示期望的角速度指令。

### 1.4 为什么需要除以1000？

PID sum的范围通常是 ±500 deg/s，除以1000后得到 ±0.5 的归一化值：
- 500 deg/s ÷ 1000 = 0.5（归一化值）
- 这样将物理单位（deg/s）转换为混控器使用的归一化单位

## 2. 为什么计算所有电机的最大最小值，而不是按轴计算？

### 2.1 混控器的工作原理

混控器将三个轴的指令（roll, pitch, yaw）**混合后**分配到各个电机：

```cpp
// 每个电机的混控值 = Roll贡献 + Pitch贡献 + Yaw贡献
for (int i = 0; i < motor_count_; i++) {
    float mix = scaledAxisPidRoll * current_mixer_[i].roll + 
                scaledAxisPidPitch * current_mixer_[i].pitch +
                scaledAxisPidYaw * current_mixer_[i].yaw;
    motorMix[i] = mix;
}
```

### 2.2 为什么需要看所有电机的范围？

**关键点**：每个电机的混控值是**三个轴的加权和**，不是单独某个轴的值。

**例子**（QuadX混控器）：
```
假设：
- scaledAxisPidRoll = 0.3
- scaledAxisPidPitch = 0.2
- scaledAxisPidYaw = 0.1

QuadX混控器矩阵：
Motor 0 (REAR_R):  mix = +0.3*1.0 + 0.2*(-1.0) + 0.1*(-1.0) = 0.3 - 0.2 - 0.1 = 0.0
Motor 1 (FRONT_R): mix = +0.3*1.0 + 0.2*(-1.0) + 0.1*(+1.0) = 0.3 - 0.2 + 0.1 = 0.2
Motor 2 (REAR_L):  mix = +0.3*1.0 + 0.2*(+1.0) + 0.1*(+1.0) = 0.3 + 0.2 + 0.1 = 0.6
Motor 3 (FRONT_L): mix = +0.3*1.0 + 0.2*(+1.0) + 0.1*(-1.0) = 0.3 + 0.2 - 0.1 = 0.4

motorMixMax = 0.6
motorMixMin = 0.0
motorMixRange = 0.6
```

**如果按轴计算会怎样？**

如果单独看每个轴：
- Roll轴范围：[-0.3, 0.3]（假设所有电机roll系数都是±1.0）
- Pitch轴范围：[-0.2, 0.2]
- Yaw轴范围：[-0.1, 0.1]

但**实际电机的混控值是三个轴的组合**，Motor 2的混控值是0.6，超过了任何单个轴的范围！

### 2.3 为什么必须看所有电机的范围？

**原因**：混控器矩阵的设计使得某些电机的混控值可能是多个轴的同向叠加，导致该电机的混控值可能超过单个轴的范围。

**例子**：
- 如果只有Roll指令（0.5），所有电机的混控值范围是 [-0.5, 0.5]
- 如果Roll(0.5) + Pitch(0.5)同时存在，某些电机的混控值可能达到 1.0（两个轴同向叠加）

因此，**必须计算所有电机的混控值，然后找出最大最小值**，才能正确判断是否需要归一化。

### 2.4 按轴计算的局限性

如果按轴单独计算饱和：
- 无法处理多轴同时作用时的叠加效应
- 可能导致某些电机的混控值超出预期范围
- 无法正确进行归一化处理

**结论**：Betaflight的做法是正确的，必须计算所有电机的混控值范围。

## 3. 归一化原理详解

### 3.1 归一化过程

```cpp
// Step 1: 计算所有电机的混控值
for (int i = 0; i < motor_count_; i++) {
    motorMix[i] = scaledAxisPidRoll * mixer[i].roll + 
                  scaledAxisPidPitch * mixer[i].pitch +
                  scaledAxisPidYaw * mixer[i].yaw;
}

// Step 2: 找出最大最小值
motorMixMax = max(motorMix[0..3])
motorMixMin = min(motorMix[0..3])
motorMixRange = motorMixMax - motorMixMin

// Step 3: 归一化混控值（如果范围>1.0）
if (motorMixRange > 1.0f) {
    factor = 1.0f / motorMixRange;
    for (int i = 0; i < motor_count_; i++) {
        motorMix[i] *= factor;  // 所有混控值按比例缩小
    }
    // 归一化后，motorMixMax和motorMixMin也会按比例缩小
    motorMixMax *= factor;
    motorMixMin *= factor;
    motorMixRange = 1.0f;  // 归一化后范围变为1.0
}

// Step 4: 限制油门范围（防止clipping）
normalizedMotorMixMin = motorMixMin;  // 归一化后的最小值
normalizedMotorMixMax = motorMixMax;  // 归一化后的最大值
throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);

// Step 5: 计算最终电机输出
for (int i = 0; i < motor_count_; i++) {
    motorOutput[i] = motorMix[i] + throttle;  // 混控值 + 油门
    motorOutput[i] = constrainf(motorOutput[i], 0.0f, 1.0f);  // 最终限制
}
```

### 3.2 为什么motorMix乘以factor，而throttle被限制？

**关键理解**：
- **motorMix**：是**相对值**，表示相对于油门的调整量（可以是正负）
- **throttle**：是**基准值**，表示基础推力

**归一化的目的**：
- 如果混控值范围过大（>1.0），需要按比例缩小所有混控值
- 这样确保混控值在合理范围内，不会导致电机输出饱和

**throttle限制的目的**：
- 确保 `motorMix[i] + throttle` 不会超出 [0.0, 1.0] 范围
- 通过限制throttle的范围，为混控值留出空间

### 3.3 完整计算示例

**场景1：正常情况（范围≤1.0）**

```
输入：
- scaledAxisPidRoll = 0.3
- scaledAxisPidPitch = 0.2
- scaledAxisPidYaw = 0.1
- throttle = 0.6

QuadX混控器计算：
Motor 0: mix = 0.3*1.0 + 0.2*(-1.0) + 0.1*(-1.0) = 0.0
Motor 1: mix = 0.3*1.0 + 0.2*(-1.0) + 0.1*(+1.0) = 0.2
Motor 2: mix = 0.3*1.0 + 0.2*(+1.0) + 0.1*(+1.0) = 0.6
Motor 3: mix = 0.3*1.0 + 0.2*(+1.0) + 0.1*(-1.0) = 0.4

motorMixMax = 0.6
motorMixMin = 0.0
motorMixRange = 0.6 < 1.0，不需要归一化

归一化后（实际上没变化）：
Motor 0: mix = 0.0
Motor 1: mix = 0.2
Motor 2: mix = 0.6
Motor 3: mix = 0.4

限制throttle：
normalizedMotorMixMin = 0.0
normalizedMotorMixMax = 0.6
throttle范围：[-0.0, 1.0 - 0.6] = [0.0, 0.4]
throttle = 0.6 → 被限制为 0.4

最终输出：
Motor 0: output = 0.0 + 0.4 = 0.4
Motor 1: output = 0.2 + 0.4 = 0.6
Motor 2: output = 0.6 + 0.4 = 1.0  ← 达到最大值
Motor 3: output = 0.4 + 0.4 = 0.8
```

**场景2：需要归一化（范围>1.0）**

```
输入：
- scaledAxisPidRoll = 0.5
- scaledAxisPidPitch = 0.5
- scaledAxisPidYaw = 0.0
- throttle = 0.5

QuadX混控器计算：
Motor 0: mix = 0.5*1.0 + 0.5*(-1.0) + 0.0*(-1.0) = 0.0
Motor 1: mix = 0.5*1.0 + 0.5*(-1.0) + 0.0*(+1.0) = 0.0
Motor 2: mix = 0.5*1.0 + 0.5*(+1.0) + 0.0*(+1.0) = 1.0
Motor 3: mix = 0.5*1.0 + 0.5*(+1.0) + 0.0*(-1.0) = 1.0

motorMixMax = 1.0
motorMixMin = 0.0
motorMixRange = 1.0，刚好不需要归一化

但如果：
- scaledAxisPidRoll = 0.6
- scaledAxisPidPitch = 0.6

Motor 0: mix = 0.6*1.0 + 0.6*(-1.0) = 0.0
Motor 1: mix = 0.6*1.0 + 0.6*(-1.0) = 0.0
Motor 2: mix = 0.6*1.0 + 0.6*(+1.0) = 1.2  ← 超出范围！
Motor 3: mix = 0.6*1.0 + 0.6*(+1.0) = 1.2  ← 超出范围！

motorMixMax = 1.2
motorMixMin = 0.0
motorMixRange = 1.2 > 1.0，需要归一化！

归一化因子：factor = 1.0 / 1.2 = 0.8333

归一化后：
Motor 0: mix = 0.0 * 0.8333 = 0.0
Motor 1: mix = 0.0 * 0.8333 = 0.0
Motor 2: mix = 1.2 * 0.8333 = 1.0  ← 归一化到1.0
Motor 3: mix = 1.2 * 0.8333 = 1.0  ← 归一化到1.0

归一化后的范围：
motorMixMax = 1.0
motorMixMin = 0.0
motorMixRange = 1.0

限制throttle：
normalizedMotorMixMin = 0.0
normalizedMotorMixMax = 1.0
throttle范围：[-0.0, 1.0 - 1.0] = [0.0, 0.0]
throttle = 0.5 → 被限制为 0.0

最终输出：
Motor 0: output = 0.0 + 0.0 = 0.0
Motor 1: output = 0.0 + 0.0 = 0.0
Motor 2: output = 1.0 + 0.0 = 1.0  ← 达到最大值
Motor 3: output = 1.0 + 0.0 = 1.0  ← 达到最大值
```

### 3.4 关键理解点

1. **motorMix是相对值**：表示相对于油门的调整量
2. **归一化是按比例缩小**：如果范围>1.0，所有混控值按相同比例缩小
3. **throttle限制是为混控值留空间**：确保 `mix + throttle` 不会超出 [0.0, 1.0]
4. **最终输出 = 混控值 + 油门**：混控值可以是负值（降低某个电机），油门是基准值

### 3.5 为什么不能按轴单独归一化？

如果按轴单独归一化：
- Roll轴归一化：所有电机的roll贡献按比例缩小
- Pitch轴归一化：所有电机的pitch贡献按比例缩小
- Yaw轴归一化：所有电机的yaw贡献按比例缩小

**问题**：
- 每个轴的归一化因子可能不同
- 归一化后，某些电机的混控值（三个轴的组合）可能仍然超出范围
- 无法保证所有电机的最终输出都在 [0.0, 1.0] 范围内

**正确做法**：
- 先计算所有电机的混控值（三个轴的组合）
- 找出所有混控值的最大最小值
- 如果范围>1.0，按比例缩小所有混控值
- 这样确保所有电机的混控值都在合理范围内

## 4. 总结

1. **PID sum单位是deg/s**：因为PID工作在Rate模式，输出是角速度指令
2. **必须计算所有电机的范围**：因为混控值是三个轴的组合，可能超出单个轴的范围
3. **归一化是按比例缩小所有混控值**：确保混控值范围≤1.0
4. **throttle限制是为混控值留空间**：确保最终输出不会超出 [0.0, 1.0]

