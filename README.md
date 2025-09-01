# GG-All-In-One 嵌入式飞控项目

## 🎯 项目主旨

**GG-All-In-One** 是一个革命性的嵌入式飞控项目，采用"菜谱式"（Recipe）架构理念，将复杂的飞控系统开发转化为简单、可组合的模块化配置过程。

> **核心理念**: 像烹饪一样简单，像积木一样灵活，像菜谱一样可重复

### 🌟 项目特点

#### 🍳 菜谱式架构 (Recipe Architecture)
- **模块化设计**: 每个功能模块都像菜谱中的食材，可以独立验证、添加或移除
- **即插即用**: 传感器、MCU、中间件都可以像食材一样自由搭配
- **标准化接口**: 统一的模块接口，确保不同"食材"之间的兼容性
- **可重复性**: 成功的配置可以保存为"菜谱"，快速复用到其他项目

#### 🔧 高度模块化
- **硬件模块化**: 支持任意MCU平台（STM32、AT32、RISC-V等）
- **传感器模块化**: 支持任意传感器型号（MPU6000、ICM42688、BMP280等）
- **软件模块化**: 支持不同中间件（RT-Thread、FreeRTOS等）
- **业务模块化**: 支持不同飞控算法栈（PX4、FMT、APM等）

#### ⚡ 快速适配能力
- **MCU适配**: 通过配置文件快速切换目标MCU，无需重写底层代码
- **传感器适配**: 新传感器只需实现标准接口，即可无缝集成
- **中间件适配**: 支持多种RTOS，通过抽象层实现平滑切换
- **算法栈适配**: 支持PX4、FMT、APM等不同飞控算法栈，可快速切换
- **业务层适配**: 飞控算法栈独立于硬件，可快速适配不同应用场景

## 🏗️ 菜谱式架构详解

### 📋 Recipe（菜谱）概念

每个飞控项目都可以看作一个"菜谱"，包含以下要素：

```yaml
# PX4算法栈菜谱示例
Recipe: STM32F411_PX4_Quadcopter
Ingredients:
  MCU: STM32F411CE
  Sensors:
    - IMU: MPU6000
    - Barometer: BMP280
    - GPS: NEO-8M
  Middleware: RT-Thread
  FlightController: PX4
  Features:
    - PID Control
    - GPS Navigation
    - Telemetry
    - Mission Planning

# FMT算法栈菜谱示例
Recipe: STM32F427_FMT_Quadcopter
Ingredients:
  MCU: STM32F427VI
  Sensors:
    - IMU: ICM42688
    - Barometer: SPL06
    - GPS: NEO-8M
  Middleware: RT-Thread
  FlightController: FMT
  Features:
    - Advanced PID Control
    - GPS Navigation
    - Telemetry
    - Advanced Mission Planning
```

### 🔄 模块化验证流程

```
1. 独立验证每个"食材"（模块）
   ├── MCU基础功能验证
   ├── 传感器驱动验证
   ├── 中间件功能验证
   └── 业务逻辑验证

2. 组合验证"菜谱"
   ├── 硬件兼容性测试
   ├── 软件集成测试
   ├── 系统性能测试
   └── 飞行测试

3. 保存成功"菜谱"
   ├── 配置文件归档
   ├── 测试报告记录
   ├── 使用说明文档
   └── 版本管理
```

## 📁 项目结构详解

### `.vscode/` - 开发环境配置
- **代码格式化设置**: 统一的代码风格和格式化规则
- **目标烧录配置**: 支持多种MCU的烧录和调试配置
- **开发体验优化**: 智能提示、错误检查、调试支持

### `configs/` - 菜谱配置文件
- **01-base/**: 基础"菜谱"，包含各种MCU的基础配置
- **02-tools/**: 工具"菜谱"，开发工具和调试配置
- **03-project/**: 项目"菜谱"，特定飞控算法栈的完整配置

**菜谱使用示例**:
```bash
# 使用STM32F411 minifly菜谱构建固件
menuconfig --config=..\..\..\configs\01-base\01-base_f411ce-8m-minifly --silent && scons

# 使用其他菜谱构建不同项目
menuconfig --config=..\..\..\configs\03-project\your-recipe --silent && scons

# 菜谱示例
# PX4算法栈: menuconfig --config=..\..\..\configs\03-project\px4_quadcopter --silent && scons
# FMT算法栈: menuconfig --config=..\..\..\configs\03-project\fmt_quadcopter --silent && scons
# APM算法栈: menuconfig --config=..\..\..\configs\03-project\apm_quadcopter --silent && scons

### `document/` - 技术文档库
- **原理图**: 各目标板子的硬件设计图纸
- **数据手册**: MCU、传感器、外设的详细技术规格
- **菜谱文档**: 成功配置的详细说明和使用指南
- **设计规范**: 硬件设计标准和最佳实践

### `driverFramework/` - 模块化驱动框架
采用四层架构，每层都是独立的"食材"：

- **L0_task (应用层)**: 飞控任务调度、控制算法、用户应用
- **L1_middleWare (中间层)**: 通信协议、数据管理、系统服务
- **L2_device (设备层)**: 传感器驱动、执行器控制、设备抽象
- **L3_peripheral (外设层)**: 硬件寄存器操作、底层驱动、外设管理

**模块化优势**:
- 每个模块都可以独立开发、测试和验证
- 模块间通过标准接口通信，降低耦合度
- 支持模块的热插拔和动态配置

### `kernel/` - 操作系统内核
- **RT-Thread**: 作为子模块集成，支持任意版本切换
- **多RTOS支持**: 预留接口支持FreeRTOS等其他实时操作系统
- **版本管理**: 灵活的内核版本选择和升级

### `python/` - 工具脚本
- **数据处理脚本**: 传感器数据分析和处理
- **开发工具**: 自动化构建、测试和部署脚本
- **数据分析**: 飞行数据可视化、性能分析工具

### `target/` - 硬件平台支持
- **01-stm32/**: STM32系列MCU的BSP支持
- **02-at32/**: AT32系列MCU的BSP支持
- **多平台支持**: 计划支持RISC-V等架构
- **BSP标准化**: 统一的硬件抽象接口

### `tools/` - 构建工具链
- **env工具**: RT-Thread官方构建环境，作为子模块集成
- **工具链管理**: 支持GCC 12.3等多种编译器版本
- **一键构建**: 无需额外软件安装，克隆即用

### `env.bat` - 环境启动脚本
- **一键启动**: 在项目根目录执行 `./env.bat` 即可启动完整构建环境
- **自动配置**: 自动设置环境变量、工具链路径和构建工具
- **Windows优化**: 专为Windows环境优化的启动脚本

## 🚀 快速开始

### 环境准备
```bash
# 1. 克隆项目
git clone <repository-url>
cd gg-all-in-one

# 2. 同步子模块
git submodule update --init --recursive

# 3. 启动构建环境
./env.bat
```

### 菜谱式构建流程
```bash
# 1. 选择目标菜谱（进入对应BSP目录）
cd .\target\01-stm32\01-stm32f411ce\

# 2. 使用菜谱配置并构建
menuconfig --config=..\..\..\configs\01-base\01-base_f411ce-8m-minifly --silent && scons
```

### 菜谱使用示例
```bash
# 使用STM32F411 minifly菜谱
cd .\target\01-stm32\01-stm32f411ce\
menuconfig --config=..\..\..\configs\01-base\01-base_f411ce-8m-minifly --silent && scons

# 使用其他菜谱（需要先进入对应的BSP目录）
cd .\target\your-target-path\
menuconfig --config=..\..\..\configs\01-base\your-recipe --silent && scons
```

## 🔧 技术特性

### 🍳 菜谱式特性
- **模块化设计**: 每个功能都是独立的"食材"，可以自由组合
- **标准化接口**: 统一的模块接口，确保兼容性
- **可重复性**: 成功的配置可以保存为菜谱，快速复用
- **可验证性**: 每个模块都可以独立验证和测试

### 🔄 快速适配能力
- **MCU适配**: 通过配置文件快速切换目标MCU
- **传感器适配**: 新传感器只需实现标准接口
- **中间件适配**: 支持多种RTOS，平滑切换
- **算法栈适配**: 支持PX4、FMT、APM等不同飞控算法栈
- **业务层适配**: 飞控算法栈独立于硬件

### 🛠️ 开发效率
- **开箱即用**: 完整的开发工具链，无需额外配置
- **版本管理**: 灵活的子模块管理，支持版本切换
- **自动化构建**: 简化的构建流程，降低开发门槛
- **调试支持**: 完整的调试工具和配置

## 📚 开发指南

### 创建新菜谱
1. **选择基础菜谱**: 从现有菜谱中选择最接近的基础配置
2. **修改食材**: 根据需要替换或添加模块（MCU、传感器、算法栈等）
3. **验证食材**: 独立测试每个新模块
4. **组合测试**: 验证整个菜谱的兼容性
5. **保存菜谱**: 将成功配置保存为新的菜谱文件

### 模块开发规范
- 遵循标准接口定义
- 提供完整的测试用例
- 编写详细的文档说明
- 支持配置参数化

详细的开发文档请参考 `document/` 文件夹中的相关文档。

## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进项目。请确保：
- 遵循项目的菜谱式架构理念
- 遵循模块化开发规范
- 添加必要的测试用例
- 更新相关文档和菜谱

## 📄 许可证

[请在此处添加许可证信息]

## 📞 联系方式

[请在此处添加联系方式信息]

---

**GG-All-In-One** - 让飞控开发变得像烹饪一样简单而有趣！

> *"在GG-All-In-One的世界里，每个飞控项目都是一道美味的菜，每个模块都是精心挑选的食材，每个菜谱都是成功的配方。"*
