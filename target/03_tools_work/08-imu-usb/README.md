# STM32H750 + RT-Thread USB 动态 CDC/MSC 方案

这个目录已经放入一套可移植的工程骨架，目标就是在 **STM32H750 + RT-Thread + SDMMC + USB Device(OTG FS)** 上实现：

- 模式1：`CDC`，PC 只看到虚拟串口，MCU 挂载 SD 并持续记录数据
- 模式2：`MSC`，PC 只看到 U 盘，MCU 不再访问 SD，SD 完全交给 USB MSC
- 切换方式：CDC 命令 `enter_msc`，或者 MCU 内部直接调用 `enter_msc_mode()` / `enter_cdc_mode()`
- 切换必须重新枚举，不使用 composite

注意：本目录代码重点放在 **工程落地骨架 + 切换时序 + SD 互斥控制**。默认传感器采集函数是一个 `weak` 假数据实现，方便你直接跑通 USB/SD 架构；接入真实 IMU 时，只要替换 `usb_mode_fill_sensor_line()` 即可。

---

## 1. menuconfig 需要打开的选项

建议最终配置至少包含以下项：

### RT-Thread Components

- `RT_USING_USB`
- `RT_USING_USB_DEVICE`
- `RT_USB_DEVICE_CDC`
- `RT_USB_DEVICE_MSTORAGE`
- **不要**启用真正的 CDC+MSC 复合设备运行逻辑
- `RT_USING_DFS`（仅为了兼容 RT-Thread 的块设备/SD 卡驱动依赖，不把它作为主文件系统接口）
- `DFS_USING_POSIX`（保持当前仓库的 DFS 依赖可编译）
- 文件读写实际走本工程内置的 `FatFS API`，不依赖 DFS 挂载层

### Device Drivers

- `RT_USING_SDIO`
- `RT_USING_PIN`
- `RT_USING_SERIAL`
- `RT_USING_SPI`（你的 IMU 若走 SPI）

### BSP / STM32 外设

- `BSP_USING_USBD`
- `BSP_USING_SDIO`
- `BSP_USING_SDIO1`

### 本工程里额外加的宏

- `RT_USB_DYNAMIC_CLASS_SELECT`

这个宏的作用是：

- 禁掉 RT-Thread USB class 的自动注册启动流程
- 由应用层自己决定当前枚举成 CDC 还是 MSC
- 避免一上电就固定成某个 class，便于动态重枚举

当前目录里已经改了这些位置：

- `rtconfig.h`
- `.config`
- `kernel/rtthread/components/legacy/usb/usbdevice/class/cdc_vcom.c`
- `kernel/rtthread/components/legacy/usb/usbdevice/class/mstorage.c`
- `kernel/rtthread/bsp/stm32/libraries/HAL_Drivers/drivers/drv_usbd.c`

---

## 2. USB Device 配置

### CDC 模式

调用 RT-Thread legacy USB device class：

- `rt_usbd_function_cdc_create()`

模式下行为：

- 枚举为虚拟串口
- PC 端只看到 COM 口
- MCU 自己挂载 SD 并记录数据

### MSC 模式

调用：

- `rt_usbd_function_mstorage_create()`

依赖块设备名字：

- `RT_USB_MSTORAGE_DISK_NAME "sd0"`

模式下行为：

- 枚举为 MSC 磁盘
- PC 端只看到 U 盘
- MCU 不再挂载也不再写 SD

---

## 3. 动态切换实现方式

核心文件：

- `applications/usb_mode_manager.c`
- `applications/usb_mode_manager.h`
- `applications/fatfs_sdcard_port.c`
- `applications/fatfs_sdcard_port.h`

### 3.1 状态机

```c
typedef enum
{
    USB_APP_MODE_CDC = 0,
    USB_APP_MODE_MSC,
    USB_APP_MODE_SWITCHING,
} usb_app_mode_t;
```

### 3.2 切换总原则

#### CDC -> MSC

严格顺序：

1. 停止日志/采集线程
2. `f_sync()` / `f_close()`
3. `RT_DEVICE_CTRL_BLK_SYNC`
4. `f_mount(NULL, "0:", 0)` 解除 FatFS 挂载
5. `stm_usbd_stop()` 断开 USB
6. 重新创建 MSC function
7. `stm_usbd_start()`
8. `rt_usbd_set_config(..., 1)` 重新枚举

#### MSC -> CDC

严格顺序：

1. 停止 MSC USB
2. 释放 MSC function
3. 重新 `f_mount()` 挂载逻辑盘 `0:`
4. 重新打开日志文件
5. 启动 CDC USB
6. 恢复采集线程

### 3.3 关键函数

- `usb_mode_switch_to()`
- `usb_mode_start_usb()`
- `usb_mode_stop_usb()`
- `usb_mode_mount_fs()`
- `usb_mode_unmount_fs()`
- `enter_msc_mode()`
- `enter_cdc_mode()`

---

## 4. 如何 deinit USB / re-init 为另一种 class

这个目录里已经给 STM32 USB Device driver 做了可控启动/停止封装：

文件：

- `kernel/rtthread/bsp/stm32/libraries/HAL_Drivers/drivers/drv_usbd.c`

新增接口：

```c
udcd_t stm_usbd_get_udcd(void);
int stm_usbd_start(void);
int stm_usbd_stop(void);
```

### `stm_usbd_stop()` 做的事

```c
HAL_PCD_DevDisconnect(&_stm_pcd);
rt_thread_mdelay(20);
HAL_PCD_Stop(&_stm_pcd);
HAL_PCD_DeInit(&_stm_pcd);
HAL_NVIC_DisableIRQ(USBD_IRQ_TYPE);
```

这一步就是 **强制 USB 断开 + 让 PC 看到设备消失**。

### `stm_usbd_start()` 做的事

本质是重新走：

- `rt_device_init((rt_device_t)&_stm_udc)`
- `HAL_PCD_Init`
- `HAL_PCD_Start`

然后应用层再调用：

```c
rt_usbd_set_config(device, 1);
```

于是 PC 会把它当成一个 **新设备类型** 重新识别。

---

## 5. SD 卡 + FatFS + MSC 的关系处理

这是整个方案最关键的点。

### CDC 模式时

所有权属于 MCU：

- MCU 通过 `FatFS(f_mount/f_open/f_write)` 独占 SD
- MCU 打开日志文件
- MCU 周期写入数据
- USB 只提供 CDC，不暴露磁盘

### MSC 模式时

所有权属于 PC：

- MCU 关闭日志文件
- MCU 卸载文件系统
- MCU 不再读写 `sd0`
- USB MSC 直接透传块设备 `sd0`

### 绝对不能做的事

- MSC 模式下 MCU 还在写文件
- MSC 模式下 FatFS 还挂着
- PC 和 MCU 同时访问 SD

不然非常容易把 FAT 表写坏。

---

## 6. 关键代码框架

### 6.1 主入口

文件：`applications/main.c`

```c
int main(void)
{
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    usb_mode_manager_start();

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
```

### 6.2 模式切换 API

文件：`applications/usb_mode_manager.c`

```c
int enter_msc_mode(void)
{
    usb_mode_request(USB_APP_MODE_MSC);
    return RT_EOK;
}

int enter_cdc_mode(void)
{
    usb_mode_request(USB_APP_MODE_CDC);
    return RT_EOK;
}
```

### 6.3 通过 CDC 命令触发

CDC 命令线程识别：

- `enter_msc`
- `status`

如果你还想加业务逻辑自动切换，直接在应用任意位置调用：

```c
enter_msc_mode();
enter_cdc_mode();
```

### 6.4 日志采集钩子

默认实现：

```c
rt_weak int usb_mode_fill_sensor_line(char *buf, rt_size_t size)
```

你把它替换成真实 IMU 读取即可，比如：

```c
int usb_mode_fill_sensor_line(char *buf, rt_size_t size)
{
    imu_raw_t raw;
    imu_read(&raw);
    return rt_snprintf(buf, size,
                       "%lu,%d,%d,%d\r\n",
                       raw.timestamp_us,
                       raw.ax,
                       raw.ay,
                       raw.az);
}
```

---

## 7. USB 重新枚举怎么触发

本方案不是简单改 descriptor，而是：

1. 先 `HAL_PCD_DevDisconnect()`
2. 再 `HAL_PCD_Stop()`
3. 再 `HAL_PCD_DeInit()`
4. 延时一小段时间
5. 重新 `HAL_PCD_Init()` + `HAL_PCD_Start()`
6. 重新建立另一种 USB function

所以对 PC 来说：

- 原设备先消失
- 再出现一个全新的设备类型

这就是你要的 **重枚举切换**。

---

## 8. H750 特殊注意事项

### 8.1 Cache 一致性

当前 BSP 在 `board/board.c` 里默认把 DCache 关掉了：

```c
board_disable_dcache_if_enabled();
```

这样做的好处是：

- USB DMA
- SDMMC DMA/IDMA

都不会踩到 DCache 一致性问题，先把功能跑通最稳。

如果后面你要重新打开 DCache，必须自己处理：

- DMA buffer 放到 non-cacheable 区域
- 或者在 DMA 前 `SCB_CleanDCache_by_Addr()`
- DMA 后 `SCB_InvalidateDCache_by_Addr()`

### 8.2 USB buffer 对齐

H7 上建议：

- USB/MSC 数据 buffer 至少 32-byte 对齐
- 大块缓冲放 AXI SRAM 或专门 non-cache section

如果你后续打开 DCache，这一点非常重要。

### 8.3 SDMMC 引脚

当前按常见 H750/WeAct TF 连接配置为：

- `PC8`  -> D0
- `PC9`  -> D1
- `PC10` -> D2
- `PC11` -> D3
- `PC12` -> CK
- `PD2`  -> CMD

对应代码已经补到：

- `board/CubeMX_Config/Src/stm32h7xx_hal_msp.c`

### 8.4 SDMMC 时钟

已在：

- `board/board.c`

增加：

```c
PeriphClkInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_SDMMC;
PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
```

---

## 9. 常见坑 + 解决方案

### 坑1：切到 MSC 后 PC 不识别

排查：

- 是否真的执行了 `HAL_PCD_DevDisconnect()`
- 是否有足够的断开延时（建议 100~200ms）
- descriptor/class 是否已经换成 MSC

### 坑2：切到 MSC 后 U 盘打不开

排查：

- `RT_USB_MSTORAGE_DISK_NAME` 是否真的是 `sd0`
- SD 卡块设备是否已正常枚举
- MCU 侧是否还挂着 FatFS

### 坑3：切回 CDC 后日志打不开

排查：

- MSC 停止后是否重新 `dfs_mount`
- 是否重新创建/打开日志文件
- 是否 PC 在 MSC 模式下写坏了 FAT，需要 PC 先安全弹出

### 坑4：文件系统损坏

根因几乎都是：

- 切换前没有 `fsync`
- 没有关闭文件
- 没有 `BLK_SYNC`
- 没有卸载文件系统
- PC 和 MCU 同时访问 SD

### 坑5：H750 上偶发 USB/SDMMC 数据错乱

优先检查：

- DCache 是否开启
- buffer 是否对齐
- DMA 缓冲是否在可 DMA 区域

---

## 10. 当前目录改动说明

### 应用层

- `applications/main.c`
- `applications/usb_mode_manager.c`
- `applications/usb_mode_manager.h`
- `applications/fatfs_sdcard_port.c`
- `applications/fatfs_sdcard_port.h`
- `applications/imu_reader_thread.cpp`（改成空兼容桩，避免旧 demo 影响）

### BSP / 配置

- `rtconfig.h`
- `.config`
- `board/board.c`
- `board/CubeMX_Config/Inc/stm32h7xx_hal_conf.h`
- `board/CubeMX_Config/Src/stm32h7xx_hal_msp.c`

### RT-Thread USB 动态切换补丁

- `kernel/rtthread/components/drivers/include/drivers/usb_device.h`
- `kernel/rtthread/components/legacy/usb/usbdevice/core/usbdevice_core.c`
- `kernel/rtthread/components/legacy/usb/usbdevice/class/cdc_vcom.c`
- `kernel/rtthread/components/legacy/usb/usbdevice/class/mstorage.c`
- `kernel/rtthread/bsp/stm32/libraries/HAL_Drivers/drivers/drv_usbd.c`

---

## 11. 运行步骤建议

1. 先确认 SDMMC1 硬件连线就是 `PC8~PC12 + PD2`
2. `menuconfig` 按上面选项核对一遍
3. `pkgs --update`
4. 重新生成工程并编译
5. 上电后默认进入 CDC
6. PC 打开虚拟串口，发送：

```text
enter_msc
```

7. 设备会断开并重新枚举为 U 盘
8. 需要切回 CDC 时，从 UART shell 或内部逻辑执行：

```text
enter_cdc_mode
```

---

## 12. 说明

当前代码已经把 **动态切换框架、USB 受控重启、直连 FatFS 日志、SD 访问互斥、CDC 命令触发** 都放进去了。

如果你下一步要我继续，我建议直接做这两件事之一：

1. 把 `usb_mode_fill_sensor_line()` 改成你当前 IMU 驱动的真实采样输出
2. 把 SD 卡检测、卡拔出处理、PC 安全弹出策略再补齐成生产版

---

## 13. 调试文件约定

- 所有调试/烧录过程中临时生成或手工维护的 `.jlink` 文件，统一放到 `build/` 目录下
- 不要把 `flash_flags.jlink`、`flash_verify.jlink` 这类文件放在工程根目录
- 后续如果新增新的 J-Link 命令脚本，也按这个规则放到 `build/`
