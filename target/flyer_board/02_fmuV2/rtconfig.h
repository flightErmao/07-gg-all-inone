#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

#define SOC_STM32F427II

/* RT-Thread Kernel */

/* klibc options */

/* rt_vsnprintf options */

/* end of rt_vsnprintf options */

/* rt_vsscanf options */

/* end of rt_vsscanf options */

/* rt_memset options */

/* end of rt_memset options */

/* rt_memcpy options */

/* end of rt_memcpy options */

/* rt_memmove options */

/* end of rt_memmove options */

/* rt_memcmp options */

/* end of rt_memcmp options */

/* rt_strstr options */

/* end of rt_strstr options */

/* rt_strcasecmp options */

/* end of rt_strcasecmp options */

/* rt_strncpy options */

/* end of rt_strncpy options */

/* rt_strcpy options */

/* end of rt_strcpy options */

/* rt_strncmp options */

/* end of rt_strncmp options */

/* rt_strcmp options */

/* end of rt_strcmp options */

/* rt_strlen options */

/* end of rt_strlen options */

/* rt_strnlen options */

/* end of rt_strnlen options */
/* end of klibc options */
#define RT_NAME_MAX 24
#define RT_CPUS_NR 1
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256

/* kservice options */

/* end of kservice options */
#define RT_USING_DEBUG
#define RT_DEBUGING_ASSERT
#define RT_DEBUGING_COLOR
#define RT_DEBUGING_CONTEXT

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
/* end of Inter-Thread communication */

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP
#define RT_USING_HEAP
/* end of Memory Management */
#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart6"
#define RT_VER_NUM 0x50201
#define RT_BACKTRACE_LEVEL_MAX_NR 32
/* end of RT-Thread Kernel */
#define RT_USING_HW_ATOMIC
#define RT_USING_CPU_FFS
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M4

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 80
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10
#define FINSH_USING_OPTION_COMPLETION

/* DFS: device virtual file system */

/* end of DFS: device virtual file system */
#define RT_USING_FAL
#define FAL_USING_DEBUG
#define FAL_PART_HAS_TABLE_CFG

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_UNAMED_PIPE_NUMBER 64
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V2
#define RT_SERIAL_BUF_STRATEGY_OVERWRITE
#define RT_SERIAL_USING_DMA
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PWM
#define RT_USING_SPI
#define RT_USING_PIN
#define RT_USING_KTIME
/* end of Device Drivers */

/* C/C++ and POSIX layer */

/* ISO-ANSI C layer */

/* Timezone and Daylight Saving Time */

#define RT_LIBC_USING_LIGHT_TZ_DST
#define RT_LIBC_TZ_DEFAULT_HOUR 8
#define RT_LIBC_TZ_DEFAULT_MIN 0
#define RT_LIBC_TZ_DEFAULT_SEC 0
/* end of Timezone and Daylight Saving Time */
/* end of ISO-ANSI C layer */

/* POSIX (Portable Operating System Interface) layer */

#define RT_USING_POSIX_DELAY

/* Interprocess Communication (IPC) */


/* Socket is in the 'Network' category */

/* end of Interprocess Communication (IPC) */
/* end of POSIX (Portable Operating System Interface) layer */
#define RT_USING_CPLUSPLUS
/* end of C/C++ and POSIX layer */

/* Network */

/* end of Network */

/* Memory protection */

/* end of Memory protection */

/* Utilities */

/* end of Utilities */

/* Using USB legacy version */

#define RT_USING_USB
#define RT_USING_USB_DEVICE
#define RT_USBD_THREAD_STACK_SZ 4096
#define USB_VENDOR_ID 0x0FFE
#define USB_PRODUCT_ID 0x0001
#define RT_USB_DEVICE_COMPOSITE
#define RT_USB_DEVICE_CDC
#define RT_USB_DEVICE_NONE
#define RT_VCOM_TASK_STK_SIZE 512
#define RT_CDC_RX_BUFSIZE 128
#define RT_VCOM_SERNO "32021919830108"
#define RT_VCOM_SER_LEN 14
#define RT_VCOM_TX_TIMEOUT 1000
/* end of Using USB legacy version */
/* end of RT-Thread Components */

/* RT-Thread Utestcases */

/* end of RT-Thread Utestcases */

/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */

/* end of Marvell WiFi */

/* Wiced WiFi */

/* end of Wiced WiFi */

/* CYW43012 WiFi */

/* end of CYW43012 WiFi */

/* BL808 WiFi */

/* end of BL808 WiFi */

/* CYW43439 WiFi */

/* end of CYW43439 WiFi */
/* end of Wi-Fi */

/* IoT Cloud */

/* end of IoT Cloud */
/* end of IoT - internet of things */

/* security packages */

/* end of security packages */

/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* end of JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* XML: Extensible Markup Language */

/* end of XML: Extensible Markup Language */
/* end of language packages */

/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */

/* end of LVGL: powerful and easy-to-use embedded GUI library */

/* u8g2: a monochrome graphic library */

/* end of u8g2: a monochrome graphic library */
/* end of multimedia packages */

/* tools packages */

#define PKG_USING_UMCN
#define PKG_USING_UMCN_LATEST_VERSION
#define UMCN_USING_CMD
#define PKG_USING_VCONSOLE
#define PKG_USING_VCONSOLE_LATEST_VERSION
/* end of tools packages */

/* system packages */

/* enhanced kernel services */

/* end of enhanced kernel services */

/* acceleration: Assembly language or algorithmic acceleration packages */

/* end of acceleration: Assembly language or algorithmic acceleration packages */

/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

#define PKG_USING_CMSIS_CORE
#define PKG_USING_CMSIS_CORE_LATEST_VERSION
/* end of CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

/* Micrium: Micrium software products porting for RT-Thread */

/* end of Micrium: Micrium software products porting for RT-Thread */
/* end of system packages */

/* peripheral libraries and drivers */

/* HAL & SDK Drivers */

/* STM32 HAL & SDK Drivers */

#define PKG_USING_STM32F4_HAL_DRIVER
#define PKG_USING_STM32F4_HAL_DRIVER_LATEST_VERSION
#define PKG_USING_STM32F4_CMSIS_DRIVER
#define PKG_USING_STM32F4_CMSIS_DRIVER_LATEST_VERSION
/* end of STM32 HAL & SDK Drivers */

/* Infineon HAL Packages */

/* end of Infineon HAL Packages */

/* Kendryte SDK */

/* end of Kendryte SDK */

/* WCH HAL & SDK Drivers */

/* end of WCH HAL & SDK Drivers */

/* AT32 HAL & SDK Drivers */

/* end of AT32 HAL & SDK Drivers */

/* HC32 DDL Drivers */

/* end of HC32 DDL Drivers */

/* NXP HAL & SDK Drivers */

/* end of NXP HAL & SDK Drivers */

/* NUVOTON Drivers */

/* end of NUVOTON Drivers */
/* end of HAL & SDK Drivers */

/* sensors drivers */

/* end of sensors drivers */

/* touch drivers */

/* end of touch drivers */
/* end of peripheral libraries and drivers */

/* AI packages */

/* end of AI packages */

/* Signal Processing and Control Algorithm Packages */

/* end of Signal Processing and Control Algorithm Packages */

/* miscellaneous packages */

/* project laboratory */

/* end of project laboratory */

/* samples: kernel and components samples */

#define PKG_USING_PERIPHERAL_SAMPLES
#define PKG_USING_PERIPHERAL_SAMPLES_LATEST_VERSION
/* end of samples: kernel and components samples */

/* entertainment: terminal games and other interesting software packages */

/* end of entertainment: terminal games and other interesting software packages */
#define PKG_USING_OPTPARSE
#define PKG_USING_OPTPARSE_LATEST_VERSION
#define PKG_USING_UPARAM
#define PKG_USING_UPARAM_LATEST_VERSION
/* end of miscellaneous packages */

/* Arduino libraries */


/* Projects and Demos */

/* end of Projects and Demos */

/* Sensors */

/* end of Sensors */

/* Display */

/* end of Display */

/* Timing */

/* end of Timing */

/* Data Processing */

/* end of Data Processing */

/* Data Storage */

/* Communication */

/* end of Communication */

/* Device Control */

/* end of Device Control */

/* Other */

/* end of Other */

/* Signal IO */

/* end of Signal IO */

/* Uncategorized */

/* end of Arduino libraries */
/* end of RT-Thread online packages */
#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32F4

/* DriverFramework Config */

/* L0_TASK_CONFIG */

#define TASK_TOOL_01_ANOTC_TELEM_EN
#define TASK_TOOL_01_ANOTC_TELEM_DEVICE_DEFAULT "uart2"
#define TASK_TOOL_01_ANOTC_TELEM_BAUD_RATE 500000
#define PROJECT_MINIFLY_TASK_SENSOR_EN
#define PROJECT_MINIFLY_TASK_SENSOR_IMU_NAME "mpu6000"
#define PROJECT_MINIFLY_TASK_SENSOR_TIMER_TRIGGER_EN
#define PROJECT_MINIFLY_TASK_SENSOR_ROTATION "ROTATION_PITCH_180_NEGATE_X"
#define PROJECT_MINIFLY_TASK_STABLIZE_EN
#define PROJECT_MINIFLY_TASK_STABLIZE_ATKP_LOG_EN
#define PROJECT_MINIFLY_TASK04_DISTRIBUTE_EN
#define PROJECT_MINIFLY_TASK04_DISTRIBUTE_ACK_EN
#define PROJECT_MINIFLY_TASK04_DISTRIBUTE_PID_EN
#define PROJECT_MINIFLY_TASK05_PARAM_EN
#define PROJECT_MINIFLY_TASK05_PARAM_AUTO_SAVE_EN
#define PROJECT_FMT_TASK01_RC_EN
#define PROJECT_FMT_TASK01_RC_DEVICE_DEFAULT "rc_sbus"
/* end of L0_TASK_CONFIG */

/* L1_MIDDLEWARE_CONFIG */

#define L1_MIDDLEWARE_01_MODULE_01_FLOATCONVER_EN
#define L1_MIDDLEWARE_01_MODULE_02_MATHS_EN
#define L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_EN
#define L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_EN
#define L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_0_PIN "PD13"
#define L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_EN
#define L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_1_PIN "PD14"
#define L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_TESTSELF_EN
#define L1_MIDDLEWARE_01_MODULE_04_PID_MINIFLY_EN
/* end of L1_MIDDLEWARE_CONFIG */

/* L2_DEVICE_CONFIG */

/* 01 IMU CONFIG */

/* 01 ICM42688_130 CONFIG */

/* end of 01 ICM42688_130 CONFIG */

/* 02 MPU6500_minifly CONFIG */

/* end of 02 MPU6500_minifly CONFIG */

/* 03 MPU6000_FMUV2 CONFIG */

#define BSP_USING_MPU6000
#define SENSOR_NAME_MPU6000 "mpu6000"
#define SENSOR_MPU6000_ACC_RANGE_G 8
#define SENSOR_MPU6000_GYRO_RANGE_DPS 2000
#define SENSOR_SPI_NAME_MPU6000 "spi1"
#define SENSOR_SPI_SLAVE_NAME_MPU6000 "spi10"
#define SENSOR_MPU6000_SPI_CS_PIN_NAME "PC2"
#define SENSOR_MPU6000_SPI_MAX_HZ 4000000
/* end of 03 MPU6000_FMUV2 CONFIG */
/* end of 01 IMU CONFIG */

/* 02 RC CONFIG */

/* 01 SBUS CONFIG */

#define RC_USING_SBUS
#define RC_SBUS_UART_NAME "uart3"
#define RC_SBUS_BAUDRATE 100000
#define RC_SBUS_INVERTED
#define RC_SBUS_DEBUGPIN_EN
/* end of 01 SBUS CONFIG */

/* 02 CRSF CONFIG */

/* end of 02 CRSF CONFIG */
/* end of 02 RC CONFIG */

/* 03 MOTOR CONFIG */

/* 01 PWM CONFIG(Hollow slab) */

/* end of 01 PWM CONFIG(Hollow slab) */

/* 02 DSHOT CONFIG */

/* end of 02 DSHOT CONFIG */

/* 03 PWM CONFIG */

#define L2_DEVICE_03_MOTOR_03_PWM_EN
#define L2_DEVICE_03_MOTOR_03_PWM_CHANNEL_REMAP "4321"
#define L2_DEVICE_03_MOTOR_03_PWM_CMD_EN
/* end of 03 PWM CONFIG */
/* end of 03 MOTOR CONFIG */

/* 04 BARO CONFIG */

/* 01 SPL16 CONFIG */

/* end of 01 SPL16 CONFIG */

/* 02 SPA06 CONFIG */

/* end of 02 SPA06 CONFIG */

/* 03 DPS3XX CONFIG */

/* end of 03 DPS3XX CONFIG */
/* end of 04 BARO CONFIG */
/* end of L2_DEVICE_CONFIG */

/* L3_PERIPHERAL_CONFIG */

#define L3_PERIPHERAL_01_HSE_CONFIG_EN
#define MCU_HSE_24MHZ
#define HSE_VALUE 24000000
#define L3_PERIPHERAL_02_NVIC_CONFIG_EN
#define L3_PERIPHERAL_02_NVIC_STM32_PRIORITY_GROUP 0
#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART2
#define BSP_UART2_RX_USING_DMA
#define BSP_UART2_TX_USING_DMA
#define BSP_UART2_RX_BUFSIZE 256
#define BSP_UART2_TX_BUFSIZE 64
#define BSP_UART2_DMA_PING_BUFSIZE 64
#define BSP_USING_UART3
#define BSP_UART3_RX_USING_DMA
#define BSP_UART3_TX_USING_DMA
#define BSP_UART3_RX_BUFSIZE 64
#define BSP_UART3_TX_BUFSIZE 64
#define BSP_UART3_DMA_PING_BUFSIZE 32
#define BSP_USING_SPI
#define BSP_USING_SPI1
#define BSP_USING_USBD
#define BSP_USING_PWM
#define BSP_USING_PWM1
#define BSP_USING_PWM1_CH1
#define BSP_USING_PWM1_CH2
#define BSP_USING_PWM1_CH3
#define BSP_USING_PWM1_CH4
#define BSP_USING_ON_CHIP_FLASH
#define L3_PERIPHERAL_USING_ON_CHIP_FLASH_AUTOSTART_FALINIT
/* end of L3_PERIPHERAL_CONFIG */
/* end of DriverFramework Config */

#endif
