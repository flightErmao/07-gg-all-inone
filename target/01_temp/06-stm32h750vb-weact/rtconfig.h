#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

#define SOC_STM32H750VBT6

/* RT-Thread Kernel */
#define RT_NAME_MAX 8
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

/* Memory Management */
#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_MEMHEAP_FAST_MODE
#define RT_USING_MEMHEAP_AS_HEAP
#define RT_USING_MEMHEAP_AUTO_BINDING
#define RT_USING_HEAP

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x50201
#define RT_BACKTRACE_LEVEL_MAX_NR 32
/* end of RT-Thread Kernel */

#define RT_USING_CACHE
#define RT_USING_HW_ATOMIC
#define RT_USING_CPU_FFS
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */
#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* Device Drivers */
#define RT_USING_DEVICE_IPC
#define RT_UNAMED_PIPE_NUMBER 64
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V2
#define RT_SERIAL_BUF_STRATEGY_OVERWRITE
#define RT_SERIAL_USING_DMA
#define RT_USING_PIN
/* end of Device Drivers */

/* C/C++ and POSIX layer */
#define RT_LIBC_USING_LIGHT_TZ_DST
#define RT_LIBC_TZ_DEFAULT_HOUR 8
#define RT_LIBC_TZ_DEFAULT_MIN 0
#define RT_LIBC_TZ_DEFAULT_SEC 0

#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32H7

/* Hardware Drivers Config */

/* On-chip Peripheral Drivers */
#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_UART1_RX_BUFSIZE 256
#define BSP_UART1_TX_BUFSIZE 256
/* end of On-chip Peripheral Drivers */

/* end of Hardware Drivers Config */

/* RT-Thread online packages */
#define PKG_USING_CMSIS_CORE
#define PKG_USING_CMSIS_CORE_LATEST_VERSION
#define PKG_USING_STM32H7_HAL_DRIVER
#define PKG_USING_STM32H7_HAL_DRIVER_LATEST_VERSION
#define PKG_USING_STM32H7_CMSIS_DRIVER
#define PKG_USING_STM32H7_CMSIS_DRIVER_LATEST_VERSION
/* end of RT-Thread online packages */

#endif
