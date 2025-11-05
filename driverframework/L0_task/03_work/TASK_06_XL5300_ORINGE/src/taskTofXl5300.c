#include <rtdevice.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <string.h>
#include <stdarg.h>

#include "I2cInterface.h"
#include "../inc/taskTofXl5300.h"
#include "../inc/VI530x_User_Handle.h"
#include "../inc/VI530x_API.h"
#include "../inc/VI530x_Firmware.h"
#include "../inc/VI530x_System_Data.h"

#ifdef WORK_TASK_TOF_XL5300_ORINGE_UART_EN
#include "../../../../L3_peripheral/11_uartConfig/uartConfig.h"
#endif

/* Static variables */
static I2cInterface_t g_i2c_interface;
static struct rt_event g_tof_int_event;
static rt_bool_t g_tof_event_inited = RT_FALSE;

#ifdef WORK_TASK_TOF_XL5300_ORINGE_UART_EN
/* UART device for data output */
static rt_device_t g_tof_uart = RT_NULL;
static struct serial_configure g_uart_cfg;
#endif

/* I2C读写函数实现（供VI530x_User_Handle.c调用） */
uint8_t IIC_Write_X_Bytes(uint8_t dev_addr, uint8_t addr, uint8_t *pValue, uint16_t tlen)
{
    int8_t ret = i2c_write_reg8_mult_pack(g_i2c_interface, addr, pValue, tlen);
    return (ret == 0) ? 0 : 1;
}

uint8_t IIC_Read_X_Bytes(uint8_t dev_addr, uint8_t addr, uint8_t *value, uint16_t tlen)
{
    int8_t ret = i2c_read_reg8_mult_pack(g_i2c_interface, addr, value, tlen);
    return (ret == 0) ? 0 : 1;
}

/* 延时函数实现 */
void VI530x_Delay_Ms(uint16_t nMs)
{
    rt_thread_mdelay(nMs);
}

/* XSHUT引脚控制实现 */
void VI530x_XSHUT_Enable(uint8_t state)
{
    if (state) {
        rt_pin_write(XSHUT_PIN_NUM, PIN_HIGH);
    } else {
        rt_pin_write(XSHUT_PIN_NUM, PIN_LOW);
    }
}

/* GPIO中断处理函数（供VI530x_User_Handle.c调用） */
void VI530x_GPIO_Interrupt_Handle(void)
{
    if(VI530x_Cali_Data.VI530x_Interrupt_Mode_Status)
    {
        VI530x_GPIO_Interrupt_status = 1;
        if (g_tof_event_inited) {
            rt_event_send(&g_tof_int_event, TOF_INT_EVENT_FLAG);
        }
    }
}

/* GPIO中断回调函数 */
static void tof_int_callback(void *args)
{
    VI530x_GPIO_Interrupt_Handle();
}

/* GPIO初始化 */
rt_err_t tof_gpio_init(void)
{
#ifdef WORK_TASK_TOF_XL5300_ORINGE_USE_INTERRUPT
    /* 初始化事件（仅在中断模式下需要） */
    if (!g_tof_event_inited) {
        if (rt_event_init(&g_tof_int_event, "tof_ie", RT_IPC_FLAG_FIFO) != RT_EOK) {
            return -RT_ERROR;
        }
        g_tof_event_inited = RT_TRUE;
    }
#endif

    /* 配置XSHUT引脚（PB6）为输出，上拉 */
    rt_pin_mode(XSHUT_PIN_NUM, PIN_MODE_OUTPUT);
    /* 初始状态：XSHUT拉低（复位） */
    rt_pin_write(XSHUT_PIN_NUM, PIN_LOW);

#ifdef WORK_TASK_TOF_XL5300_ORINGE_USE_INTERRUPT
    /* 配置中断引脚（PB7）为输入，下拉，改为上升沿中断（参考 mpu6500） */
    rt_pin_mode(INT_PIN_NUM, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_attach_irq(INT_PIN_NUM, PIN_IRQ_MODE_RISING, tof_int_callback, RT_NULL);
    rt_pin_irq_enable(INT_PIN_NUM, PIN_IRQ_ENABLE);
#endif
    
    return RT_EOK;
}

/* UART初始化 */
#ifdef WORK_TASK_TOF_XL5300_ORINGE_UART_EN
static rt_err_t tof_uart_init(void)
{
    rt_err_t result;
    const char* uart_name = WORK_TASK_TOF_XL5300_ORINGE_UART_NAME;
    
    /* 查找UART设备 */
    g_tof_uart = rt_device_find(uart_name);
    if (!g_tof_uart) {
        rt_kprintf("[TOF_XL5300] find uart %s fail\n", uart_name);
        return -RT_ERROR;
    }
    
    /* 打开UART设备 */
    result = rt_device_open(g_tof_uart, RT_DEVICE_FLAG_TX_BLOCKING);
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] open uart %s fail\n", uart_name);
        return result;
    }
    
    /* 配置UART参数 */
    result = uart_config_by_device_name(uart_name, WORK_TASK_TOF_XL5300_ORINGE_UART_BAUD, &g_uart_cfg);
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] get uart cfg fail\n");
        return result;
    }
    
    result = rt_device_control(g_tof_uart, RT_DEVICE_CTRL_CONFIG, &g_uart_cfg);
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] config uart fail\n");
        return result;
    }
    
    rt_kprintf("[TOF_XL5300] UART %s initialized, baud:%d\n", uart_name, WORK_TASK_TOF_XL5300_ORINGE_UART_BAUD);
    return RT_EOK;
}

/* 通过UART发送数据 */
static void tof_uart_printf(const char* format, ...)
{
    if (!g_tof_uart) {
        return;
    }
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = rt_vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < (int)sizeof(buffer)) {
        rt_device_write(g_tof_uart, 0, buffer, len);
    }
}
#endif

/* I2C初始化 */
rt_err_t tof_i2c_init(void)
{
    rt_err_t result = get_i2c_interface(WORK_TASK_TOF_XL5300_ORINGE_I2C_NAME, VI530x_I2C_ADDR, &g_i2c_interface);
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] get i2c interface fail\n");
        return result;
    }
    
    /* 设置I2C速度为1MHz（支持1MHz） */
    set_i2c_fast_plus_speed(g_i2c_interface.i2c_dev);
    
    return RT_EOK;
}

/* VI530x初始化主函数（移植自VI530x_main） */
void vi530x_init_and_run(void)
{
    VI530x_Status ret = VI530x_OK;
    VI530x_MEASURE_TypeDef result;
    
    rt_kprintf("[TOF_XL5300] Starting initialization...\n");
    
    // 1、IIC 初始化（已在tof_i2c_init中完成）
    
    // 2、GPIO 初始化（已在tof_gpio_init中完成）

    // 3、选择中断方式：0x88----GPIO硬件中断，0x00----寄存器查询
#ifdef WORK_TASK_TOF_XL5300_ORINGE_USE_INTERRUPT
    VI530x_Cali_Data.VI530x_Interrupt_Mode_Status = VI530x_INTERRUPT_MODE_HW;  // GPIO引脚启用，硬件中断
#else
    VI530x_Cali_Data.VI530x_Interrupt_Mode_Status = VI530x_INTERRUPT_MODE_SW;  // 寄存器查询模式
#endif

    // 4、VI530x初始化，选择复位方式
    VI530x_Chip_PowerON();  // Xshut引脚启用，硬件复位/使能，**建议方式**
    ret |= VI530x_Chip_Init();
    
    // 5、VI530x固件写入，系统参数配置
    ret |= VI530x_Download_Firmware((uint8_t *)VI5300_M31_firmware_buff, FirmwareSize());
    ret |= VI530x_Set_Integralcounts_Frame(20, 321000);  // 帧率，积分次数

    // 7、其它配置
    ret |= VI530x_Set_Sys_Temperature_Enable(0x01);
    
    // 8、开启测距
    ret |= VI530x_Start_Continue_Ranging_Cmd();  // 连续模式
    
    if (ret) {
        rt_kprintf("[TOF_XL5300] Config Error! ret = 0x%04X\n", ret);
    } else {
        rt_kprintf("[TOF_XL5300] Config OK!\n");
    }
    
    // 9、循环读取测距数据
    while (1) {
#ifdef WORK_TASK_TOF_XL5300_ORINGE_USE_INTERRUPT
        // 采用事件方式等待中断到来（参考 mpu6500）
        if (g_tof_event_inited) {
            rt_event_recv(&g_tof_int_event, TOF_INT_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, RT_NULL);
        }
#endif
        ret = VI530x_Get_Measure_Data(&result, 1);
        if (!ret) {
#ifdef WORK_TASK_TOF_XL5300_ORINGE_UART_EN
            /* 通过UART输出数据 */
            tof_uart_printf("TOF[%4d] C[%3d] P[%4lu] N[%4lu] I[%4lu]\r\n",
                           result.correction_tof, result.confidence, 
                           (unsigned long)result.peak, (unsigned long)result.noise,
                           (unsigned long)result.intecounts);
#else
            /* 通过rt_kprintf输出数据 */
            rt_kprintf("[TOF_XL5300] tof = %4d mm, confidence = %3d, peak = %4lu, noise = %4lu, intecounts = %4lu\n",
                       result.correction_tof, result.confidence, (unsigned long)result.peak, (unsigned long)result.noise,
                       (unsigned long)result.intecounts);
#endif
        }

#ifdef WORK_TASK_TOF_XL5300_ORINGE_USE_INTERRUPT
        // 中断模式下不需要延时，等待中断即可
#else
        // 软件查询模式下需要延时
        rt_thread_mdelay(5);
#endif
    }
}

/* 任务线程入口 */
void tof_thread_entry(void* parameter)
{
    rt_err_t result;
    
    /* 初始化I2C */
    result = tof_i2c_init();
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] I2C init failed\n");
        return;
    }
    
    /* 初始化GPIO */
    result = tof_gpio_init();
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] GPIO init failed\n");
        return;
    }
    
#ifdef WORK_TASK_TOF_XL5300_ORINGE_UART_EN
    /* 初始化UART */
    result = tof_uart_init();
    if (result != RT_EOK) {
        rt_kprintf("[TOF_XL5300] UART init failed, will use rt_kprintf\n");
    }
#endif
    
    /* 延时等待硬件稳定 */
    rt_thread_mdelay(10);
    
    /* 运行VI530x初始化 */
    vi530x_init_and_run();
}

/* Global thread handle for command control */
rt_thread_t tof_task_thread = RT_NULL;

/* 任务初始化 */
static int tof_task_init(void)
{
#ifdef WORK_TASK_TOF_XL5300_ORINGE_AUTO_START
    rt_thread_t thread = rt_thread_create("tof_xl5300", tof_thread_entry, RT_NULL,
                                           THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (thread != RT_NULL) {
        tof_task_thread = thread;
        rt_thread_startup(thread);
        rt_kprintf("[TOF_XL5300] thread started automatically, i2c:%s, addr:0x%02X\n",
                   WORK_TASK_TOF_XL5300_ORINGE_I2C_NAME, VI530x_I2C_ADDR);
    } else {
        rt_kprintf("[TOF_XL5300] thread create fail\n");
        return -RT_ERROR;
    }
#else
    rt_kprintf("[TOF_XL5300] Task initialized (manual start via 'tof start')\n");
    rt_kprintf("[TOF_XL5300] i2c:%s, addr:0x%02X\n",
               WORK_TASK_TOF_XL5300_ORINGE_I2C_NAME, VI530x_I2C_ADDR);
#endif
    
    return RT_EOK;
}

#ifdef WORK_TASK_TOF_XL5300_ORINGE_EN
INIT_APP_EXPORT(tof_task_init);
#endif


