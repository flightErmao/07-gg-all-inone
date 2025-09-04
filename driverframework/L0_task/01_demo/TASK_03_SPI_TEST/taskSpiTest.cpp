#include <rtdevice.h>
#include <rtthread.h>
#include "spi_interface.hpp"
#include "rtconfig.h"

#ifdef TASK_DEMO_03_SPI_TEST_DEBUGPIN_EN
#include "debugPin.h"
#endif

#define THREAD_PRIORITY 7
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE 5

// IMU42688 WHO_AM_I register address
#define IMU42688_WHO_AM_I_REG 0x75

// Global SPI interface instance
static SpiInterface g_spi_;

// SPI wrapper functions for IMU42688 communication
// static int write_reg_wrap(uint8_t reg, uint8_t val) { 
//     return g_spi_.write_reg(reg, val); 
// }

static int read_multi_wrap(uint8_t reg, uint8_t *buff, uint8_t len) { 
    return g_spi_.read_multi(reg, buff, len); 
}

// static void delay_ms_wrap(unsigned int ms) { 
//     rt_thread_mdelay(ms); 
// }

/**
 * @brief SPI test task function
 * 
 * This task demonstrates SPI communication with IMU42688 sensor:
 * 1. Initialize SPI interface
 * 2. Configure SPI parameters
 * 3. Read WHO_AM_I register
 * 4. Print the result
 */
void spiTestTask(void *param) {
    uint8_t who_am_i_data = 0;
    int ret;
    
    rt_kprintf("SPI Test Task Started\n");
    
    // Initialize SPI interface
    if (!g_spi_.init(TASK_DEMO_03_SPI_TEST_SPI_BUS_NAME, 
                     TASK_DEMO_03_SPI_TEST_SPI_SLAVE_NAME, 
                     TASK_DEMO_03_SPI_TEST_SPI_CS_PIN_NAME)) {
        rt_kprintf("SPI initialization failed!\n");
        return;
    }
    
    rt_kprintf("SPI interface initialized successfully\n");
    
    // Configure SPI parameters (Mode 3, MSB first)
    if (!g_spi_.configure((RT_SPI_MODE_3 | RT_SPI_MSB) & RT_SPI_MODE_MASK, 
                         TASK_DEMO_03_SPI_TEST_SPI_MAX_HZ)) {
        rt_kprintf("SPI configuration failed!\n");
        return;
    }
    
    rt_kprintf("SPI configured: Mode=3, MaxHz=%d\n", TASK_DEMO_03_SPI_TEST_SPI_MAX_HZ);
    
    // Wait a bit for sensor to be ready
    rt_thread_mdelay(100);
    
    while (1) {
        // Read WHO_AM_I register from IMU42688
        ret = read_multi_wrap(IMU42688_WHO_AM_I_REG, &who_am_i_data, 1);
        
        if (ret == 0) {
            rt_kprintf("IMU42688 WHO_AM_I register (0x%02X): 0x%02X\n", 
                      IMU42688_WHO_AM_I_REG, who_am_i_data);
            
            // Check if the value is correct (IMU42688 WHO_AM_I should be 0x47)
            if (who_am_i_data == 0x47) {
                rt_kprintf("IMU42688 sensor detected successfully!\n");
            } else {
                rt_kprintf("Warning: Unexpected WHO_AM_I value. Expected: 0x47, Got: 0x%02X\n", 
                          who_am_i_data);
            }
        } else {
            rt_kprintf("Failed to read WHO_AM_I register, error: %d\n", ret);
        }
        
        // Wait 5 seconds before next read
        rt_thread_mdelay(1000);
    }
}

/**
 * @brief Initialize SPI test task
 * 
 * This function creates and starts the SPI test thread
 */
static int taskSpiTestInit(void) {
    rt_thread_t thread;
    
    rt_kprintf("Initializing SPI Test Task...\n");
    
    // Create SPI test thread
    thread = rt_thread_create("spi_test", 
                             spiTestTask, 
                             RT_NULL, 
                             THREAD_STACK_SIZE, 
                             THREAD_PRIORITY, 
                             THREAD_TIMESLICE);
    
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
        rt_kprintf("SPI Test Task started successfully\n");
        rt_kprintf("SPI Bus: %s, Slave: %s, CS Pin: %s\n", 
                  TASK_DEMO_03_SPI_TEST_SPI_BUS_NAME,
                  TASK_DEMO_03_SPI_TEST_SPI_SLAVE_NAME,
                  TASK_DEMO_03_SPI_TEST_SPI_CS_PIN_NAME);
        return RT_EOK;
    } else {
        rt_kprintf("Failed to create SPI test thread!\n");
        return -RT_ERROR;
    }
}

// Auto-start the SPI test task
#ifdef TASK_DEMO_03_SPI_TEST_EN
INIT_APP_EXPORT(taskSpiTestInit);
#endif