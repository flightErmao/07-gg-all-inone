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

// IMU42688 register addresses
#define IMU42688_WHO_AM_I_REG 0x75
#define IMU42688_ACCEL_CONFIG_REG 0x14

// IMU42688 accelerometer range values
#define IMU42688_ACCEL_RANGE_2G 0x00   // ±2g
#define IMU42688_ACCEL_RANGE_4G 0x01   // ±4g
#define IMU42688_ACCEL_RANGE_8G 0x02   // ±8g
#define IMU42688_ACCEL_RANGE_16G 0x03  // ±16g

// Global SPI interface instance
static SpiInterface g_spi_;

// SPI wrapper functions for IMU42688 communication
static int write_reg_wrap(uint8_t reg, uint8_t val) { return g_spi_.write_reg(reg, val); }

static void delay_ms_wrap(unsigned int ms) { rt_thread_mdelay(ms); }

static int read_multi_wrap(uint8_t reg, uint8_t *buff, uint8_t len) { return g_spi_.readMultiReg8(reg, buff, len); }

/**
 * @brief Test accelerometer range configuration
 *
 * This function tests writing and reading accelerometer range configuration
 * to verify SPI write/read functionality
 */
static int test_accel_range_config(uint8_t range_value, const char *range_name) {
  uint8_t read_value = 0;
  int ret;

  rt_kprintf("\n=== Testing ACC Range: %s ===\n", range_name);

  // Write range configuration
  ret = write_reg_wrap(IMU42688_ACCEL_CONFIG_REG, range_value);
  if (ret != 0) {
    rt_kprintf("Failed to write ACC range config, error: %d\n", ret);
    return ret;
  }
  rt_kprintf("Written ACC range config: 0x%02X (%s)\n", range_value, range_name);

  // Use delay to ensure write operation completes
  delay_ms_wrap(10);

  // Read back the configuration
  ret = read_multi_wrap(IMU42688_ACCEL_CONFIG_REG, &read_value, 1);
  if (ret != 0) {
    rt_kprintf("Failed to read ACC range config, error: %d\n", ret);
    return ret;
  }
  rt_kprintf("Read back ACC range config: 0x%02X\n", read_value);

  // Verify the values match
  if (read_value == range_value) {
    rt_kprintf("✓ ACC range config verification PASSED\n");
    return 0;
  } else {
    rt_kprintf("✗ ACC range config verification FAILED\n");
    rt_kprintf("  Expected: 0x%02X, Got: 0x%02X\n", range_value, read_value);
    return -1;
  }
}

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
    int test_cycle = 0;

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
    delay_ms_wrap(100);

    while (1) {
      rt_kprintf("\n========== SPI Test Cycle %d ==========\n", ++test_cycle);

      // Step 1: Read WHO_AM_I register from IMU42688
      rt_kprintf("\n--- Step 1: WHO_AM_I Register Test ---\n");
      ret = read_multi_wrap(IMU42688_WHO_AM_I_REG, &who_am_i_data, 1);

      if (ret == 0) {
        rt_kprintf("IMU42688 WHO_AM_I register (0x%02X): 0x%02X\n", IMU42688_WHO_AM_I_REG, who_am_i_data);

        // Check if the value is correct (IMU42688 WHO_AM_I should be 0x47)
        if (who_am_i_data == 0x47) {
          rt_kprintf("✓ IMU42688 sensor detected successfully!\n");
        } else {
          rt_kprintf("✗ Warning: Unexpected WHO_AM_I value. Expected: 0x47, Got: 0x%02X\n", who_am_i_data);
        }
      } else {
        rt_kprintf("✗ Failed to read WHO_AM_I register, error: %d\n", ret);
      }

      // Step 2: Test ACC range configuration (4G)
      rt_kprintf("\n--- Step 2: ACC Range Configuration Test (4G) ---\n");
      ret = test_accel_range_config(IMU42688_ACCEL_RANGE_4G, "±4G");
      if (ret != 0) {
        rt_kprintf("✗ ACC 4G range test failed\n");
      }

      // Wait between tests
      delay_ms_wrap(50);

      // Step 3: Test ACC range configuration (8G)
      rt_kprintf("\n--- Step 3: ACC Range Configuration Test (8G) ---\n");
      ret = test_accel_range_config(IMU42688_ACCEL_RANGE_8G, "±8G");
      if (ret != 0) {
        rt_kprintf("✗ ACC 8G range test failed\n");
      }

      //   // Step 4: Test ACC range configuration (2G)
      //   rt_kprintf("\n--- Step 4: ACC Range Configuration Test (2G) ---\n");
      //   ret = test_accel_range_config(IMU42688_ACCEL_RANGE_2G, "±2G");
      //   if (ret != 0) {
      //     rt_kprintf("✗ ACC 2G range test failed\n");
      //   }

      //   // Step 5: Test ACC range configuration (16G)
      //   rt_kprintf("\n--- Step 5: ACC Range Configuration Test (16G) ---\n");
      //   ret = test_accel_range_config(IMU42688_ACCEL_RANGE_16G, "±16G");
      //   if (ret != 0) {
      //     rt_kprintf("✗ ACC 16G range test failed\n");
      //   }

      //   rt_kprintf("\n========== Test Cycle %d Completed ==========\n", test_cycle);
      //   rt_kprintf("All SPI interface functions tested:\n");
      //   rt_kprintf("  - SPI read operation (WHO_AM_I)\n");
      //   rt_kprintf("  - SPI write operation (ACC config)\n");
      //   rt_kprintf("  - SPI read-back verification\n");
      //   rt_kprintf("  - Delay function verification\n");

      // Wait 10 seconds before next test cycle
      delay_ms_wrap(1000);
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