#include "debugPin.h"
#include <stdio.h>
#include <rtthread.h>
#include <rtconfig.h>

#ifdef L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_TESTSELF_EN

/**
 * @brief Debug pin self test command handler
 * @param argc Number of arguments
 * @param argv Argument array
 * @return int Command execution result
 */
static int cmdPinTestSelf(int argc, char **argv)
{
    printf("[DEBUG_PIN_TEST] Starting debug pin self test...\n");
    
    // Initialize all pins to low level using exposed macros
    DEBUG_PIN_DEBUG0_LOW();
    DEBUG_PIN_DEBUG1_LOW();
    DEBUG_PIN_DEBUG2_LOW();
    DEBUG_PIN_DEBUG3_LOW();
    
    printf("[DEBUG_PIN_TEST] All pins initialized to low level\n");
    rt_thread_mdelay(500); // Wait 500ms
    
    // Test each pin in sequence using exposed macros
    printf("[DEBUG_PIN_TEST] Testing pin 0...\n");
    DEBUG_PIN_DEBUG0_HIGH();
    printf("[DEBUG_PIN_TEST] Pin 0 set to high level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    DEBUG_PIN_DEBUG0_LOW();
    printf("[DEBUG_PIN_TEST] Pin 0 set to low level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    
    printf("[DEBUG_PIN_TEST] Testing pin 1...\n");
    DEBUG_PIN_DEBUG1_HIGH();
    printf("[DEBUG_PIN_TEST] Pin 1 set to high level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    DEBUG_PIN_DEBUG1_LOW();
    printf("[DEBUG_PIN_TEST] Pin 1 set to low level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    
    printf("[DEBUG_PIN_TEST] Testing pin 2...\n");
    DEBUG_PIN_DEBUG2_HIGH();
    printf("[DEBUG_PIN_TEST] Pin 2 set to high level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    DEBUG_PIN_DEBUG2_LOW();
    printf("[DEBUG_PIN_TEST] Pin 2 set to low level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    
    printf("[DEBUG_PIN_TEST] Testing pin 3...\n");
    DEBUG_PIN_DEBUG3_HIGH();
    printf("[DEBUG_PIN_TEST] Pin 3 set to high level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    DEBUG_PIN_DEBUG3_LOW();
    printf("[DEBUG_PIN_TEST] Pin 3 set to low level\n");
    rt_thread_mdelay(1000); // Wait 1 second
    
    printf("[DEBUG_PIN_TEST] Debug pin self test completed!\n");
    return 0;
}

// Register command
MSH_CMD_EXPORT_ALIAS(cmdPinTestSelf, cmdPinTestSelf, Debug pin self test);

#endif // L1_MIDDLEWARE_01_MODULE_03_DEBUGPIN_TESTSELF_EN
