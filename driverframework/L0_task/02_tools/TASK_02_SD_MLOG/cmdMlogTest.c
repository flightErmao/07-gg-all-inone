/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>
#include "taskMlogTest.h"

#ifdef RT_USING_FINSH
#ifdef TASK_TOOL_02_SD_MLOG_TEST

#define TAG "cmdMlogTest"

/* Main mlog test command with subcommands */
static void cmdMlogTest(int argc, char **argv) {
    if (argc < 2) {
        rt_kprintf("Usage: mlogtest <command> [options]\n");
        rt_kprintf("Commands:\n");
        rt_kprintf("  start    - Start mlog test data generation\n");
        rt_kprintf("  stop     - Stop mlog test data generation\n");
        rt_kprintf("  status   - Show mlog test status\n");
        rt_kprintf("  data     - Get latest test data\n");
        rt_kprintf("Examples:\n");
        rt_kprintf("  mlogtest start     # Start test data generation\n");
        rt_kprintf("  mlogtest stop      # Stop test data generation\n");
        rt_kprintf("  mlogtest status    # Show test status\n");
        rt_kprintf("  mlogtest data      # Display latest test data\n");
        return;
    }

    /* Parse subcommand */
    if (strcmp(argv[1], "start") == 0) {
        /* Start mlog test */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'start' command\n", TAG);
            rt_kprintf("Usage: mlogtest start\n");
            return;
        }
        
        rt_err_t result = mlog_test_start();
        if (result == RT_EOK) {
            rt_kprintf("[%s] Mlog test started successfully\n", TAG);
        } else {
            rt_kprintf("[%s] Failed to start mlog test: %d\n", TAG, result);
        }
        
    } else if (strcmp(argv[1], "stop") == 0) {
        /* Stop mlog test */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'stop' command\n", TAG);
            rt_kprintf("Usage: mlogtest stop\n");
            return;
        }
        
        mlog_test_stop();
        rt_kprintf("[%s] Mlog test stopped\n", TAG);
        
    } else if (strcmp(argv[1], "status") == 0) {
        /* Show mlog test status */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'status' command\n", TAG);
            rt_kprintf("Usage: mlogtest status\n");
            return;
        }
        
        rt_bool_t is_running = mlog_test_is_running();
        rt_kprintf("[%s] Mlog Test Status:\n", TAG);
        rt_kprintf("  Running: %s\n", is_running ? "Yes" : "No");
        rt_kprintf("  Frequency: %d Hz\n", MLOG_TEST_02_SD_MLOG_FREQ_HZ);
        
    } else if (strcmp(argv[1], "data") == 0) {
        /* Get latest test data */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'data' command\n", TAG);
            rt_kprintf("Usage: mlogtest data\n");
            return;
        }
        
        sensorData_t test_data;
        mlog_test_get_data(&test_data);
        
        rt_kprintf("[%s] Latest Test Data:\n", TAG);
        rt_kprintf("  Timestamp: %lu\n", test_data.timestamp);
        rt_kprintf("  Acc Raw: x=%d, y=%d, z=%d\n", 
                   test_data.acc_raw.x, test_data.acc_raw.y, test_data.acc_raw.z);
        rt_kprintf("  Gyro Raw: x=%d, y=%d, z=%d\n", 
                   test_data.gyro_raw.x, test_data.gyro_raw.y, test_data.gyro_raw.z);
        rt_kprintf("  Acc Filter: x=%.2f, y=%.2f, z=%.2f\n", 
                   test_data.acc_filter.x, test_data.acc_filter.y, test_data.acc_filter.z);
        rt_kprintf("  Gyro Filter: x=%.2f, y=%.2f, z=%.2f\n", 
                   test_data.gyro_filter.x, test_data.gyro_filter.y, test_data.gyro_filter.z);
        
    } else {
        /* Unknown command */
        rt_kprintf("[%s] Error: Unknown command '%s'\n", TAG, argv[1]);
        rt_kprintf("Available commands: start, stop, status, data\n");
        rt_kprintf("Use 'mlogtest' without arguments to see usage help\n");
    }
}

/* Export command to MSH */
MSH_CMD_EXPORT_ALIAS(cmdMlogTest, cmdMlogTest, mlog test data generation control command);

#endif /* TASK_TOOL_02_SD_MLOG_TEST */
#endif /* RT_USING_FINSH */
