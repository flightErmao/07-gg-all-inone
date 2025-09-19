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
#include "taskMlog.h"
#include "fileManager.h"

#ifdef RT_USING_FINSH

#define TAG "cmdMlog"

/* Main mlog command with subcommands */
static void cmdMlog(int argc, char **argv) {
    if (argc < 2) {
        rt_kprintf("Usage: mlog <command> [options]\n");
        rt_kprintf("Commands:\n");
        rt_kprintf("  start [file_path]  - Start mlog logging\n");
        rt_kprintf("  stop               - Stop mlog logging\n");
        rt_kprintf("  status             - Show mlog status and statistics\n");
        rt_kprintf("  session            - Show working log session path\n");
        rt_kprintf("Examples:\n");
        rt_kprintf("  mlog start                    # Start logging with auto-generated path\n");
        rt_kprintf("  mlog start /log/test.bin      # Start logging to specific file\n");
        rt_kprintf("  mlog stop                     # Stop current logging\n");
        rt_kprintf("  mlog status                   # Show status and statistics\n");
        rt_kprintf("  mlog session                  # Show working session path\n");
        return;
    }

    /* Parse subcommand */
    if (strcmp(argv[1], "start") == 0) {
        /* Start mlog logging */
        char* file_path = NULL;
        
        if (argc > 3) {
            rt_kprintf("[%s] Error: Too many arguments for 'start' command\n", TAG);
            rt_kprintf("Usage: mlog start [file_path]\n");
            return;
        }
        
        /* Get file path if provided */
        if (argc == 3) {
            file_path = argv[2];
        }

        /* Start mlog logging */
        rt_err_t result = task_mlog_start_logging(file_path);
        if (result == RT_EOK) {
            rt_kprintf("[%s] Mlog started successfully\n", TAG);
        } else {
            rt_kprintf("[%s] Failed to start mlog: %d\n", TAG, result);
        }
        
    } else if (strcmp(argv[1], "stop") == 0) {
        /* Stop mlog logging */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'stop' command\n", TAG);
            rt_kprintf("Usage: mlog stop\n");
            return;
        }
        
        task_mlog_stop_logging();
        rt_kprintf("[%s] Mlog stopped\n", TAG);
        
    } else if (strcmp(argv[1], "status") == 0) {
        /* Show mlog status */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'status' command\n", TAG);
            rt_kprintf("Usage: mlog status\n");
            return;
        }
        
        /* Get and display mlog status */
        uint8_t status = task_mlog_get_status();
        char* file_name = task_mlog_get_file_name();

        rt_kprintf("[%s] Mlog Status:\n", TAG);
        rt_kprintf("  Status: %d\n", status);
        
        if (status != 0) {  // Not idle
            rt_kprintf("  File: %s\n", file_name ? file_name : "Unknown");
            rt_kprintf("  Statistics:\n");
            task_mlog_show_statistics();
        } else {
            rt_kprintf("  No active logging session\n");
        }
        
    } else if (strcmp(argv[1], "session") == 0) {
        /* Show working session */
        if (argc > 2) {
            rt_kprintf("[%s] Error: Too many arguments for 'session' command\n", TAG);
            rt_kprintf("Usage: mlog session\n");
            return;
        }
        
        char path[100];
        rt_err_t result = current_log_session(path);
        
        if (result == RT_EOK) {
            rt_kprintf("[%s] Working log session: %s\n", TAG, path);
        } else {
            rt_kprintf("[%s] No available log session\n", TAG);
        }
        
    } else {
        /* Unknown command */
        rt_kprintf("[%s] Error: Unknown command '%s'\n", TAG, argv[1]);
        rt_kprintf("Available commands: start, stop, status, session\n");
        rt_kprintf("Use 'mlog' without arguments to see usage help\n");
    }
}

/* Export command to MSH */
MSH_CMD_EXPORT_ALIAS(cmdMlog, cmdMlog, mlog logging control command);
#endif /* RT_USING_FINSH */