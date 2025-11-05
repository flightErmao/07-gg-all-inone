/****************************************************************************
 *
 * TOF XL5300 Command Interface
 * Provides calibration and control commands via MSH
 *
 ****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "../inc/taskTofXl5300.h"
#include "../inc/VI530x_User_Handle.h"
#include "../inc/VI530x_API.h"
#include "../inc/VI530x_Firmware.h"
#include "../inc/VI530x_System_Data.h"

#ifdef RT_USING_FINSH

#define TAG "TOF_XL5300"

/* Forward declarations */
extern rt_thread_t tof_task_thread;
extern void tof_thread_entry(void* parameter);

/* Start task command */
static void cmd_tof_start(int argc, char **argv)
{
    /* Check if thread already exists and is running */
    rt_thread_t thread = rt_thread_find("tof_xl5300");
    if (thread != RT_NULL) {
        rt_kprintf("[%s] Task already running\n", TAG);
        return;
    }

    rt_kprintf("[%s] Starting TOF task...\n", TAG);
    
    /* Create and start thread */
    thread = rt_thread_create("tof_xl5300", tof_thread_entry, RT_NULL,
                              THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (thread != RT_NULL) {
        tof_task_thread = thread;
        rt_thread_startup(thread);
        rt_kprintf("[%s] Task thread started\n", TAG);
    } else {
        rt_kprintf("[%s] Failed to create task thread\n", TAG);
    }
}

#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_REFTOF_CALIB
/* RefToF calibration command */
static void cmd_tof_reftof_calib(int argc, char **argv)
{
    rt_kprintf("[%s] Starting RefToF calibration...\n", TAG);
    rt_kprintf("[%s] Environment requirement: Room temperature\n", TAG);
    
    VI530x_Status ret = VI530x_Reftof_Calibration();
    if (ret == VI530x_OK) {
        rt_kprintf("[%s] RefToF Calibration OK!\n", TAG);
        rt_kprintf("[%s] VI530x_Calibration_Reftof = %4d\n", TAG, VI530x_Cali_Data.VI530x_Calibration_Reftof);
    } else {
        rt_kprintf("[%s] RefToF Calibration Failed! Error: 0x%04X\n", TAG, ret);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_tof_reftof_calib, tof_reftof, RefToF calibration);
#endif

#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_XTALK_CALIB
/* Xtalk calibration command */
static void cmd_tof_xtalk_calib(int argc, char **argv)
{
    rt_kprintf("[%s] Starting Xtalk calibration...\n", TAG);
    rt_kprintf("[%s] Environment requirement: No target within 60cm\n", TAG);
    
    VI530x_Status ret = VI530x_Xtalk_Calibration();
    if (ret == VI530x_OK) {
        rt_kprintf("[%s] Xtalk Calibration OK!\n", TAG);
        rt_kprintf("[%s] VI530x_Calibration_CG_Pos = %d\n", TAG, VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
        rt_kprintf("[%s] VI530x_Calibration_CG_Maxratio = %d\n", TAG, VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
        
        /* Xtalk control check */
        if (VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio > 15) {
            rt_kprintf("[%s] WARNING: Xtalk = %d is too large!\n", TAG, VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
        }
    } else {
        rt_kprintf("[%s] Xtalk Calibration Failed! Error: 0x%04X\n", TAG, ret);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_tof_xtalk_calib, tof_xtalk, Xtalk calibration);
#endif

#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_OFFSET_CALIB
/* Offset calibration command */
static void cmd_tof_offset_calib(int argc, char **argv)
{
    uint16_t distance = VI530x_OFFSET_DISTANCE;  // Default distance
    
    if (argc > 1) {
        distance = atoi(argv[1]);
        if (distance == 0) {
            rt_kprintf("[%s] Invalid distance parameter, using default: %d mm\n", TAG, VI530x_OFFSET_DISTANCE);
            distance = VI530x_OFFSET_DISTANCE;
        }
    }
    
    rt_kprintf("[%s] Starting Offset calibration at %d mm...\n", TAG, distance);
    rt_kprintf("[%s] Please place target at specified distance\n", TAG);
    
    VI530x_Status ret = VI530x_Offset_Calibration(distance);
    if (ret == VI530x_OK) {
        rt_kprintf("[%s] Offset Calibration OK!\n", TAG);
        rt_kprintf("[%s] VI530x_Calibration_Offset = %f\n", TAG, VI530x_Cali_Data.VI530x_Calibration_Offset);
    } else {
        rt_kprintf("[%s] Offset Calibration Failed! Error: 0x%04X\n", TAG, ret);
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_tof_offset_calib, tof_offset, Offset calibration [distance_mm]);
#endif

#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_GRADIENT_CALIB
/* Gradient calibration command */
static void cmd_tof_gradient_calib(int argc, char **argv)
{
#ifdef VI530x_GRADIENTK_CALIBRATION
    uint16_t offset_distance = VI530x_OFFSET_DISTANCE;
    uint16_t gradient_distance = VI530x_GRADIENT_DISTANCE;
    
    if (argc > 1) {
        offset_distance = atoi(argv[1]);
    }
    if (argc > 2) {
        gradient_distance = atoi(argv[2]);
    }
    
    if (offset_distance == gradient_distance) {
        rt_kprintf("[%s] Error: Offset distance (%d) cannot equal gradient distance (%d)\n", 
                   TAG, offset_distance, gradient_distance);
        return;
    }
    
    rt_kprintf("[%s] Starting Gradient calibration...\n", TAG);
    rt_kprintf("[%s] Offset distance: %d mm, Gradient distance: %d mm\n", TAG, offset_distance, gradient_distance);
    rt_kprintf("[%s] Note: Offset calibration must be done first\n", TAG);
    
    VI530x_Status ret = VI530x_GradientK_Calibration(offset_distance, gradient_distance);
    if (ret == VI530x_OK) {
        rt_kprintf("[%s] Gradient Calibration OK!\n", TAG);
        rt_kprintf("[%s] VI530x_Calibration_GradientK = %f\n", TAG, VI530x_Cali_Data.VI530x_Calibration_GradientK);
        rt_kprintf("[%s] VI530x_Calibration_Offset = %f\n", TAG, VI530x_Cali_Data.VI530x_Calibration_Offset);
    } else {
        rt_kprintf("[%s] Gradient Calibration Failed! Error: 0x%04X\n", TAG, ret);
    }
#else
    rt_kprintf("[%s] Gradient calibration is not enabled in configuration\n", TAG);
#endif
}
MSH_CMD_EXPORT_ALIAS(cmd_tof_gradient_calib, tof_gradient, Gradient calibration [offset_mm] [gradient_mm]);
#endif

/* Show calibration data command */
static void cmd_tof_show_calib(int argc, char **argv)
{
    rt_kprintf("[%s] Calibration Data:\n", TAG);
    rt_kprintf("  CG_Pos:        %d\n", VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
    rt_kprintf("  CG_Maxratio:   %d\n", VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
    rt_kprintf("  CG_Peak:       %d\n", VI530x_Cali_Data.VI530x_Calibration_CG_peak);
    rt_kprintf("  Reftof:        %d\n", VI530x_Cali_Data.VI530x_Calibration_Reftof);
    rt_kprintf("  Offset:        %f\n", VI530x_Cali_Data.VI530x_Calibration_Offset);
#ifdef VI530x_GRADIENTK_CALIBRATION
    rt_kprintf("  GradientK:     %f\n", VI530x_Cali_Data.VI530x_Calibration_GradientK);
#endif
}
MSH_CMD_EXPORT_ALIAS(cmd_tof_show_calib, tof_show, Show calibration data);

/* Main TOF command with subcommands */
static void cmd_tof(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: tof <command> [options]\n");
        rt_kprintf("Commands:\n");
        rt_kprintf("  start              - Start TOF task\n");
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_REFTOF_CALIB
        rt_kprintf("  reftof             - RefToF calibration\n");
#endif
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_XTALK_CALIB
        rt_kprintf("  xtalk              - Xtalk calibration\n");
#endif
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_OFFSET_CALIB
        rt_kprintf("  offset [distance]  - Offset calibration (default: %d mm)\n", VI530x_OFFSET_DISTANCE);
#endif
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_GRADIENT_CALIB
        rt_kprintf("  gradient [offset] [gradient] - Gradient calibration\n");
#endif
        rt_kprintf("  show               - Show calibration data\n");
        rt_kprintf("Examples:\n");
        rt_kprintf("  tof start                    # Start TOF task\n");
        rt_kprintf("  tof reftof                   # RefToF calibration\n");
        rt_kprintf("  tof xtalk                    # Xtalk calibration\n");
        rt_kprintf("  tof offset 600              # Offset calibration at 600mm\n");
        rt_kprintf("  tof show                     # Show calibration data\n");
        return;
    }

    if (strcmp(argv[1], "start") == 0) {
        cmd_tof_start(argc, argv);
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_REFTOF_CALIB
    } else if (strcmp(argv[1], "reftof") == 0) {
        cmd_tof_reftof_calib(argc, argv);
#endif
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_XTALK_CALIB
    } else if (strcmp(argv[1], "xtalk") == 0) {
        cmd_tof_xtalk_calib(argc, argv);
#endif
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_OFFSET_CALIB
    } else if (strcmp(argv[1], "offset") == 0) {
        cmd_tof_offset_calib(argc, argv);
#endif
#ifdef WORK_TASK_TOF_XL5300_ORINGE_CMD_GRADIENT_CALIB
    } else if (strcmp(argv[1], "gradient") == 0) {
        cmd_tof_gradient_calib(argc, argv);
#endif
    } else if (strcmp(argv[1], "show") == 0) {
        cmd_tof_show_calib(argc, argv);
    } else {
        rt_kprintf("[%s] Error: Unknown command '%s'\n", TAG, argv[1]);
        rt_kprintf("Use 'tof' without arguments to see usage help\n");
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_tof, tof, TOF XL5300 control and calibration);

#endif /* RT_USING_FINSH */

