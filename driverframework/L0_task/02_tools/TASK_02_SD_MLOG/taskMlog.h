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

#ifndef TASK_MLOG_H__
#define TASK_MLOG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mlog task interface functions */

/**
 * @brief Start mlog logging
 *
 * @param file_path Log file path, if NULL, auto-generate path
 * @return rt_err_t RT_EOK on success, error code on failure
 */
rt_err_t task_mlog_start_logging(char* file_path);

/**
 * @brief Stop mlog logging
 */
void task_mlog_stop_logging(void);

/**
 * @brief Get mlog status
 *
 * @return uint8_t Mlog status
 */
uint8_t task_mlog_get_status(void);

/**
 * @brief Get current log file name
 *
 * @return char* Current log file name
 */
char* task_mlog_get_file_name(void);

/**
 * @brief Show mlog statistics
 */
void task_mlog_show_statistics(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MLOG_H__ */
