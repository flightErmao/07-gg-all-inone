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

#ifndef TASK_MLOG_TEST_H__
#define TASK_MLOG_TEST_H__

#include <rtthread.h>
#include "sensorsTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mlog test interface functions */

/**
 * @brief Start mlog test data generation
 *
 * @return rt_err_t RT_EOK on success, error code on failure
 */
rt_err_t mlog_test_start(void);

/**
 * @brief Stop mlog test data generation
 */
void mlog_test_stop(void);

/**
 * @brief Check if mlog test is running
 *
 * @return rt_bool_t RT_TRUE if running, RT_FALSE otherwise
 */
rt_bool_t mlog_test_is_running(void);

/**
 * @brief Get latest test data
 *
 * @param data Pointer to sensorData_t structure to store data
 */
void mlog_test_get_data(sensorData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MLOG_TEST_H__ */
