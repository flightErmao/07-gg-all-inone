/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmi270_maximum_fifo.h
* @date       2020-11-04
* @version    v2.63.1
*
*/

/**
 * \ingroup bmi2xy
 * \defgroup bmi270_maximum_fifo BMI270_MAXIMUM_FIFO
 * @brief Sensor driver for BMI270_MAXIMUM_FIFO sensor
 */

#ifndef BMI270_MAXIMUM_FIFO_H_
#define BMI270_MAXIMUM_FIFO_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include <stdint.h>

/***************************************************************************/

/*!               Macro definitions
 ****************************************************************************/

/*! @name BMI270 Chip identifier */
#define BMI270_MAXIMUM_FIFO_CHIP_ID (0x24u)

/***************************************************************************/

/*!     BMI270 User Interface function prototypes
 ****************************************************************************/

/* 配置固件数组及长度，由 bmi270_maximum_fifo.c 提供 */
extern const uint8_t bmi270_maximum_fifo_config_file[];
extern const uint32_t bmi270_maximum_fifo_config_file_size;

/******************************************************************************/
/*! @name       C++ Guard Macros                                      */
/******************************************************************************/
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMI270_MAXIMUM_FIFO_H_ */
