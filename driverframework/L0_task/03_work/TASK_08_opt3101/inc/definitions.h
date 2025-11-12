/*!
* \file definition.h
* \author  Karthik Rajagopal <krthik@ti.com>
* \version 0.9.1
*
* \section COPYRIGHT
* TEXAS INSTRUMENTS TEXT FILE LICENSE
* Copyright (c) 2018 Texas Instruments Incorporated
* All rights reserved not granted herein.
* Limited License.
* Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  No hardware patent is licensed hereunder.
* Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution
* Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:
* * No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.
* * any redistribution and use are licensed by TI for use only with TI Devices.
* * Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
* If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:
* * any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.
* * any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.
* DISCLAIMER.
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* \section DESCRIPTION
* This is header file with all #define statements
*
*/
#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

/*! \def TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
\brief This pre-processor derivative dictates whether the host is TI MSP430 calibration hardware is being used or not
*/
#define TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL

#ifndef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
/*! \def HOST_PC
\brief This pre-processor derivative dictates whether the host is PC
In a PC environment file storage, std::iostream and stdio::printf methods are available. All these libraries are enabled then this derivative is defined
Disable this derivative in MCU or RTOS like environments where file storage doesn't make sense
Disabling this disabled report routines and file storage and load routines
*/
#define HOST_PC
#endif

#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
/*! \def PRINT_COLOR_TERMINAL
\brief Prints colors in terminal
*/
#define PRINT_COLOR_TERMINAL
#endif

#define OPT3101_I2C_SLAVEADDRESS 0x58


#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
/*! \def VERBOSE_MODE
\brief This pre-processor derivative dictates whether to print INFO, WARN and ERROR
*/
#define VERBOSE_MODE
#endif


#define TEMP_CYCLE_DELAY_IN_SECONDS_BETWEEN_DATA_POINTS 1
#define TEMP_CYCLE_TOTAL_NUMBER_OF_DATA_POINTS_PER_SETTING 1000 // MAX is 65535


#ifdef HOST_PC
#define PRINT_COLOR_TERMINAL
/*! \def VERBOSE_MODE
\brief This pre-processor derivative dictates whether to print INFO, WARN and ERROR
*/
#define VERBOSE_MODE
/*! \def OPT3101_USE_SERIALLIB
\brief This pre-processor derivative dictates whether to use included serial.h library
This is enabled by default in SDK. Not defining this derivate would disable the serial communication capability.
In case of usage with TI EVM this derivative is required, if using any other method of communication this can be disabled by user
*/
#define OPT3101_USE_SERIALLIB

/*! \def OPT3101_USE_STREAMLIB
\brief This pre-processor derivative dictates whether to load std::iostream and std::fstream  libraries
This is enabled by default in SDK. Not defining this derivate would disable all the std::iostream and related libraries. File storage will no longer be possible with the SDK.
All methods related to loadFromFile and saveToFile are hidden to the compiler.
*/
#define OPT3101_USE_STREAMLIB

/*! \def OPT3101_USE_STDIOLIB
\brief This pre-processor derivative dictates whether to load stdio library which contains printf and sprintf methods
This is enabled by default in SDK. Not defining this derivate would disable all sprintf and printf methods.
This means that the report() methods on several calls will be blank to the compiler. All file storage is also disabled since the name for the files cannot be resolved without the sprintf method
*/
#define OPT3101_USE_STDIOLIB
#endif

#endif /* DEFINITIONS_H_ */
