/*!
* \file main.cpp
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
* This is example main file to invoke calibration routines on the device.
*/

#include <stdio.h>
#include "OPT3101device.h"


/*
 * README FIRST:
 * ------------
 * Files that may require editing by user
 * 1.) hostController.h   --> Set host related defines based on if the host is PC (HOST_PC) or MSP430 (TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL)
 * 2.) hostController.cpp --> Implement I2C Calls which are platform/hardware specific.
 *							Implement Hardware reset generation on RSTZ_MS pin which are platform/hardware speicific.
 *							Implement delay/sleep routines which are platform/hardware specific.
 * 3.) environmentControl.cpp --> Implement setup/hardware specific controls for thermal chamber control,target, cover,  wait/user input prompts for physical change to environment for calibration
 * 4.) OPT3101_configuration.cpp --> File included is a placeholder. Needs to be replaced with a file generated using OPT3101 Configurator tool based on desired configuration available on http://www.ti.com/tool/OPT3101CONFIG-SW
 */

#define INLAB_STEP_0 //check raw crosstalk using the live view with no calibration loaded.
//#define OPTIONAL_INLAB_STEP_1A //optional! compensates the xtalk on tx2. only needed for 3ch boards with high xtalk on tx2
//#define INLAB_STEP_1 //simple bringup
//#define INLAB_STEP_2
//#define INLAB_STEP_3
//#define INLAB_STEP_4
//#define INPRODCTION
//#define TESTING_LIVE_VIEW //load existing calibrations and run live view. Useful during inlab steps to verify calibrations are working properly.

int main() {

#ifdef VERBOSE_MODE
	host.printf("-------------------------------\r\n");
	host.printf("Starting Main Program Execution\r\n");
	host.printf("-------------------------------\r\n");
#endif

	OPT3101::device dev; ///* Declared variable dev of class OPT3101::device
#ifdef HOST_PC
	host.initialize();
#endif

	host.pause();

	if (!dev.validateI2C()) {
		host.pause();
		return 1;
	}
	if (!dev.validateDesignID()) {
		host.pause();
	}

#ifdef INLAB_STEP_0
    dev.resetInitAndViewData(3000, false);
#endif
#ifdef INLAB_STEP_1
	dev.calibrationSession_firstTimeBringUp();
	dev.resetInitAndViewData(3000, true);
#endif
#ifdef OPTIONAL_INLAB_STEP_1A //optional! compensates the xtalk on tx2. only needed for 3ch boards with high xtalk on tx2
    dev.calibrationSession_perDesignTx2IllumXtalkCorrection();
#endif
#ifdef INLAB_STEP_2
	dev.calibrationSession_perDesignCalibrationCrosstalkTemp();
#endif
#ifdef INLAB_STEP_3
	dev.calibrationSession_perDesignCalibrationPhaseTemp();
#endif
#ifdef INLAB_STEP_4
	dev.calibrationSession_perDesignCalibrationPhaseAmbient();
#endif
#ifdef INPRODCTION
	dev.calibrationSession_perUnitFactoryCalibration();
#endif
#ifdef TESTING_LIVE_VIEW
	dev.resetInitAndViewData(3000, true);
#endif

	host.printf("-------------------------------\r\n");
	host.pause();
 	return 0;
}
