/*!
* \file OPT3101Calibration.cpp
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
* This file contains the method definitions for class OPT3101::calibrationC
*/

#include "OPT3101Calibration.h"

void OPT3101::calibrationC::findCrosstalkTempRegisterValues(OPT3101::crosstalkTempCoffC *illumXtalkCoff, bool *txActiveList,bool *registerSetActiveList, OPT3101::crosstalkC *illumXtalk) {
	uint8_t c0,c1,scaleCoff;
	double maxCoff = -1;
	double magnitude[3][2];

	for (c0 = 0; c0 < 3; c0++) {
		for (c1 = 0; c1 < 2; c1++) {
			if(txActiveList[c0] && registerSetActiveList[c1]){
				magnitude[c0][c1] = illumXtalk[(c0<<1)+c1].magnitude();
				maxCoff = abs(illumXtalkCoff[(c0<<1)+c1].coffI*magnitude[c0][c1]) > maxCoff ? abs(illumXtalkCoff[(c0<<1)+c1].coffI*magnitude[c0][c1]) : maxCoff;
				maxCoff = abs(illumXtalkCoff[(c0<<1)+c1].coffQ*magnitude[c0][c1]) > maxCoff ? abs(illumXtalkCoff[(c0<<1)+c1].coffQ*magnitude[c0][c1]) : maxCoff;
			}
		}
	}
	scaleCoff = 7;
	for (c0 = 12; c0 >= 5; c0--)
		scaleCoff = maxCoff < (1 << (c0)) ? (12 - c0) : scaleCoff;

	for (c0 = 0; c0 < 3; c0++) {
		for (c1 = 0; c1 < 2; c1++) {
			if(txActiveList[c0] && registerSetActiveList[c1]){
				if (scaleCoff >= 5) {
					illumXtalkCoff[(c0<<1)+c1].coffIReg = (uint8_t)((illumXtalkCoff[(c0<<1)+c1].coffI*magnitude[c0][c1])*(1 << (scaleCoff - 5)));
					illumXtalkCoff[(c0<<1)+c1].coffQReg = (uint8_t)((illumXtalkCoff[(c0<<1)+c1].coffQ*magnitude[c0][c1])*(1 << (scaleCoff - 5)));
				}
				else {
					illumXtalkCoff[(c0<<1)+c1].coffIReg = (uint8_t)((illumXtalkCoff[(c0<<1)+c1].coffI*magnitude[c0][c1]) / (1 << (5 - scaleCoff)));
					illumXtalkCoff[(c0<<1)+c1].coffQReg = (uint8_t)((illumXtalkCoff[(c0<<1)+c1].coffQ*magnitude[c0][c1]) / (1 << (5 - scaleCoff)));
				}
				illumXtalkCoff[(c0<<1)+c1].commonScale = scaleCoff;
			}
		}
	}
}

void OPT3101::calibrationC::findPhaseTempRegisterValues(OPT3101::phaseTempCoffC* phaseTempCoff, bool *txActiveList,bool *registerSetActiveList, uint16_t freqCount) {
	uint8_t scaleCoff;
	uint8_t c1;
	int8_t c0;
	double maxCoff = -1;
	/// <b>Algorithm of the method is as follows</b>
	for (c0 = 0; c0 < 3; c0++) {
		for (c1 = 0; c1 < 2; c1++) {
			if(txActiveList[c0] && registerSetActiveList[c1])
				maxCoff = abs(phaseTempCoff[(c0<<1)+c1].coff) > maxCoff ? abs(phaseTempCoff[(c0<<1)+c1].coff) : maxCoff; ///* Identifies max of absolute of OPT3101::phaseTempCoffC::coff values
		}
	}
	maxCoff *= 16384.0 / freqCount; ///* scales the max coefficient with input frequencyCount
								  // 3200 
	scaleCoff = 0;
	for (c0 = 7; c0 >= 0; c0--)
		scaleCoff = (maxCoff*(1 << (10 - c0))) < (1 << 11) ? c0 : scaleCoff; ///* Finds a OPT3101::phaseTempCoffC::commonScale which can fit the OPT3101::phaseTempCoffC::coff to 12 bit registers
	for (c0 = 0; c0 < 3; c0++) {
		for (c1 = 0; c1 < 2; c1++) {
			if(txActiveList[c0] && registerSetActiveList[c1]){
				phaseTempCoff[(c0<<1)+c1].coffReg = ((int16_t)(16384.0 / freqCount * phaseTempCoff[(c0<<1)+c1].coff*(1 << (10 - scaleCoff)))) & 0xFFF;///* Finds OPT3101::phaseTempCoffC::coffReg values (12 bit register values) based on OPT3101::phaseTempCoffC::coff and OPT3101::phaseTempCoffC::commonScale
				phaseTempCoff[(c0<<1)+c1].commonScale = scaleCoff;
			}
		}
	}
}

void OPT3101::calibrationC::report()
{
	/// <b>Algorithm of the method is as follows</b>
	host.printf("------------------------\r\n");
	host.printf("Calibration Class Report\r\n");
	host.printf("------------------------\r\n"); ///* Prints all the members and values of members on screen.
	this->internalCrosstalk[0].report();
	host.printf("EEPROM_connected=%d\r\n", this->EEPROM_connected);
	host.printf("extTempSensor_connected=%d\r\n", this->extTempSensor_connected);
	host.printf("------------------------\r\n");
}
