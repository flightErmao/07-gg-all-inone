/*!
* \file OPT3101Coefficients.cpp
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
* This file contains the coefficients to be set for the boards
*/

#include"OPT3101device.h"

void OPT3101::device::manuallySetIllumCrosstalkTempCoffs(){

	 ///* Units for coefficient: ScaledIorQ/tempRegister/MagnitudeCalc
    this->calibration[0].illumCrosstalkTempCoff[0][0].coffI=0.124734811687;
    this->calibration[0].illumCrosstalkTempCoff[0][0].coffQ=0.13592223628;
    this->calibration[0].illumCrosstalkTempCoff[0][1].coffI=0.121257044071;
    this->calibration[0].illumCrosstalkTempCoff[0][1].coffQ=0.0290795903941;
    this->calibration[0].illumCrosstalkTempCoff[1][0].coffI=-0.000172933230344;
    this->calibration[0].illumCrosstalkTempCoff[1][0].coffQ=-0.0163536237361;
    this->calibration[0].illumCrosstalkTempCoff[1][1].coffI=-0.010450043412;
    this->calibration[0].illumCrosstalkTempCoff[1][1].coffQ=-0.0910940906909;
	this->calibration[0].illumCrosstalkTempCoff[2][0].coffI=0.0;
	this->calibration[0].illumCrosstalkTempCoff[2][0].coffQ=0.0;
	this->calibration[0].illumCrosstalkTempCoff[2][1].coffI=0.0;
	this->calibration[0].illumCrosstalkTempCoff[2][1].coffQ=0.0;
}

void environmentalController::manuallySetReferenceDistances(){
	this->refDistancesInMM[0][0]=10;
	this->refDistancesInMM[0][1]=20;
	this->refDistancesInMM[1][0]=80;
	this->refDistancesInMM[1][1]=120;
	this->refDistancesInMM[2][0]=0;
	this->refDistancesInMM[2][1]=0;
}

void OPT3101::device::manuallySetPhaseTempCoffs(){
	///* Units for coefficient: Phase/tempRegister
    this->calibration[0].phaseTempCoff[0][0].coff=3.1183945334;
    this->calibration[0].phaseTempCoff[0][1].coff=3.91217820975;
    this->calibration[0].phaseTempCoff[1][0].coff=3.25081198763;
    this->calibration[0].phaseTempCoff[1][1].coff=2.73744376613;
	this->calibration[0].phaseTempCoff[2][0].coff=0.0;
	this->calibration[0].phaseTempCoff[2][1].coff=0.0;
	this->calibration[0].phaseTempCoff[0][0].istMainCoff=true;
	this->calibration[0].phaseTempCoff[0][1].istMainCoff=true;
	this->calibration[0].phaseTempCoff[1][0].istMainCoff=true;
	this->calibration[0].phaseTempCoff[1][1].istMainCoff=true;
	this->calibration[0].phaseTempCoff[2][0].istMainCoff=true;
	this->calibration[0].phaseTempCoff[2][1].istMainCoff=true;
}

void OPT3101::device::manuallySetPhaseAmbientCoffs() {
	///* <b>Warning:</b> User is expected to curve fit the phase ambient data with PWL curve fit and provide the floating point precision coefficients and split points
	this->calibration[0].phaseAmbientCoff[0].coff[0] = 0.0; ///* Users is expected to enter all the OPT3101::calibration::phaseAmbientCoff::coff values manually after computation outside based on data generated by OPT3101::device::calibrationSession_perDesignCalibrationPhaseAmbient
	this->calibration[0].phaseAmbientCoff[0].coff[1] = 0.0; // Set this based on PWL curveFit data
	this->calibration[0].phaseAmbientCoff[0].coff[2] = 0.0; // Set this based on PWL curveFit data
	this->calibration[0].phaseAmbientCoff[0].coff[3] = 0.0; // Set this based on PWL curveFit data
	this->calibration[0].phaseAmbientCoff[0].splitsReg[0] = 0; // Set this based on PWL curveFit data
	this->calibration[0].phaseAmbientCoff[0].splitsReg[1] = 0; // Set this based on PWL curveFit data
	this->calibration[0].phaseAmbientCoff[0].splitsReg[2] = 0; ///* Users is expected to enter all the OPT3101::calibration::phaseAmbientCoff::splitsReg values manually after computation outside based on data generated by OPT3101::device::calibrationSession_perDesignCalibrationPhaseAmbient
}
