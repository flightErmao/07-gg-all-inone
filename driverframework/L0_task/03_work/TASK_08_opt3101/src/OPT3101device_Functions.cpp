/*!
* \file OPT3101device_Functions.cpp
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
* The file contains methods definitions for classes OPT3101::device OPT3101::frameData OPT3101::calibrationC
*/
#include "OPT3101device.h"
#include "definitions.h"

void OPT3101::device::reset() {
	/// <b>Algorithm of the method is as follows</b>
	host.printf("INFO::Resetting Host\r\n");
	host.resetDevice(); ///* Invokes the hostController::reset method
	host.sleep(100);

}

void OPT3101::device::liveDataView(uint32_t nFrames,uint32_t refreshDelayinms,bool scroll){
	uint32_t c0;
	uint8_t c1;
	OPT3101::frameData data;
	host.printf("\r\n");
	for(c1=0;c1<3;c1++)
		host.printf("|Distmm, Phase,Amplud,SigS,AmbS,HDR|--");
	host.printf("|Amb,Cntr,Tmain,Tillum   |  Count/Total  |");
	host.printf("\r\n");
	for(c0=0;c0<nFrames;c0++){
		data.capture(&host, this,this->calibration->extTempSensor_connected);
		this->data[data.ledChannel]=data;
		for(c1=0;c1<3;c1++){
			if(this->data[c1].sigovl || this->data[c1].ambovl)
				host.printfSetColor(0b100);
			else
				host.printfSetColor(0b010);
			host.printf("|%06u,",(this->data[c1].phase*14989)>>16);
			host.printf("%06u,",this->data[c1].phase);
			host.printf("%06u,",this->data[c1].amplitude);
			host.printf("   %1u,",this->data[c1].sigovl);
			host.printf("   %1u,",this->data[c1].ambovl);
			host.printf("  %1u|--",this->data[c1].illumDac);
		}
		host.printfSetColor(0xFF);
		host.printf("|%03d,",data.ambient);
		host.printf("0x%02x,",data.frameCounter);
		host.printf("  %+03d,",(int32_t) (((int32_t)data.tmain>>3)-256));
		host.printf("%+07.4f|",((double)data.tillum/16-128));
		host.printf("%07u",c0);
		host.printf("/%07u|\r",nFrames);
		if(scroll)
			host.printf("\n");
		host.sleep(refreshDelayinms);
	}
	host.printf("\n");
}

void OPT3101::device::resetInitAndViewData(uint32_t nFrames,bool loadCalibration){
	this->reset();
	host.sleep(5000);
	this->initialize();
    this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
    this->calibration->internalCrosstalk[0].printHeader(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis
    this->calibration->internalCrosstalk[0].print(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis
    host.printf("\r\n");
	if(loadCalibration){
		this->manuallySetIllumCrosstalkTempCoffs();
		this->manuallySetPhaseTempCoffs();
		this->manuallySetPhaseAmbientCoffs();

		this->loadIllumCrosstalkSet(true);
		this->loadIllumCrosstalkTempCoffSet();

		this->loadPhaseOffsetSet(true);
		this->loadPhaseOffsetTempCoffSet();

		this->loadPhaseAmbientCoffSet();
	}
	this->liveDataView(nFrames, this->configurationFlags_frameTimeInMilliSeconds, false);
}

bool OPT3101::device::validateI2C() {
	/// <b>Algorithm of the method is as follows</b>
	uint32_t data;
	bool status;
	//0x8001A0
	//data=0xA001802E
	//data[0]=0x8;
	//data[1]=001A0
	status = 0;
	host.printf("INFO::Validating I2C Transaction\r\n");
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL	
	status=host.i2c.read(0x2E,&data);
	if(status){
		host.printfSetColor(0b010);
		host.printf("INFO::I2C Transaction Successful\r\n");
		host.printfSetColor(0xFF);
	}
	else{
		host.printfSetColor(0b100);
		host.printf("EROR::I2C Transaction Failed. Please check Connection\r\n");
		host.printfSetColor(0xFF);
	}
	return status;
#endif

#ifndef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	host.printfSetColor(0b110);
	host.printf("WARN::I2C Status checking not implemented\r\n");
	host.printfSetColor(0xFF);
	return true;
#endif

}

bool OPT3101::device::validateDesignID() {
	/// <b>Algorithm of the method is as follows</b>
	uint32_t data[2];
	bool status;
	status = 0;
	host.printf("INFO::Validating OPT3101 Design ID\r\n");
	this->reg.iq_read_data_sel=4;
	data[0]=host.readI2C(0x3b);
	this->reg.iq_read_data_sel=5;
	data[1]=host.readI2C(0x3b);

	if(data[0]==0x100411 && data[1]==0xc01000){
		status=true;
		host.printfSetColor(0b010);
		host.printf("INFO::Design ID 0x%06lx%06lx Verified\r\n",data[1],data[0]);
		host.printfSetColor(0xFF);
	}
	else{
		status=false;
		host.printfSetColor(0b110);
		host.printf("WARN::Design ID 0x%06lx%06lx Not Maching\r\n",data[1],data[0]);
		host.printfSetColor(0xFF);
	}
	this->reg.iq_read_data_sel=0;
	return status;
}


void OPT3101::device::measureAndCorrectInternalCrosstalk(OPT3101::crosstalkC *internalXtalk){
	/// <b>Algorithm of the method is as follows</b>
	uint8_t regStore[5];
	regStore[0]=this->reg.en_adaptive_hdr.read();
	regStore[1]=this->reg.hdr_mode.read();
	regStore[2]=this->reg.en_sequencer.read(); ///* Critical registers which are modified in this method are read from the h/w and temporarily buffered to local variables.
	regStore[3]=this->reg.en_processor_values.read(); ///* Temporarily buffered register states are restored to the h/w with register writes while exiting this method.
	
	this->reg.tg_en=0;
	this->reg.en_adaptive_hdr=0;
	this->reg.hdr_mode=0;
	this->reg.en_sequencer=0;
	this->reg.en_processor_values=0;
	this->reg.tg_en=1;

	this->reg.use_xtalk_reg_int=0;
	this->reg.int_xtalk_calib=1; ///* Enables internal crosstalk measure and correct register OPT3101::registers::int_xtalk_calib
	host.printf("INFO::Performing Internal Cross talk Measurement...\r\n");
	host.sleep(this->configurationFlags_xtalkSettlingOneTauInMilliSeconds<<3); ///* Sleeps the host for 8 times the OPT3101::device::configurationFlags_xtalkSettlingOneTauInMilliSeconds so that device settles
	/* host.sleepDataReadyCounts(this->configurationFlags_xtalkSettlingOneTauInDataReadyCounts<<3);  */
	///* Sleep delay can be replaced with wait for 8 times the OPT3101::device::configurationFlags_xtalkSettlingOneTauInDataReadyCounts data ready pulses.User needs to implement based on host configuration

	this->reg.int_xtalk_calib=0;  ///* Disables internal crosstalk measure register OPT3101::registers::int_xtalk_calib to complete internal crosstalk measurement

	regStore[4]=this->reg.iq_read_data_sel.read(); 
	this->reg.iq_read_data_sel=0;
	internalXtalk->readCrosstalkRegisters(this); ///* Reads the internal crosstalk registers OPT3101::registers::iphase_xtalk and OPT3101::registers::qphase_xtalk and assigns to internalXtalk input instance pointer of type OPT3101::crosstalkC class
	this->reg.iq_read_data_sel=regStore[4];

	this->reg.tg_en=0;
	this->reg.en_adaptive_hdr=regStore[0];
	this->reg.hdr_mode=regStore[1];
	this->reg.en_sequencer=regStore[2];
	this->reg.en_processor_values=regStore[3]; ///* Restores the device state to the same state as before entering this method from the buffered local variables
	this->reg.tg_en=1;
	host.printf("INFO::Internal Cross talk Measurement Completed\r\n");

}

void OPT3101::device::measureIllumCrosstalk(OPT3101::crosstalkC *illumXtalk) {
	uint8_t regStore[2];
	/// <b>Algorithm of the method is as follows</b>
	regStore[0] = this->reg.use_xtalk_reg_illum.read(); 
	this->reg.use_xtalk_reg_illum = 0;
	this->reg.illum_xtalk_calib = 1; ///* Enables illum crosstalk measure register OPT3101::registers::illum_xtalk_calib
	host.sleep(this->configurationFlags_xtalkSettlingOneTauInMilliSeconds << 3); ///* Sleeps the host for 8 times the OPT3101::device::configurationFlags_xtalkSettlingOneTauInMilliSeconds so that device settles
	// The same can be replated with 
	// host.sleepDataReadyCounts(this->configurationFlags_xtalkSettlingOneTauInDataReadyCounts<<3); // This is for 8 Tau settings
	///* Sleep delay can be replaced with wait for 8 times the OPT3101::device::configurationFlags_xtalkSettlingOneTauInDataReadyCounts data ready pulses.User needs to implement based on host configuration
	
	this->reg.illum_xtalk_calib = 0;
	//Wait Time

	regStore[1] = this->reg.iq_read_data_sel.read(); ///* Reads the internal crosstalk registers OPT3101::registers::iphase_xtalk and OPT3101::registers::qphase_xtalk and assigns to illumXtalk input instance pointer of type OPT3101::crosstalkC class
	this->reg.iq_read_data_sel = 1;
	illumXtalk->readCrosstalkRegisters(this);
	illumXtalk->illumXtalk = true;
	
	this->reg.iq_read_data_sel = regStore[1];
	this->reg.use_xtalk_reg_illum = regStore[0];
	
	illumXtalk->readTemperatureMain(this); ///* Reads main temp sensor and captures the same to input argument illumXtalk's OPT3101::crosstalkC::tmain
	illumXtalk->readTemperatureIllum(this); ///* Reads external temp sensor and captures the same to input argument illumXtalk's OPT3101::crosstalkC::tillum

}

void OPT3101::device::measureIllumCrosstalkSet(bool saveToFile) {
	uint8_t c0, c1;
	uint16_t flashLocation;
#ifdef OPT3101_USE_STDIOLIB
	char fileName[FILENAME_LENGTH];
#endif

	envController.setTargetToInfinity_OR_coverPhotodiode(); ///* Calls the method environmentalController::setTargetToInfinity_OR_coverPhotodiode , which is expected to set the environment so as to be able to measure illumination crosstalk
	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) {  ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers
				if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
					host.printf("INFO:Performing Illum Crosstalk for TX%d HDR %d\r\n",c0,c1);
					this->measureIllumCrosstalk(&this->calibration->illumCrosstalk[c0][c1], c0, c1 ? 'h' : 'l');  ///* Calls method OPT3101::device::measureIllumCrosstalk with argument OPT3101::calibrationC::illumCrosstalk
					this->calibration->illumCrosstalk[c0][c1].printHeader(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis
					this->calibration->illumCrosstalk[c0][c1].print(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis
					host.printf("\r\n");

																	  // Provide file name if save to File is implemented eg: illumCrosstalkTX{txChannel}Reg{regSet}_roomTemp
				   if(saveToFile){
#ifdef OPT3101_USE_STDIOLIB
						sprintf(fileName, "%s/illumCrosstalk_TX%d_%c.txt", filePath, c0, c1 ? 'h' : 'l'); ///* Creates filename  illumCrosstalk_TX{channel}_{h/l}.txt in path filePath
						this->calibration->illumCrosstalk[c0][c1].storeToFile(fileName);  ///* Calls method OPT3101::crosstalkC::storeToFile to store the data
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
						flashLocation=(c1+(c0<<1))<<4;
						host.flash.write(flashLocation, (uint8_t*)&this->calibration->illumCrosstalk[c0][c1] ,16);
						host.printf("INFO:Writing Illum Cross talk for TX%d HDR %d to Flash location 0x%04x\r\n",c0,c1,flashLocation);
#endif // TIHOST
				   }
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
					if(this->calibration->illumCrosstalk[c0][c1].magnitude()>800)
						host.printf("WARN:Illumination Cross talk is very high. Check if Photo diode is covered well.. if so increase [force_scale_val] register by 1 count and add it to initialization method\r\n",flashLocation);
#endif // TIHOST
				}
			}
		}
	}
	host.printf("INFO:Completed Illumination cross talk measurement for all channels\r\n");
}


void OPT3101::device::measureIllumCrosstalk( OPT3101::crosstalkC *illumXtalk, uint8_t txChannel, char registerSet,uint8_t shiftIllumPhase) {
	uint8_t regStore[8];
	/// <b>Algorithm of the method is as follows</b>
	regStore[0] = this->reg.en_sequencer.read(); ///* Critical registers which are modified in this method are read from the h/w and temporarily buffered to local variables.
	regStore[1] = this->reg.en_processor_values.read();
	regStore[2] = this->reg.sel_hdr_mode.read();
	regStore[3] = this->reg.sel_tx_ch.read();
	regStore[4] = this->reg.en_adaptive_hdr.read();
	regStore[5] = this->reg.en_tx_switch.read();
	regStore[6] = this->reg.shift_illum_phase.read();
	regStore[7] = this->reg.en_temp_xtalk_corr.read();

	this->reg.tg_en = 0;
	this->reg.en_sequencer = 0;
	this->reg.en_processor_values = 0;
	if (registerSet == 'h')
		this->reg.sel_hdr_mode = 1; ///* Register set is chosen based on registerSet input to this method by setting OPT3101::registers::sel_hdr_mode
	if (registerSet == 'l')
		this->reg.sel_hdr_mode = 0;
	this->reg.sel_tx_ch = txChannel; ///* TX channel is chosen based on txChannel input to this method by setting OPT3101::registers::sel_tx_ch
	this->reg.en_tx_switch = 0;
	this->reg.en_adaptive_hdr = 0;
	this->reg.shift_illum_phase = shiftIllumPhase; ///* Sets up the OPT3101::registers::shift_illum_phase based on the input to this method shiftIllumPhase
	this->reg.tg_en = 1;
	
	this->measureIllumCrosstalk(illumXtalk); ///* Measures illum crosstalk from registers OPT3101::registers::iphase_xtalk , and OPT3101::registers::qphase_xtalk and assigns to illumXtalk instance pointer of type OPT3101::crosstalkC class
	illumXtalk->illumDCdac = this->reg.ILLUM_DC_CURR_DAC.read();
	illumXtalk->shiftIllumPhase = shiftIllumPhase; ///* Takes snapshot of device TX register configuration and assign them to illumXtalk instance pointer of type OPT3101::crosstalkC class
	switch (txChannel) {
		case 0:
			if (registerSet == 'h') {
				illumXtalk->illumDac = this->reg.illum_dac_h_tx0.read();
				illumXtalk->illumScale = this->reg.illum_scale_h_tx0.read();
			}
			else if (registerSet == 'l') {
				illumXtalk->illumDac = this->reg.illum_dac_l_tx0.read();
				illumXtalk->illumScale = this->reg.illum_scale_l_tx0.read();
			}
			break;
		case 1:
			if (registerSet == 'h') {
				illumXtalk->illumDac = this->reg.illum_dac_h_tx1.read();
				illumXtalk->illumScale = this->reg.illum_scale_h_tx1.read();
			}
			else if (registerSet == 'l') {
				illumXtalk->illumDac = this->reg.illum_dac_l_tx1.read();
				illumXtalk->illumScale = this->reg.illum_scale_l_tx1.read();
			}
			break;
		case 2:
			if (registerSet == 'h') {
				illumXtalk->illumDac = this->reg.illum_dac_h_tx2.read();
				illumXtalk->illumScale = this->reg.illum_scale_h_tx2.read();
			}
			else if (registerSet == 'l') {
				illumXtalk->illumDac = this->reg.illum_dac_l_tx2.read();
				illumXtalk->illumScale = this->reg.illum_scale_l_tx2.read();
			}
			break;
		default:
			break;
	}
	//printf("Illum Cross-talk for Illum current [%d] Magnitude is %4.2f [%d][%d][%d]\n", i, InternalXtalk.magnitude(), InternalXtalk.I, InternalXtalk.Q, InternalXtalk.xtalkScale);
	
	this->reg.tg_en = 0;
	this->reg.en_sequencer = regStore[0];  
	this->reg.en_processor_values = regStore[1];
	this->reg.sel_hdr_mode = regStore[2];
	this->reg.sel_tx_ch = regStore[3];
	this->reg.en_adaptive_hdr = regStore[4];
	this->reg.en_tx_switch = regStore[5];
	this->reg.shift_illum_phase = regStore[6];
	this->reg.en_temp_xtalk_corr = regStore[7]; ///* Restores the device state to the same state as before entering this method from the buffered local variables

	this->reg.tg_en = 1;
}

void OPT3101::device::loadIllumCrosstalk(OPT3101::crosstalkC *illumXtalk, uint8_t txChannel, char registerSet) {
	uint8_t maxIllumXtalkScale;
	
	/// <b>Algorithm of the method is as follows</b>
	maxIllumXtalkScale = illumXtalk->commonScale; ///* Load the common to all TX channels/register set registers (like for eg OPT3101::registers::illum_xtalk_reg_scale) to the h/w from the method input argument illumXtalk
	this->reg.illum_xtalk_reg_scale = illumXtalk->commonScale;
	this->reg.shift_illum_phase = illumXtalk->shiftIllumPhase;
	
	//printf("Scale reduction is [%d] [%d] [%d] \n", illumXtalk->I, maxIllumXtalkScale - illumXtalk->xtalkScale, illumXtalk->I >>(maxIllumXtalkScale - illumXtalk->xtalkScale));
	switch (txChannel) { ///* Based on txChannel and registerSet argument the the method loads the OPT3101::crosstalkC::I, OPT3101::crosstalkC::Q, OPT3101::crosstalkC::tmain and OPT3101::crosstalkC::tillum to device h/w specified TX channels and register set  
	case 0: ///* for eg: then txChannel=1 and registerSet='l' then the method loads the argument illumXtalk's OPT3101::crosstalkC::I to OPT3101::registers::iphase_xtalk_reg_hdr0_tx1 . Similarly the qphase_reg* registers, OPT3101::registers::tmain_calib_hdr0_tx1 and OPT3101::registers::tillum_calib_hdr0_tx1 are loaded from the input argument illumXtalk's  OPT3101::crosstalkC::tmain and OPT3101::crosstalkC::tillum
		if (registerSet == 'h') {
			this->reg.iphase_xtalk_reg_hdr1_tx0 = (uint16_t) (illumXtalk->I >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.qphase_xtalk_reg_hdr1_tx0 = (uint16_t) (illumXtalk->Q >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.tmain_calib_hdr1_tx0 = illumXtalk->tmain;
			this->reg.tillum_calib_hdr1_tx0 = illumXtalk->tillum;
		}
		else if (registerSet == 'l') {
			this->reg.iphase_xtalk_reg_hdr0_tx0 = (uint16_t)(illumXtalk->I >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.qphase_xtalk_reg_hdr0_tx0 = (uint16_t)(illumXtalk->Q >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.tmain_calib_hdr0_tx0 = illumXtalk->tmain;
			this->reg.tillum_calib_hdr0_tx0 = illumXtalk->tillum;
		}
		break;
	case 1:
		if (registerSet == 'h') {
			this->reg.iphase_xtalk_reg_hdr1_tx1 = (uint16_t)(illumXtalk->I >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.qphase_xtalk_reg_hdr1_tx1 = (uint16_t)(illumXtalk->Q >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.tmain_calib_hdr1_tx1 = illumXtalk->tmain;
			this->reg.tillum_calib_hdr1_tx1 = illumXtalk->tillum;
		}
		else if (registerSet == 'l') {
			this->reg.iphase_xtalk_reg_hdr0_tx1 = (uint16_t)(illumXtalk->I >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.qphase_xtalk_reg_hdr0_tx1 = (uint16_t)(illumXtalk->Q >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.tmain_calib_hdr0_tx1 = illumXtalk->tmain;
			this->reg.tillum_calib_hdr0_tx1 = illumXtalk->tillum;
		}
		break;
	case 2:
		if (registerSet == 'h') {
			this->reg.iphase_xtalk_reg_hdr1_tx2 = (uint16_t)(illumXtalk->I >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.qphase_xtalk_reg_hdr1_tx2 = (uint16_t)(illumXtalk->Q >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.tmain_calib_hdr1_tx2 = illumXtalk->tmain;
			this->reg.tillum_calib_hdr1_tx2 = illumXtalk->tillum;
		}
		else if (registerSet == 'l') {
			this->reg.iphase_xtalk_reg_hdr0_tx2 = (uint16_t)(illumXtalk->I >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.qphase_xtalk_reg_hdr0_tx2 = (uint16_t)(illumXtalk->Q >> (maxIllumXtalkScale - illumXtalk->xtalkScale));
			this->reg.tmain_calib_hdr0_tx2 = illumXtalk->tmain;
			this->reg.tillum_calib_hdr0_tx2 = illumXtalk->tillum;
		}
		break;
	default:
		break;
	}

}

void OPT3101::device::loadPhaseOffset(OPT3101::phaseOffsetC *phaseOffset, uint8_t txChannel, char registerSet)
{
	/// <b>Algorithm of the method is as follows</b>

	//printf("Scale reduction is [%d] [%d] [%d] \n", illumXtalk->I, maxIllumXtalkScale - illumXtalk->xtalkScale, illumXtalk->I >>(maxIllumXtalkScale - illumXtalk->xtalkScale));
	this->reg.en_phase_corr = 1; ///* Enables the phase offset correction register OPT3101::registers::en_phase_corr
	switch (txChannel) {  ///* Based on txChannel and registerSet argument the the method loads the OPT3101::phaseOffsetC::phaseOffset, OPT3101::phaseOffsetC::frameData::tmain and OPT3101::phaseOffsetC::frameData::tillum to device h/w specified TX channels and register set  
	case 0: ///* for eg: then txChannel=1 and registerSet='l' then the method loads the argument phaseOffset's OPT3101::phaseOffsetC::phaseOffset to OPT3101::registers::phase_offset_hdr0_tx1 OPT3101::registers::tmain_calib_hdr0_tx1 and OPT3101::registers::tillum_calib_hdr0_tx1 are loaded from the input argument phaseOffset's  OPT3101::phaseOffsetC::frameData::tmain and OPT3101::phaseOffsetC::frameData::tillum
		if (registerSet == 'h') {
			this->reg.phase_offset_hdr1_tx0 = phaseOffset->phaseOffset;
			this->reg.tmain_calib_hdr1_tx0 = phaseOffset->data.tmain;
			this->reg.tillum_calib_hdr1_tx0 = phaseOffset->data.tillum;
		}
		else if (registerSet == 'l') {
			this->reg.phase_offset_hdr0_tx0 = phaseOffset->phaseOffset;
			this->reg.tmain_calib_hdr0_tx0 = phaseOffset->data.tmain;
			this->reg.tillum_calib_hdr0_tx0 = phaseOffset->data.tillum;
		}
		break;
	case 1:
		if (registerSet == 'h') {
			this->reg.phase_offset_hdr1_tx1 = phaseOffset->phaseOffset;
			this->reg.tmain_calib_hdr1_tx1 = phaseOffset->data.tmain;
			this->reg.tillum_calib_hdr1_tx1 = phaseOffset->data.tillum;
		}
		else if (registerSet == 'l') {
			this->reg.phase_offset_hdr0_tx1 = phaseOffset->phaseOffset;
			this->reg.tmain_calib_hdr0_tx1 = phaseOffset->data.tmain;
			this->reg.tillum_calib_hdr0_tx1 = phaseOffset->data.tillum;
		}
		break;
	case 2:
		if (registerSet == 'h') {
			this->reg.phase_offset_hdr1_tx2 = phaseOffset->phaseOffset;
			this->reg.tmain_calib_hdr1_tx2 = phaseOffset->data.tmain;
			this->reg.tillum_calib_hdr1_tx2 = phaseOffset->data.tillum;
		}
		else if (registerSet == 'l') {
			this->reg.phase_offset_hdr0_tx2 = phaseOffset->phaseOffset;
			this->reg.tmain_calib_hdr0_tx2 = phaseOffset->data.tmain;
			this->reg.tillum_calib_hdr0_tx2 = phaseOffset->data.tillum;
		}
		break;
	default:
		break;
	}
}

void OPT3101::device::loadIllumCrosstalkTempCoff(OPT3101::crosstalkTempCoffC *illumXtalkTempCoff, uint8_t txChannel, char registerSet)
{
	/// <b>Algorithm of the method is as follows</b>

	this->reg.scale_temp_coeff_xtalk = illumXtalkTempCoff->commonScale; ///* loads common scale OPT3101::crosstalkTempCoffC::commonScale from input argument illumXtalkTempCoff to device register OPT3101::registers::scale_temp_coeff_xtalk
	switch (txChannel) { ///* Based on txChannel and registerSet argument the  method loads the OPT3101::crosstalkTempCoffC::coffIReg and OPT3101::crosstalkTempCoffC::coffQReg  to respective register set and channel registers 
	case 0: ///* for eg: then txChannel=1 and registerSet='l' then the method loads the argument illumXtalkTempCoff's OPT3101::crosstalkTempCoffC::coffIReg to OPT3101::registers::temp_coeff_xtalk_iphase_hdr0_tx1 and similarly for the qphase register as well
		if (registerSet == 'h') {
			this->reg.temp_coeff_xtalk_iphase_hdr1_tx0 = illumXtalkTempCoff->coffIReg;
			this->reg.temp_coeff_xtalk_qphase_hdr1_tx0 = illumXtalkTempCoff->coffQReg;
		}
		else if (registerSet == 'l') {
			this->reg.temp_coeff_xtalk_iphase_hdr0_tx0 = illumXtalkTempCoff->coffIReg;
			this->reg.temp_coeff_xtalk_qphase_hdr0_tx0 = illumXtalkTempCoff->coffQReg;
		}
		break;
	case 1:
		if (registerSet == 'h') {
			this->reg.temp_coeff_xtalk_iphase_hdr1_tx1 = illumXtalkTempCoff->coffIReg;
			this->reg.temp_coeff_xtalk_qphase_hdr1_tx1 = illumXtalkTempCoff->coffQReg;
		}
		else if (registerSet == 'l') {
			this->reg.temp_coeff_xtalk_iphase_hdr0_tx1 = illumXtalkTempCoff->coffIReg;
			this->reg.temp_coeff_xtalk_qphase_hdr0_tx1 = illumXtalkTempCoff->coffQReg;
		}
		break;
	case 2:
		if (registerSet == 'h') {
			this->reg.temp_coeff_xtalk_iphase_hdr1_tx2 = illumXtalkTempCoff->coffIReg;
			this->reg.temp_coeff_xtalk_qphase_hdr1_tx2 = illumXtalkTempCoff->coffQReg;
		}
		else if (registerSet == 'l') {
			this->reg.temp_coeff_xtalk_iphase_hdr0_tx2 = illumXtalkTempCoff->coffIReg;
			this->reg.temp_coeff_xtalk_qphase_hdr0_tx2 = illumXtalkTempCoff->coffQReg;
		}
		break;
	default:
		break;
	}

}

void OPT3101::device::loadPhaseOffsetTempCoff(OPT3101::phaseTempCoffC *phaseTempCoff, uint8_t txChannel, char registerSet)
{
	/// <b>Algorithm of the method is as follows</b>

	this->reg.en_temp_corr = 1; ///* Enables the phase temp coefficient register OPT3101::registers::en_temp_corr
	this->reg.scale_phase_temp_coeff = phaseTempCoff->commonScale; ///* Sets the common scale register OPT3101::registers::scale_phase_temp_coeff with value from input argument phaseTempCoff's OPT3101::phaseTempCoffC::commonScale
	switch (txChannel) { ///* Based on txChannel and registerSet argument the  method loads the OPT3101::phaseTempCoffC::coffReg to respective register set and channel registers 
	case 0: ///* for eg: then txChannel=1 and registerSet='l' then the method loads the argument phaseTempCoff's OPT3101::phaseTempCoffC::coffReg to OPT3101::registers::temp_coeff_illum_hdr0_tx1 or OPT3101::registers::temp_coeff_main_hdr0_tx1 depending on if phaseTempCoff's OPT3101::phaseTempCoffC::istMainCoff is True or False
		if (registerSet == 'h') {
			if (phaseTempCoff->istMainCoff)
				this->reg.temp_coeff_main_hdr1_tx0 = phaseTempCoff->coffReg;
			else
				this->reg.temp_coeff_illum_hdr1_tx0 = phaseTempCoff->coffReg;
		}
		else if (registerSet == 'l') {
			if (phaseTempCoff->istMainCoff)
				this->reg.temp_coeff_main_hdr0_tx0 = phaseTempCoff->coffReg;
			else
				this->reg.temp_coeff_illum_hdr0_tx0 = phaseTempCoff->coffReg;
		}
		break;
	case 1:
		if (registerSet == 'h') {
			if (phaseTempCoff->istMainCoff)
				this->reg.temp_coeff_main_hdr1_tx1 = phaseTempCoff->coffReg;
			else
				this->reg.temp_coeff_illum_hdr1_tx1 = phaseTempCoff->coffReg;
		}
		else if (registerSet == 'l') {
			if (phaseTempCoff->istMainCoff)
				this->reg.temp_coeff_main_hdr0_tx1 = phaseTempCoff->coffReg;
			else
				this->reg.temp_coeff_illum_hdr0_tx1 = phaseTempCoff->coffReg;
		}
		break;
	case 2:
		if (registerSet == 'h') {
			if (phaseTempCoff->istMainCoff)
				this->reg.temp_coeff_main_hdr1_tx2 = phaseTempCoff->coffReg;
			else
				this->reg.temp_coeff_illum_hdr1_tx2 = phaseTempCoff->coffReg;
		}
		else if (registerSet == 'l') {
			if (phaseTempCoff->istMainCoff)
				this->reg.temp_coeff_main_hdr0_tx2 = phaseTempCoff->coffReg;
			else
				this->reg.temp_coeff_illum_hdr0_tx2 = phaseTempCoff->coffReg;
		}
		break;
	default:
		break;
	}
}

void OPT3101::device::loadPhaseAmbientCoff(OPT3101::phaseAmbientCoffC *phaseAmbientCoff)
{
	/// <b>Algorithm of the method is as follows</b>

	this->reg.scale_amb_phase_corr_coeff = phaseAmbientCoff->commonScale; ///* Assigns all the register entries from phaseAmbientCoff OPT3101::phaseAmbientCoffC to h/w registers 
	this->reg.amb_phase_corr_pwl_coeff0 = phaseAmbientCoff->coffReg[0]; ///* for eg:  input argument phaseAmbientCoff's  OPT3101::phaseAmbientCoffC::coffReg is loaded to OPT3101::registers::amb_phase_corr_pwl_coeff0 and other related registers
	this->reg.amb_phase_corr_pwl_coeff1 = phaseAmbientCoff->coffReg[1];
	this->reg.amb_phase_corr_pwl_coeff2 = phaseAmbientCoff->coffReg[2];
	this->reg.amb_phase_corr_pwl_coeff3 = phaseAmbientCoff->coffReg[3];
	this->reg.amb_phase_corr_pwl_x0 = phaseAmbientCoff->splitsReg[0];
	this->reg.amb_phase_corr_pwl_x1 = phaseAmbientCoff->splitsReg[1];
	this->reg.amb_phase_corr_pwl_x2 = phaseAmbientCoff->splitsReg[2];
}

OPT3101::frameData::frameData()
{
	/// <b>Algorithm of the method is as follows</b>

	this->phase = 0; ///* Initializes all members to 0
	this->phaseovl = false;
	this->phaseovlf2 = false;
	this->ambovl = false;
	this->illumDac = false;
	this->ledChannel = 0;
	this->frameStatus = false;
	this->dealiasFreq = false;
	this->frameCounter = 0;
	this->amplitude = 0;
	this->sigovl = false;
	this->dealiasBin = 0;
	this->ambient = 0;
	this->temp = 0;
	this->tmain = 0;
	this->tillum = 0;

}

void OPT3101::frameData::capture(hostController *host, OPT3101::device *dev,bool readTillum) {
	uint8_t c0;
	uint32_t data32[3];
	/// <b>Algorithm of the method is as follows</b>
	//host->sleep((dev->configurationFlags_frameTimeInMilliSeconds)<<1); ///* Sleep host for a specified time depending on device configuration to OPT3101 AFE can update measurements to the registers.
	for (c0 = 8; c0 < 11; c0++) ///* Performs a direct read of I2C registers 0x08 0x09 and 0x0A directly though hostController::readI2C method 
		data32[c0-8] = (host->readI2C(c0));

	this->phase = data32[0] & 0xFFFF; ///* Maps the I2C read values to the class members like OPT3101::frameData::phase, OPT3101::frameData::amplitude etc 
	this->phaseovl = (data32[0] >> 16) & 0x1;

	this->phase |= (((uint32_t) this->phaseovl) << 16);

	this->illumDac = (data32[0] >> 17) & 0x01;
	this->ledChannel = (data32[0] >> 18) & 0x03;
	this->frameStatus = (data32[0] >> 20) & 0x01;
	this->dealiasFreq = (data32[0] >> 21) & 0x01;
	this->ambovl = (data32[0] >> 22) & 0x01;
	this->frameCounter = (data32[0] >> 23) & 0x01;

	this->amplitude = data32[1] & 0xFFFF;
	this->frameCounter |= ((data32[1] >> 16) & 0x03) << 1;
	this->sigovl = (data32[1] >> 18) & 0x01;
	this->phaseovlf2 = (data32[1] >> 19) & 0x01;
	this->dealiasBin = (data32[1] >> 20) & 0x0F;

	this->frameCounter |= (data32[2] & 0x03) << 3;
	this->ambient = (data32[2] >> 2) & 0x3FF;
	this->temp = (data32[2] >> 12) & 0xFFF;
	this->tmain = this->temp;
	if (readTillum)
		this->tillum = dev->reg.tillum.read(); ///* Based on readIllum flag reads the OPT3101::registers::tillum and assigns to OPT3101::frameData::tillum

}

void OPT3101::frameData::report()
{
	/// <b>Algorithm of the method is as follows</b>
	host.printf("-----------------------\r\n");
	host.printf("Frame data Class Report\r\n");
	host.printf("-----------------------\r\n"); ///* Prints all the members and values of members on screen.
	host.printf("phase=%d\r\n", this->phase);
	host.printf("phaseovl=%d\r\n", this->phaseovl);
	host.printf("phaseovlf2=%d\r\n", this->phaseovlf2);
	host.printf("ambovl=%d\r\n", this->ambovl);
	host.printf("illumDac=%d\r\n", this->illumDac);
	host.printf("ledChannel=%d\r\n", this->ledChannel);
	host.printf("frameStatus=%d\r\n", this->frameStatus);
	host.printf("dealiasFreq=%d\r\n", this->dealiasFreq);
	host.printf("frameCounter=%d\r\n", this->frameCounter);
	host.printf("amplitude=%d\r\n", this->amplitude);
	host.printf("sigovl=%d\r\n", this->sigovl);
	host.printf("dealiasBin=%d\r\n", this->dealiasBin);
	host.printf("ambient=%d\r\n", this->ambient);
	host.printf("temp=%d\r\n", this->temp);
	host.printf("tmain=%d\r\n", this->tmain);
	host.printf("tillum=%d\r\n", this->tillum);
	host.printf("-----------------------\r\n"); ///* Prints all the members and values of members on screen.
}
void OPT3101::frameData::printHeader()
{
	/// <b>Algorithm of the method is as follows</b>
	host.printfSetColor(0b001);
	host.printf("Phase,");
	host.printf("Amplt,");
	host.printf("SigOl,");
	host.printf("AmbOl,");
	host.printf("Amb,");
	host.printf("tMain,");
	host.printf("tIlum,");
	host.printf("tMain(C),");
	host.printf("  tIlum(C)\r\n");
	host.printfSetColor(0xFF);
}


void OPT3101::frameData::print()
{
	/// <b>Algorithm of the method is as follows</b>
	host.printfSetColor(0b001);
	host.printf("%05u,",this->phase);
	host.printf("%05u,",this->amplitude);
	host.printf("    %1u,",this->sigovl);
	host.printf("    %1u,",this->ambovl);
	host.printf("%03u,",this->ambient);
	host.printf(" %04d,",this->tmain);
	host.printf(" %04d,",this->tillum);
	host.printf("     %+03d,",(int8_t) ((((int16_t)this->tmain)>>3)-256));
	host.printf(" %+07.4f\r\n",((double)(((int16_t)this->tillum)-2048))/16.0);
	host.printfSetColor(0xFF);
}


void OPT3101::frameData::populateWithMean(OPT3101::frameData *inputData, uint16_t nFrames)
{
	uint16_t c0;
	uint16_t shiftBits;
	uint32_t accum;
	/// <b>Algorithm of the method is as follows</b>
	shiftBits = 0;
	nFrames = nFrames == 0 ? 1 : nFrames; ///* Sets nFrames to 1 when nFrames is 0
	if(nFrames>1)
		while (((nFrames-1) >> ++shiftBits) > 0); ///* Finds the smallest 2^N number higher than the nFrames provided in the argument of this method
	else



	// All flags
	this->phaseovl = 0;
	this->phaseovlf2 = 0;
	this->ambovl = 0;
	this->illumDac = inputData[0].illumDac;
	this->ledChannel = inputData[0].ledChannel;
	this->frameStatus = 0;
	this->dealiasFreq = inputData[0].dealiasFreq;
	this->sigovl = 0;
	this->dealiasBin = inputData[0].dealiasBin;
	this->frameCounter = 0;

	for (c0 = 0; c0 < nFrames; c0++) {
		this->phaseovl |= inputData[0].phaseovl;
		this->phaseovlf2 |= inputData[0].phaseovlf2;
		this->ambovl |= inputData[0].ambovl;
		this->sigovl |= inputData[0].sigovl;
	}
	
	
	// Mean of phase if found
	accum = 0; ///* Accumulates and measures mean for all the input class instance members and assigns them to the method's class members 
	for (c0 = 0; c0 < nFrames; c0++)
		accum += inputData[c0].phase;
	this->phase = accum >> shiftBits;

	// Mean of amplitude is found
	accum = 0;
	for (c0 = 0; c0 < nFrames; c0++)
		accum += inputData[c0].amplitude;
	this->amplitude = accum >> shiftBits;
	
	// Mean of ambient is found
	accum = 0;
	for (c0 = 0; c0 < nFrames; c0++)
		accum += inputData[c0].ambient;
	this->ambient = accum >> shiftBits;

	// Mean of temp is found
	accum = 0;
	for (c0 = 0; c0 < nFrames; c0++)
		accum += inputData[c0].temp;
	this->temp = accum >> shiftBits;

	// Mean of tmain is found
	accum = 0;
	for (c0 = 0; c0 < nFrames; c0++) 
		accum += inputData[c0].tmain;

	this->tmain = accum >> shiftBits;

	// Mean of tillum is found
	accum = 0;
	for (c0 = 0; c0 < nFrames; c0++)
		accum += inputData[c0].tillum;
	this->tillum = accum >> shiftBits;
	///* <b>Warning:</b> When nFrames is a non 2^N number the mean results are expected to be lower than actual measurements

}

#ifdef OPT3101_USE_STREAMLIB
std::ostream & OPT3101::operator<<(std::ostream & os, const OPT3101::frameData * data)
{
	/// <b>Algorithm of the method is as follows</b>
	os << data->phase << '\n'; ///* Serializes all the members and returns to the stream 
	os << data->phaseovl << '\n';
	os << data->phaseovlf2 << '\n';
	os << data->ambovl << '\n';
	os << data->illumDac << '\n';
	os << data->ledChannel << '\n';
	os << data->frameStatus << '\n';
	os << data->dealiasFreq << '\n';
	os << data->frameCounter << '\n';
	os << data->amplitude << '\n';
	os << data->sigovl << '\n';
	os << data->dealiasBin << '\n';
	os << data->temp << '\n';
	os << data->tmain << '\n';
	os << data->tillum << '\n';
	return os;
}
#endif

#ifdef OPT3101_USE_STREAMLIB
std::istream & OPT3101::operator>>(std::istream & is, OPT3101::frameData * data)
{
	/// <b>Algorithm of the method is as follows</b>
	is >> data->phase; ///* Serializes all the members and returns to the stream 
	is >> data->phaseovl;
	is >> data->phaseovlf2;
	is >> data->ambovl;
	is >> data->illumDac;
	is >> data->ledChannel;
	is >> data->frameStatus;
	is >> data->dealiasFreq;
	is >> data->frameCounter;
	is >> data->amplitude;
	is >> data->sigovl;
	is >> data->dealiasBin;
	is >> data->temp;
	is >> data->tmain;
	is >> data->tillum;

	return is;

}
#endif

#ifdef OPT3101_USE_STDIOLIB
void OPT3101::frameData::storeToFile(char * fileName)
{
#ifdef OPT3101_USE_STREAMLIB
	/// <b>Algorithm of the method is as follows</b>
	std::ofstream ofs(fileName);
	ofs << this;
	ofs.close(); ///* User needs to implement file storage based on host. 
#endif
}
#endif

#ifdef OPT3101_USE_STDIOLIB
void OPT3101::frameData::loadFromFile(char * fileName)
{
#ifdef OPT3101_USE_STREAMLIB
	/// <b>Algorithm of the method is as follows</b>
	std::ifstream ifs(fileName);
	ifs >> this;
	ifs.close(); ///* User needs to implement file load/restore based on host. 
#endif
}
#endif

void OPT3101::device::measurePhaseOffsetSet(bool saveToFile) {
	uint8_t c0, c1;
	uint16_t flashLocation;
	uint16_t refDistanceInCodes;
	uint32_t refDistanceInMM;

#ifdef OPT3101_USE_STDIOLIB
	char fileName[FILENAME_LENGTH];
#endif

	envController.manuallySetReferenceDistances();

	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers
				if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
					refDistanceInMM=envController.refDistancesInMM[c0][c1];
					envController.setTargetDistance(refDistanceInMM); ///* Calls environmentalController::setTargetDistance method with the specified distance
					refDistanceInCodes = (refDistanceInMM * 4477) >> 10; ///* Converts the reference distance specified in codes related to OPT3101::frameData::phase ADC codes
					host.printf("INFO:Performing Phase Offset for TX%d HDR %d @ ref Distance %dmm\r\n",c0,c1,refDistanceInMM);
					this->measurePhaseOffset(&this->calibration->phaseOffset[c0][c1], c0, c1 ? 'h' : 'l', refDistanceInCodes); ///* Measures phase offset by calling method OPT3101::device::measurePhaseOffset with specified reference distance and OPT3101::calibrationC::phaseOffset as argument to store the phase offset values
					this->calibration->phaseOffset[c0][c1].printHeader();
					this->calibration->phaseOffset[c0][c1].print();
					host.printf("\r\n");

					if(saveToFile){
#ifdef OPT3101_USE_STDIOLIB
						sprintf(fileName, "%s/phaseOffset_TX%d_%c.txt", filePath, c0, c1 ? 'h' : 'l'); ///* Creates filename  illumCrosstalk_TX{channel}_{h/l}.txt in path filePath
						this->calibration->phaseOffset[c0][c1].storeToFile(fileName);  ///* Calls method OPT3101::crosstalkC::storeToFile to store the data
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
						flashLocation=96+36*((c0<<1)+c1);
						host.flash.write(flashLocation, (uint8_t*)&this->calibration->phaseOffset[c0][c1] ,36);
						host.printf("INFO:Writing Phase Offset for TX%d HDR %d to Flash location 0x%04x\r\n",c0,c1,flashLocation);
#endif // TIHOST
					}
				}
			}
		}
	}

}


void OPT3101::device::measurePhaseOffset(OPT3101::phaseOffsetC *phaseOffset,uint16_t refDistanceInCodes) {
	OPT3101::frameData data[32], meanData;
	uint16_t nFrames;
	int32_t phaseOffsetRegister;
	uint16_t c0;
	/// <b>Algorithm of the method is as follows</b>
	nFrames = 1 << (this->configurationFlags_avgFrameCountExponentOfTwo <= 5 ? 5 - this->configurationFlags_avgFrameCountExponentOfTwo : 0); ///* Calculates number of frames to average the data. Based on the OPT3101::device::configurationFlags_avgFrameCountExponentOfTwo number of average is decided. More frames are captured and averaged if device configuration has low average frames and vice versa
	//data = new OPT3101::frameData[nFrames];
	for (c0 = 0; c0<nFrames; c0++)
		data[c0].capture(&host, this); ///* Captures all calculated number of frames from the h/w using OPT3101::frameData::capture method

	meanData.populateWithMean(&data[0], nFrames);  ///* Measures the mean of the captured frames OPT3101::frameData::populateWithMean method
	phaseOffsetRegister = (int32_t) (meanData.phase - refDistanceInCodes);  ///* Calculates the actual phase offset register value based on the input argument refDistanceInCodes and measured phase OPT3101::frameData::phase (mean over captured frames)
	phaseOffset->freqCount = this->reg.freq_count_read_reg.read(); ///* Captures snapshot of register value OPT3101::registers::freq_count_read_reg and assigns to input argument phaseOffset's OPT3101::phaseOffsetC::freqCount
	if (phaseOffset->freqCount > 0)
		phaseOffsetRegister = (int32_t)((phaseOffsetRegister * 16384) / phaseOffset->freqCount); ///* Scales phase offset value with OPT3101::phaseOffsetC::freqCount value 
	else
		phaseOffsetRegister = 0;

	phaseOffset->phaseOffset = (uint16_t)(phaseOffsetRegister & 0xFFFF); ///* Assigns input argument phaseOffset's  OPT3101::phaseOffsetC::phaseOffset with scaled register value

	phaseOffset->data = meanData; ///* Captures the calculated mean OPT3101::frameData to OPT3101::phaseOffsetC::data
	phaseOffset->referenceDistanceInCodes = refDistanceInCodes;  ///* Captures the snapshot of input argument refDistanceInCodes to the input argument phaseOffset's OPT3101::phaseOffsetC::referenceDistanceInCodes
}

void OPT3101::device::measurePhaseOffset(OPT3101::phaseOffsetC *phaseOffset, uint8_t txChannel, char registerSet, uint16_t refDistanceInCodes,uint8_t shiftIllumPhase) {
	uint8_t regStore[14];
	uint8_t dummy;

	/// <b>Algorithm of the method is as follows</b>
	regStore[0] = this->reg.en_sequencer.read();
	regStore[1] = this->reg.en_processor_values.read();
	regStore[2] = this->reg.sel_hdr_mode.read();
	regStore[3] = this->reg.sel_tx_ch.read();
	regStore[4] = this->reg.en_adaptive_hdr.read();
	regStore[5] = this->reg.en_tx_switch.read();
	regStore[6] = this->reg.en_phase_corr.read();
	regStore[7] = this->reg.en_nl_corr.read();
	regStore[8] = this->reg.en_temp_corr.read();
	regStore[9] = this->reg.amb_phase_corr_pwl_coeff0.read();
	regStore[10] = this->reg.amb_phase_corr_pwl_coeff1.read();
	regStore[11] = this->reg.amb_phase_corr_pwl_coeff2.read();
	regStore[12] = this->reg.amb_phase_corr_pwl_coeff3.read();
	regStore[13] = this->reg.shift_illum_phase.read(); ///* Critical registers which are modified in this method are read from the h/w and temporarily buffered to local variables.


	this->reg.tg_en = 0;
	this->reg.en_sequencer = 0;
	this->reg.en_processor_values = 0;
	if (registerSet == 'h')
		this->reg.sel_hdr_mode = 1; ///* Register set is chosen based on registerSet input to this method by setting OPT3101::registers::sel_hdr_mode
	if (registerSet == 'l')
		this->reg.sel_hdr_mode = 0;
	this->reg.sel_tx_ch = txChannel; ///* TX channel is chosen based on txChannel input to this method by setting OPT3101::registers::sel_tx_ch
	this->reg.en_tx_switch = 0;
	this->reg.en_adaptive_hdr = 0;
	this->reg.en_phase_corr = 0;
	this->reg.en_nl_corr = 0;
	this->reg.en_temp_corr = 0;
	this->reg.amb_phase_corr_pwl_coeff0 = 0;
	this->reg.amb_phase_corr_pwl_coeff1 = 0;
	this->reg.amb_phase_corr_pwl_coeff2 = 0;
	this->reg.amb_phase_corr_pwl_coeff3 = 0;
	this->reg.shift_illum_phase = shiftIllumPhase; ///* Sets up the OPT3101::registers::shift_illum_phase based on the input to this method shiftIllumPhase
	this->reg.tg_en = 1;

	this->measurePhaseOffset(phaseOffset, refDistanceInCodes); ///* Measures phase offset with following steps
	// Added dummy lines so that Doxygen can recognize these multi-line comments
	dummy = 0; ///* Calculates number of frames to average the data. Based on the OPT3101::device::configurationFlags_avgFrameCountExponentOfTwo number of average is decided. More frames are captured and averaged if device configuration has low average frames and vice versa
	dummy = 0; ///* Captures all calculated number of frames from the h/w using OPT3101::frameData::capture method
	dummy = 0; ///* Measures the mean of the captured frames OPT3101::frameData::populateWithMean method
	dummy = 0; ///* Calculates the actual phase offset register value based on the input argument refDistanceInCodes and measured phase OPT3101::frameData::phase (mean over captured frames)
	dummy = 0; ///* Captures snapshot of register value OPT3101::registers::freq_count_read_reg and assigns to input argument phaseOffset's OPT3101::phaseOffsetC::freqCount
	dummy = 0; ///* Scales phase offset value with OPT3101::phaseOffsetC::freqCount value
	dummy = 0; ///* Assigns input argument phaseOffset's  OPT3101::phaseOffsetC::phaseOffset with scaled register value
	dummy = 0; ///* Captures the calculated mean OPT3101::frameData to OPT3101::phaseOffsetC::data
	dummy = 0; ///* Captures the snapshot of input argument refDistanceInCodes to the input argument phaseOffset's OPT3101::phaseOffsetC::referenceDistanceInCodes

	phaseOffset->shiftIllumPhase = shiftIllumPhase;
	phaseOffset->illumDCdac = this->reg.ILLUM_DC_CURR_DAC.read();
	switch (txChannel) { ///* Captures the snapshot of the illum dac, illum DC and illum scale settings form h/w to the phaseOffset input argument 
	case 0:
		if (registerSet == 'h') {
			phaseOffset->illumDac = this->reg.illum_dac_h_tx0.read();
			phaseOffset->illumScale = this->reg.illum_scale_h_tx0.read();
		}
		else if (registerSet == 'l') {
			phaseOffset->illumDac = this->reg.illum_dac_l_tx0.read();
			phaseOffset->illumScale = this->reg.illum_scale_l_tx0.read();
		}
		break;
	case 1:
		if (registerSet == 'h') {
			phaseOffset->illumDac = this->reg.illum_dac_h_tx1.read();
			phaseOffset->illumScale = this->reg.illum_scale_h_tx1.read();
		}
		else if (registerSet == 'l') {
			phaseOffset->illumDac = this->reg.illum_dac_l_tx1.read();
			phaseOffset->illumScale = this->reg.illum_scale_l_tx1.read();
		}
		break;
	case 2:
		if (registerSet == 'h') {
			phaseOffset->illumDac = this->reg.illum_dac_h_tx2.read();
			phaseOffset->illumScale = this->reg.illum_scale_h_tx2.read();
		}
		else if (registerSet == 'l') {
			phaseOffset->illumDac = this->reg.illum_dac_l_tx2.read();
			phaseOffset->illumScale = this->reg.illum_scale_l_tx2.read();
		}
		break;
	default:
		break;
	}

	this->reg.tg_en = 0;
	this->reg.en_sequencer = regStore[0];
	this->reg.en_processor_values = regStore[1];
	this->reg.sel_hdr_mode = regStore[2];
	this->reg.sel_tx_ch = regStore[3];
	this->reg.en_adaptive_hdr = regStore[4];
	this->reg.en_tx_switch = regStore[5];
	this->reg.en_phase_corr = regStore[6];
	this->reg.en_nl_corr = regStore[7];
	this->reg.en_temp_corr = regStore[8];
	this->reg.amb_phase_corr_pwl_coeff0 = regStore[9];
	this->reg.amb_phase_corr_pwl_coeff1 = regStore[10];
	this->reg.amb_phase_corr_pwl_coeff2 = regStore[11];
	this->reg.amb_phase_corr_pwl_coeff3 = regStore[12];
	this->reg.shift_illum_phase = regStore[13]; ///* Restores the device state to the same state as before entering this method from the buffered local variables
	this->reg.tg_en = 1;

}

void OPT3101::device::writeDataToEEPROM(uint8_t location, uint8_t data) {
	this->reg.i2c_write_data1 = location;
	this->reg.i2c_write_data2 = data;
	this->reg.i2c_trig_reg = 1;
	host.sleep(1); ///* Added delay to initiate an I2C write transaction
	this->reg.i2c_trig_reg = 0;
	host.sleep(1); ///* Added delay to initiate an I2C write transaction
}

uint8_t OPT3101::device::readDataFromEEPROM(uint8_t location) {
/* Will be updated in future revision of the SDK
	this->reg.i2c_write_data1 = location;
	this->reg.i2c_trig_reg = 1;
	host.sleep(50); ///* Added delay to initiate an I2C write transaction
	this->reg.i2c_trig_reg = 0;
	host.sleep(50); ///* Added delay to initiate an I2C write transaction
	return ((uint8_t) (this->reg.i2c_read_data.read()&0xFF));
*/
	return 0x00;
}

void OPT3101::device::loadIllumCrosstalkSet(bool loadFromFile)
{
	uint8_t c0, c1;
	uint16_t flashLocation;
	/// <b>Algorithm of the method is as follows</b>
#ifdef OPT3101_USE_STDIOLIB
	char fileName[FILENAME_LENGTH];
#endif
	host.printf("INFO::Loading Illumination cross talk Set\r\n");

	// Loop to iterate over all TX channels
	if(loadFromFile){ ///* Checks if input argument loadFromFile is true. If true proceeds to load files in to  OPT3101::device::calibration
		for (c0 = 0; c0 < 3; c0++) {  ///* Loops though all the valid TX channel and register set configurations
			if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
				for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
					if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
#ifdef OPT3101_USE_STDIOLIB
						sprintf(fileName, "%s/illumCrosstalk_TX%d_%c.txt", filePath, c0, c1 ? 'h' : 'l');
						this->calibration->illumCrosstalk[c0][c1].loadFromFile(fileName);  ///* Calls method OPT3101::crosstalkC::loadFromFile to load data from non-volatile memory to all the OPT3101::device::calibration::illumCrosstalk members
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
						flashLocation=(c1+(c0<<1))<<4;
						host.flash.read(flashLocation, (uint8_t*)&this->calibration->illumCrosstalk[c0][c1] ,16);
						host.printf("INFO:Reading Illum Cross talk from Flash location 0x%04x\r\n",flashLocation);
#endif
						this->calibration->illumCrosstalk[c0][c1].printHeader();
						this->calibration->illumCrosstalk[c0][c1].print();
						host.printf("\r\n");

					}
				}
			}
		}
	}

	this->calibration->findCommonCrosstalkScale(&this->calibration->illumCrosstalk[0][0], this->configurationFlags_isTXChannelActive,this->configurationFlags_isTXChannelActive); ///* Finds common crosstalk scale for the illum Crosstalk values captured in OPT3101::device::calibrationC using method OPT3101::calibrationC::findCommonCrosstalkScale
	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) {  ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
				if (this->configurationFlags_isRegisterSetActive[c1])  // Checking if registers are active for this profile
					this->loadIllumCrosstalk(&this->calibration->illumCrosstalk[c0][c1], c0, c1 ? 'h' : 'l'); ///* Loads all the illum crosstalk corresponding to TX and register set configurations to the device from the OPT3101::device::calibrationC::illumCrosstalk member by calling OPT3101::device::loadIllumCrosstalk
			}
		}
	}
}

void OPT3101::device::loadPhaseOffsetSet(bool loadFromFile)
{
	uint8_t c0, c1;
	uint16_t flashLocation;
#ifdef OPT3101_USE_STDIOLIB
	char fileName[FILENAME_LENGTH];
#endif
	host.printf("INFO::Loading Phase Offset Set\r\n");

	/// <b>Algorithm of the method is as follows</b>
	// Loop to iterate over all TX channels
	
	if(loadFromFile){
		for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
			if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile
				for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers
					if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
	#ifdef OPT3101_USE_STDIOLIB
						sprintf(fileName, "%s/phaseOffset_TX%d_%c.txt", filePath, c0, c1 ? 'h' : 'l'); ///* Creates filename  illumCrosstalk_TX{channel}_{h/l}.txt in path filePath
						this->calibration->phaseOffset[c0][c1].loadFromFile(fileName);  ///* Calls method OPT3101::crosstalkC::loadFromFile to store the data
	#endif
	#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
						flashLocation = 96 + 36 * ((c0 << 1) + c1);
						host.flash.read(flashLocation, (uint8_t*)&this->calibration->phaseOffset[c0][c1], 36);
						host.printf("INFO:Reading Phase Offset from TX%d HDR %d to Flash location 0x%04x\r\n", c0, c1, flashLocation);
	#endif // TIHOST
						this->calibration->phaseOffset[c0][c1].printHeader();
						this->calibration->phaseOffset[c0][c1].print();
						host.printf("\r\n");
					}
				}
			}
		}
	}
	
	for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
				if (this->configurationFlags_isRegisterSetActive[c1]) // Checking if registers are active for this profile
					this->loadPhaseOffset(&this->calibration->phaseOffset[c0][c1], c0, c1 ? 'h' : 'l'); ///* Loads all the phase offset corresponding to TX and register set configurations to the device from the OPT3101::device::calibrationC::phaseOffset member by calling OPT3101::device::loadPhaseOffset
			}
		}
	}
}

uint8_t OPT3101::device::determineConfigCount() {
	return this->calibration->recordLength;
}

void OPT3101::device::loadIllumCrosstalkTempCoffSet()
{
	uint8_t c0, c1;

	host.printf("INFO::Loading Illumination crosstalk Temperature Coefficient Set\r\n");
	this->loadIllumCrosstalkSet(true); ///* Calls OPT3101::device::loadIllumCrosstalkSet with the same loafFromFile flag specified to this method
	this->calibration->findCrosstalkTempRegisterValues(&this->calibration->illumCrosstalkTempCoff[0][0], this->configurationFlags_isTXChannelActive,this->configurationFlags_isTXChannelActive, &this->calibration->illumCrosstalk[0][0]); ///* Calls OPT3101::calibrationC::findCrosstalkTempRegisterValues based on the members from OPT3101::calibrationC class OPT3101::calibrationC::illumCrosstalkTempCoff and OPT3101::calibrationC::illumCrosstalk
	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
				if (this->configurationFlags_isRegisterSetActive[c1]) // Checking if registers are active for this profile
					this->loadIllumCrosstalkTempCoff(&this->calibration->illumCrosstalkTempCoff[c0][c1], c0, c1 ? 'h' : 'l'); ///* Calls the method OPT3101::device::loadIllumCrosstalkTempCoff to load all the crosstalk temp coff values from the OPT3101::calibrationC::illumCrosstalkTempCoff member
			}
		}
	}
	this->reg.en_temp_xtalk_corr=1;
}

void OPT3101::device::loadPhaseOffsetTempCoffSet() {
	uint8_t c0, c1;

	host.printf("INFO::Loading Phase Temperature coefficient Set\r\n");
	this->calibration->findPhaseTempRegisterValues(&this->calibration->phaseTempCoff[0][0], this->configurationFlags_isTXChannelActive,this->configurationFlags_isTXChannelActive, this->reg.freq_count_read_reg.read()); ///* Calls OPT3101::calibrationC::findPhaseTempRegisterValues based on the members from OPT3101::calibrationC class OPT3101::calibrationC::phaseTempCoff and current device register OPT3101::registers::freq_count_read_reg

	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
				if (this->configurationFlags_isRegisterSetActive[c1]) // Checking if registers are active for this profile
					this->loadPhaseOffsetTempCoff(&this->calibration->phaseTempCoff[c0][c1], c0, c1 ? 'h' : 'l');  ///* Calls the method OPT3101::device::loadPhaseOffsetTempCoff to load all the phase temp coff values from the OPT3101::calibrationC::phaseTempCoff member
			}
		}
	}
}

void OPT3101::device::loadPhaseAmbientCoffSet()
{
	host.printf("INFO::Loading Phase Ambient Coefficient Set\r\n");
	this->calibration->phaseAmbientCoff[0].calculateCoff(this->reg.freq_count_read_reg.read()); ///* Calculates the phase ambient coff by calling method OPT3101::phaseAmbientCoffC::calculateCoff also passing argument of device register OPT3101::registers::freq_count_read_reg
	this->loadPhaseAmbientCoff(&this->calibration->phaseAmbientCoff[0]); ///* Calls method OPT3101::device::loadPhaseAmbientCoff to load ambient coff to device from OPT3101::calibrationC::phaseAmbientCoff member
}

void OPT3101::calibrationC::findCommonCrosstalkScale(OPT3101::crosstalkC* illumXtalk, bool *txActiveList,bool *registerSetActiveList) {
	uint8_t maxScale, c0,c1;
	maxScale = 0;
	/// <b>Algorithm of the method is as follows</b>
	for (c0 = 0; c0 < 3; c0++) {
		for (c1 = 0; c1 < 2; c1++){
			if(txActiveList[c0] && registerSetActiveList[c1])
				maxScale = illumXtalk[(c0<<1)+c1].xtalkScale > maxScale ? illumXtalk[(c0<<1)+c1].xtalkScale : maxScale; ///* Finds the largest scale what will fit all the input OPT3101::crosstalkC arguments
		}
	}
	for (c0 = 0; c0 < 3; c0++) {
		for (c1 = 0; c1 < 2; c1++){
			if(txActiveList[c0] && registerSetActiveList[c1])
				illumXtalk[(c0<<1)+c1].commonScale = maxScale; ///* Assigns the identified maxScale to all the input argument illum Crosstalk pointers OPT3101::crosstalkC
		}
	}
}

OPT3101::calibrationC::calibrationC(bool dummyFlag) {
	/// <b>Algorithm of the method is as follows</b>
	///* Allocates memory for OPT3101::calibrationC::internalCrosstalk size based on system configuration 
	///* Allocates memory for OPT3101::calibrationC::illumCrosstalk size based on system configuration 
	///* Allocates memory for OPT3101::calibrationC::phaseOffset size based on system configuration 
	///* Allocates memory for OPT3101::calibrationC::illumCrosstalkTempCoff size based on system configuration 
	///* Allocates memory for OPT3101::calibrationC::phaseTempCoff size based on system configuration 
	///* Allocates memory for OPT3101::calibrationC::phaseAmbientCoff size based on system configuration 
	///* Sets the member OPT3101::calibrationC::registerAddressListSize based on number of calibration registers requires 
	///* Allocates memory for OPT3101::calibrationC::registerAddressList based on  OPT3101::calibrationC::registerAddressListSize
	///* Sets up the flag OPT3101::calibrationC::EEPROM_connected based on configuration
	///* Sets up the flag OPT3101::calibrationC::extTempSensor_connected based on configuration
}
