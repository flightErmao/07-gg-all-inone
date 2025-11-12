/*!
* \file OPT3101device_Calibration.cpp
* \author  Karthik Rajagopal <krthik@ti.com>
* \version 0.9
*
* \section COPYRIGHT
* TEXAS INSTRUMENTS TEXT FILE LICENSE
* Copyright (c) 2018 Texas Instruments Incorporated
* All rights reserved not granted herein.
* Limited License.
* Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
* Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution
* Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:
* *	No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.
* *	any redistribution and use are licensed by TI for use only with TI Devices.
* *	Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
* If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:
* *	any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.
* *	any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.
* DISCLAIMER.
* THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* \section DESCRIPTION
* The file contains methods definitions for top level calibration methods for OPT3101::device 
*/

#include "OPT3101device.h"

void OPT3101::device::calibrationSession_firstTimeBringUp() {
	/// <b>Algorithm of the method is as follows</b>
	this->reset(); ///* Resets the device calling OPT3101::device::reset method
	host.printf("INFO::Writing Initialization sequence I2C registers\r\n");
	this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
	this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
	this->calibration->internalCrosstalk[0].printHeader(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis
	this->calibration->internalCrosstalk[0].print(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis
	host.printf("\r\n");
													  // The folowing step is to perfrom illum crosstalk measurements
	this->measureIllumCrosstalkSet();
	this->loadIllumCrosstalkSet();
	this->measurePhaseOffsetSet();
	this->loadPhaseOffsetSet();
	host.printf("INFO::Completed First Time bring up\r\n");


}

void OPT3101::device::calibrationSession_perUnitFactoryCalibrationWriteRegisterDataToNonVolatileMemory(bool DEBUG_dry_run)
{
	uint16_t c0;
	uint8_t regStore[11], c1;
	uint32_t data;
	uint8_t datar;

	/// <b>Algorithm of the method is as follows</b>
	if (!this->calibration->EEPROM_connected){///* Method returns without doing any operation when the device configuration flag OPT3101::device::EEPROM_connected is false. User needs to implement their own function if no EEPROM is present
		host.printfSetColor(0b110);
		host.printf("WARN::NO EEPROM connected to h/w.\r\n");
		host.printfSetColor(0xFF);
		host.printf("INFO::Listing all registers to be written to host non-volatile memory\r\n");
		host.printfSetColor(0b001);
		host.printf("regAddr,regValues\r\n");
		for(c0=0;c0<this->calibration->registerAddressListSize;c0++){
			data=host.readI2C(this->calibration->registerAddressList[c0]);
			host.printf("   0x%02x,",this->calibration->registerAddressList[c0]);
			host.printf(" 0x%06lx\r\n",data);
		}
		host.printfSetColor(0xFF);
		host.printf("INFO::User is expected to program these values to non-volatile memory in host\r\n");
		return;
	}
	regStore[0] = this->reg.tsens_slave0.read();
	regStore[1] = this->reg.i2c_en.read();
	regStore[2] = this->reg.i2c_num_tran.read();
	regStore[3] = this->reg.i2c_rw.read();
	regStore[4] = this->reg.i2c_num_bytes_tran1.read(); ///* Critical registers which are modified in this method are read from the h/w and temporarily bufferd to local variables.
	regStore[5] = this->reg.en_tx_switch.read();
	regStore[6] = this->reg.sel_tx_ch.read();
	regStore[7] = this->reg.frame_vd_trig.read();
	regStore[8] = this->reg.en_processor_values.read();
	regStore[9] = this->reg.en_sequencer.read();
	regStore[10] = this->reg.en_adaptive_hdr.read();

	this->reg.tsens_slave0 = 0x50;
	this->reg.i2c_num_tran = 0;
	this->reg.i2c_rw = 0;
	this->reg.frame_vd_trig = 0;
	this->reg.i2c_num_bytes_tran1 = 1; ///* OPT3101 device is configured to write desired data though the SDA_M/SCL_M lines to the connected external EEPROM
	this->reg.en_tx_switch = 0;
	this->reg.sel_tx_ch = 0;
	this->reg.i2c_en = 1;
	this->reg.en_processor_values = 0;
	this->reg.en_sequencer = 0;
	this->reg.en_adaptive_hdr = 0;
									   // This portions clears all the element in EEPROM to 0xFF
    if(!DEBUG_dry_run) {
        host.printf("INFO::Clearing EEPROM contents\r\n");
        for (c0 = 0; c0 < 256; c0++)
            this->writeDataToEEPROM((uint8_t) c0, 0xFF); ///* Erases the EEPROM with  0xFF data in all lcoations.
	} else {
	    host.printf("INFO::Dry Run, Skipped Clear EEPROM contents\r\n");
	}

	for (c0 = 0; c0 < this->calibration->registerAddressListSize; c0++) {
	    if(!DEBUG_dry_run)
	        this->writeDataToEEPROM( (uint8_t) ((c0&0xFF)<<2), this->calibration->registerAddressList[c0]);
		data = host.readI2C(this->calibration->registerAddressList[c0]); ///* Reads all the registers from the list OPT3101::calibrationC::registerAddressList from h/w and writes the address and data to the connected external EEPROM
		host.printf("INFO::Writing reg:0x%02x ",this->calibration->registerAddressList[c0]);
		host.printf("data:0x%06lx to EEPROM\r\n",data);
		if(!DEBUG_dry_run) {
            for (c1 = 0; c1<3; c1++){
                this->writeDataToEEPROM((uint8_t) ((((uint8_t)(c0&0xFF))<<2)+c1+1), (uint8_t) ((data >> ((c1 << 3)) & 0xFF)));
            }
		}
	}

	this->reg.tsens_slave0 = regStore[0];
	this->reg.i2c_num_tran = regStore[2];
	this->reg.i2c_rw = regStore[3];
	this->reg.i2c_num_bytes_tran1 = regStore[4]; ///* Restores the device state to the same state as before entering this method from the buffered local variables
	this->reg.en_tx_switch = regStore[5];
	this->reg.sel_tx_ch = regStore[6];
	this->reg.frame_vd_trig = regStore[6];
	this->reg.i2c_en = regStore[1];
    this->reg.en_processor_values = regStore[8];
    this->reg.en_sequencer = regStore[9];
    this->reg.en_adaptive_hdr = regStore[10];
}

void OPT3101::device::readAndPrintEEPROMContents(bool rawFormat)
{
/* This section will be updated in the next revision of the SDK
 * 	uint8_t c1;
	uint16_t c0;
	uint8_t regStore[8],data[4];
	/// <b>Algorithm of the method is as follows</b>
	if (!this->calibration->EEPROM_connected) ///* Method returns without doing any operation when the device configuration flag OPT3101::device::EEPROM_connected is false. User needs to implement their own function if no EEPROM is present
		return;
	regStore[0] = this->reg.tsens_slave0.read();
	regStore[1] = this->reg.i2c_en.read();
	regStore[2] = this->reg.i2c_num_tran.read();
	regStore[3] = this->reg.i2c_rw.read();
	regStore[4] = this->reg.i2c_num_bytes_tran1.read(); ///* Critical registers which are modified in this method are read from the h/w and temporarily bufferd to local variables.
	regStore[5] = this->reg.en_tx_switch.read();
	regStore[6] = this->reg.sel_tx_ch.read();
	regStore[7] = this->reg.frame_vd_trig.read();

	this->reg.tsens_slave0 = 0x50;
	this->reg.i2c_num_tran = 0;
	this->reg.i2c_rw = 1;
	this->reg.i2c_num_bytes_tran1 = 0; ///* OPT3101 device is configured to read desired data though the SDA_M/SCL_M lines to the connected external EEPROM
	this->reg.en_tx_switch = 0;
	this->reg.frame_vd_trig = 0;
	this->reg.sel_tx_ch = 0;
	this->reg.i2c_en = 1;

	// This portions reads the data from the EERPOM and displays the same
	for (c0 = 0; c0 < 256; c0++){
		host.printf("S:0x%02x,",c0);
		host.printf("0x%02x\r\n",this->readDataFromEEPROM((uint8_t) c0)); ///* Erases the EEPROM with  0xFF data in all lcoations.
	}

	for (c0 = 0; c0 < 64; c0++) {
		for(c1=0;c1<4;c1++){
			data[c1] = this->readDataFromEEPROM((c0<<2)+c1); ///* Reads the data from EEPROM For all locations.
			if(rawFormat)
				host.printf("INFO::Loc:0x%02x Data:0x%02x\r\n",(c0<<2)+c1,data[c1]);
		}
		if(!rawFormat){
			host.printf("INFO:: regAddr:0x%02x regData:0x",data[0]);
			for(c1=1;c1<4;c1++)
				host.printf("%02x",data[c1]);
			host.printf("\r\n");
		}
	}

	this->reg.tsens_slave0 = regStore[0];
	this->reg.i2c_en = regStore[1];
	this->reg.i2c_num_tran = regStore[2];
	this->reg.i2c_rw = regStore[3];
	this->reg.i2c_num_bytes_tran1 = regStore[4]; ///* Restores the device state to the same state as before entering this method from the buffered local variables
	this->reg.en_tx_switch = regStore[5];
	this->reg.sel_tx_ch = regStore[6];
	this->reg.frame_vd_trig = regStore[6];
 */

}

void OPT3101::device::calibrationSession_perDesignTx2IllumXtalkCorrection() {
    /// <b>Algorithm of the method is as follows</b>
    this->reset(); ///* Resets the device calling OPT3101::device::reset method
    this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
    this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
    ///* Ensure all calibrations are zeroed.
    this->reg.use_xtalk_reg_illum = 0;
    this->reg.en_temp_xtalk_corr = 0;
    this->reg.en_temp_corr = 0;
    this->reg.en_phase_corr = 0;
    this->reg.amb_phase_corr_pwl_coeff0 = 0;
    this->reg.amb_phase_corr_pwl_coeff1 = 0;
    this->reg.amb_phase_corr_pwl_coeff2 = 0;
    this->reg.amb_phase_corr_pwl_coeff3 = 0;
    OPT3101::crosstalkC illumXtalk; ///* Declares temporary variable of OPT3101::crosstalkC class to hold internal crosstalk data across temperature, TX channel and register settings
    ///* print header for data table
    host.printf("  I,rS,rI,rD");
    illumXtalk.printHeader();
    ///* print baseline original xtalk
    host.printf("-%02d,%02d,%02d,%02d,",0,-1,-1,-1);
    this->measureIllumCrosstalk(&illumXtalk, 2, 1 ? 'h' : 'l'); ///* Calls method OPT3101::device::measureIllumCrosstalk with temporary variable of OPT3101::crosstalkC class
    illumXtalk.print(); ///* Calls OPT3101::crosstalkC::report method to report the crosstalk on screen
    ///* enable current on tx2
    this->reg.EN_CTALK_FB_CLK=1;
    this->reg.EN_CALIB_CLK=1;
    this->reg.calib_curr1_en_I=1;
    ///* loop through all currents and print rest of the data as shown below
    for(uint8_t scale = 0; scale < 4; scale++) {
        for(uint8_t current = 0; current < 16; current++) {
            for(uint8_t direction = 0; direction < 2; direction++) {
                ///* set current on tx2
                this->reg.calib_curr1_gain_sel=scale; //2 bits
                this->reg.calib_curr1_DAC_I=current; //4 bits
                this->reg.calib_curr1_inv_CLK_I=direction; //1 bit
                ///* print current
                if(direction==0)
                    host.printf("-%02d,%02d,%02d,%02d,",current*(scale+1),scale,current,direction);
                else
                    host.printf("+%02d,%02d,%02d,%02d,",current*(scale+1),scale,current,direction);
                //                                      tx2,hdr1
                this->measureIllumCrosstalk(&illumXtalk, 2, 1 ? 'h' : 'l'); ///* Calls method OPT3101::device::measureIllumCrosstalk with temporary variable of OPT3101::crosstalkC class
                illumXtalk.print(); ///* Calls OPT3101::crosstalkC::report method to report the crosstalk on screen
            }
        }
    }
}

void OPT3101::device::calibrationSession_perDesignCalibrationCrosstalkTemp() {
	uint8_t c0, c1;
	uint16_t count;
	/// <b>Algorithm of the method is as follows</b>
	OPT3101::crosstalkC illumXtalk; ///* Declares temporary variable of OPT3101::crosstalkC class to hold internal crosstalk data across temperature, TX channel and register settings
	this->reset(); ///* Resets the device calling OPT3101::device::reset method
	this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
	this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
	envController.setTargetToInfinity_OR_coverPhotodiode(); ///* Calls the method environmentalController::setTargetToInfinity_OR_coverPhotodiode , which is expected to set the environment so as to be able to measure illumination crosstalk
	envController.setChamberTemperature(70); ///* Calls the method environmentalController::setChamberTemperature , which is expected to set the chamber to desired temperature

	host.printf("Count,TX ,HDR,");
	illumXtalk.printHeader();
	for (count = 0; count<TEMP_CYCLE_TOTAL_NUMBER_OF_DATA_POINTS_PER_SETTING; count++)  { ///* Loops through total number of points to be data printed
		// Loop to iterate over all TX channels
		for (c0 = 0; c0 < 3; c0++) {  ///* Loops though all the valid TX channel and register set configurations
			if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
				for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers
					if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
						this->measureIllumCrosstalk(&illumXtalk, c0, c1 ? 'h' : 'l'); ///* Calls method OPT3101::device::measureIllumCrosstalk with temporary variable of OPT3101::crosstalkC class
						host.printf("%05d,  %d,  %d,",count,c0,c1);
						illumXtalk.print(); ///* Calls OPT3101::crosstalkC::report method to report the crosstalk on screen
					}
				}
			}
		}
		host.sleep(TEMP_CYCLE_DELAY_IN_SECONDS_BETWEEN_DATA_POINTS<<10);
	}
}

void OPT3101::device::calibrationSession_perDesignCalibrationPhaseTemp() {
	/// <b>Algorithm of the method is as follows</b>
	uint8_t c0, c1;
	OPT3101::frameData data;
	uint16_t count;
	uint16_t refDistanceInCodes;
	OPT3101::phaseOffsetC phaseOffset; ///* Declares temporary variable of OPT3101::phaseOffsetC class to hold phase offset data across temperature, TX channel and register settings
	uint32_t refDistanceInMM;
	this->reset(); ///* Resets the device calling OPT3101::device::reset method
	this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
	this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
	this->loadIllumCrosstalkSet(true);

	this->manuallySetIllumCrosstalkTempCoffs();
	this->loadIllumCrosstalkTempCoffSet();

	envController.manuallySetReferenceDistances();


	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
				if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
					refDistanceInMM = envController.refDistancesInMM[c0][c1]; ///* <b>Warning</b> User is expected to select and set reference distance so that the amplitude of the system for this particular TX and register set configurations measures between 16K and 24K. Default is set to 0 in the SDK
					envController.setTargetDistance(refDistanceInMM); ///* Calls environmentalController::setTargetDistance method with the specified distance
					envController.setChamberTemperature(70);  ///* Calls the method environmentalController::setChamberTemperature , which is expected to set the chamber to desired temperature
					refDistanceInCodes = (refDistanceInMM * 4477) >> 10; ///* Converts the reference distance specified in codes related to OPT3101::frameData::phase ADC codes
					host.printf("DataCt,TX,HDR,");
					phaseOffset.data.printHeader();
					for (count = 0; count < TEMP_CYCLE_TOTAL_NUMBER_OF_DATA_POINTS_PER_SETTING; count++) {  ///* Loops through chamber temperature settings to get temp coff
						host.printf("%06d, %d,  %d,",count,c0,c1);
						this->measurePhaseOffset(&phaseOffset, c0, c1 ? 'h' : 'l', refDistanceInCodes, 0);
						phaseOffset.data.print();
						host.sleep(TEMP_CYCLE_DELAY_IN_SECONDS_BETWEEN_DATA_POINTS<<10);
					}
				}
			}
		}
	}
}

void OPT3101::device::calibrationSession_perDesignCalibrationPhaseAmbient() {
	/// <b>Algorithm of the method is as follows</b>
	uint8_t c0, c1;
	OPT3101::frameData data;
	uint16_t count;
	uint16_t refDistanceInCodes;
	OPT3101::phaseOffsetC phaseOffset; ///* Declares temporary variable of OPT3101::phaseOffsetC class to hold phase offset data across temperature, TX channel and register settings
	uint32_t refDistanceInMM;
	bool breakFlag;

	/// <b>Algorithm of the method is as follows</b>

	this->reset(); ///* Resets the device calling OPT3101::device::reset method
	this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
	this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
	this->manuallySetIllumCrosstalkTempCoffs();
	this->manuallySetPhaseTempCoffs();

	this->loadIllumCrosstalkSet(true);
	this->loadIllumCrosstalkTempCoffSet();

	this->loadPhaseOffsetSet(true);
	this->loadPhaseOffsetTempCoffSet();
	envController.manuallySetReferenceDistances();

	breakFlag = false;
	// Loop to iterate over all TX channels
	for (c0 = 0; c0 < 3; c0++) { ///* Loops though all the valid TX channel and register set configurations
		if (this->configurationFlags_isTXChannelActive[c0]) { // Checking is TX channel is active for this profile 
			for (c1 = 0; c1 < 2; c1++) { // Loop to iterate over the H/L registers 
			    c1 = 0;
			    c0 = 1;
				if (this->configurationFlags_isRegisterSetActive[c1]) { // Checking if registers are active for this profile
					refDistanceInMM = envController.refDistancesInMM[c0][c1]; ///* <b>Warning</b> User is expected to select and set reference distance so that the amplitude of the system for this particular TX and register set configurations measures between 16K and 24K. Default is set to 0 in the SDK
					envController.setTargetDistance(refDistanceInMM); ///* Calls environmentalController::setTargetDistance method with the specified distance
					envController.setAmbientLight(0);  ///* Calls the method environmentalController::setChamberTemperature , which is expected to set the chamber to desired temperature
					refDistanceInCodes = (refDistanceInMM * 4477) >> 10; ///* Converts the reference distance specified in codes related to OPT3101::frameData::phase ADC codes
					host.printf("DataCt,TX,HDR,");
					phaseOffset.data.printHeader();
					for (count = 0; count < TEMP_CYCLE_TOTAL_NUMBER_OF_DATA_POINTS_PER_SETTING; count++) {  ///* Loops through chamber temperature settings to get temp coff
						host.printf("%06d, %d,  %d,",count,c0,c1);
						this->measurePhaseOffset(&phaseOffset, c0, c1 ? 'h' : 'l', refDistanceInCodes, 0);
						phaseOffset.data.print();
						host.sleep(TEMP_CYCLE_DELAY_IN_SECONDS_BETWEEN_DATA_POINTS<<10);
					}
					breakFlag = true; ///* Since Phase ambient coff is required to be done only for one TX configuration the the method breaks from the loop after 1 ambient sweep
					break; // This is required only for one TX and 1 of the registers since this is property of photo diode.
						   // Measure and populate Ambient Coefficients 
				}
				if (breakFlag)
					break;
			}
			if (breakFlag)
				break;
		}
		if (breakFlag)
			break;
	}
}

void OPT3101::device::calibrationSession_perUnitFactoryCalibration()
{

	this->reset(); ///* Resets the device calling OPT3101::device::reset method

	host.printf("INFO::Writing Initialization sequence I2C registers\r\n");
	this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
	host.printf("INFO::Device Initialization Completed\r\n");
	this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
	this->calibration->internalCrosstalk[0].report(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis

	this->manuallySetIllumCrosstalkTempCoffs();
	this->manuallySetPhaseTempCoffs();
	this->manuallySetPhaseAmbientCoffs();

	this->measureIllumCrosstalkSet(false);
	this->loadIllumCrosstalkSet(false); ///* Calls the OPT3101::device::loadIllumCrosstalkSet with false argument so that method to load all illum crosstalk settings from the OPT3101::device::crosstalk::illumCrosstalk member
	this->loadIllumCrosstalkTempCoffSet();

	this->measurePhaseOffsetSet(false);
	this->loadPhaseOffsetSet(false); ///* Calls the OPT3101::device::loadPhaseOffsetSet method with false argument so as to load all the phase offset registers from the OPT3101::device::calibration::phaseOffset instance instead of files
	this->loadPhaseOffsetTempCoffSet();

	this->loadPhaseAmbientCoffSet(); ///* Calls the OPT3101::device::loadPhaseAmbientCoffSet method to load all the phase offset temp coff

	this->calibrationSession_perUnitFactoryCalibrationWriteRegisterDataToNonVolatileMemory(); ///* Calls the OPT3101::device::calibrationSession_perUnitFactoryCalibrationWriteRegisterDataToNonVolatileMemory to store the calibration data to a non-volatile memory
}

void OPT3101::device::perUnitFactoryUpdateIllumXtalk()
{

    this->reset(); ///* Resets the device calling OPT3101::device::reset method

    host.printf("INFO::Writing Initialization sequence I2C registers\r\n");
    this->initialize(); ///* Initializes the OPT3101 device by calling OPT3101::device::initialize method
    host.printf("INFO::Device Initialization Completed\r\n");
    this->measureAndCorrectInternalCrosstalk(&this->calibration->internalCrosstalk[0]); ///* Calls method OPT3101::device::measureAndCorrectInternalCrosstalk with argument OPT3101::calibrationC::internalCrosstalk
    this->calibration->internalCrosstalk[0].report(); ///* Calls the method OPT3101::crosstalkC::report for debug and data analysis

    this->measureIllumCrosstalkSet(false);
    this->loadIllumCrosstalkSet(false); ///* Calls the OPT3101::device::loadIllumCrosstalkSet with false argument so that method to load all illum crosstalk settings from the OPT3101::device::crosstalk::illumCrosstalk member

    this->calibrationSession_perUnitFactoryCalibrationWriteRegisterDataToNonVolatileMemory(); ///* Calls the OPT3101::device::calibrationSession_perUnitFactoryCalibrationWriteRegisterDataToNonVolatileMemory to store the calibration data to a non-volatile memory
}
