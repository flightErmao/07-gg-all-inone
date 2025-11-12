/*
Copyright (c) 2018 Texas Instruments Incorporated
All rights reserved not granted herein.
Limited License.  
Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI (TI Devices).  No hardware patent is licensed hereunder.
Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution
Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:
*	No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.
*	any redistribution and use are licensed by TI for use only with TI Devices.
*	Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:
*	any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.
*	any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.
Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.
DISCLAIMER.
THIS SOFTWARE IS PROVIDED BY TI AND TI's LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "OPT3101device.h"


/* OPT3101 device initialization Sequence Header file written by OPT3101 Calibration tool - Version 0.8.0
By: a0227156
On: 2019-10-02 19:54:11
Configuration Settings:
{
	"genDeviceI2CMasterConnection": 1,
	"autoHDRMode": true,
	"monoshotMode": false,
	"TX0HCurrent": 0,
	"genDeviceI2CMasterTempSensor": false,
	"TXDCCurrent": 0,
	"superHDRMode": false,
	"LEDOrder": 73746,
	"timeStamp": 2019-10-02 19:53:28,
	"genDeviceAmbientSupport": 5,
	"freqComp": false,
	"user": a0227156,
	"autoHDRRatio": 4.0,
	"multiLEDMode": true,
	"TX1HCurrent": 0,
	"genDeviceEEPROMConnection": true,
	"toolVersion": 0.8.0,
	"TX2HCurrent": 0,
	"crc": -1193321828,
	"genDeviceSubFrameCount": 256,
	"autoHDRHThreshold": 25500,
	"genDeviceAvgFrameCount": 6,
}
*/
void OPT3101::device::initialize(void){
	// List of registers to initialize OPT3101 device after power-up

	this->reg.tg_ovl_window_start = 7000; // //Overload flab observation window
	this->reg.en_temp_conv = 1; // //Enables the internal

	this->reg.clip_mode_fc = 1; // //Enables Clip mode for Frequency correction
	this->reg.clip_mode_temp = 0; // //Disables Clip mode for Temp coff phase correction
	this->reg.clip_mode_offset = 0; // //Disables Clip mode for phase offset correction
	this->reg.iq_read_data_sel = 3; // //Enables 16 bit frame counter
	this->reg.iamb_max_sel = 14; // //Sets maximum ambient support
	this->reg.en_temp_corr = 1; // //Enables Temperature Correction
	this->reg.gpio1_obuf_en=1; // //Enabled output buffer on GPIO1 pin
	this->reg.gpo1_mux_sel=2; 	    // //select dig_gpo_0 on gpio1
	this->reg.dig_gpo_sel0 = 9; 	// //Select Data Ready on dig_gpo_0

	this->reg.num_sub_frames = 255; // //Sub frames count
	this->reg.num_avg_sub_frames = 127; // //Average frames count
	this->reg.xtalk_filt_time_const = 3; // //Crosstalk filter time constant
	this->reg.tg_seq_int_start 	= 9850; // //Sequence Start
	this->reg.tg_seq_int_end 		= 9858; // //Sequence End
	this->reg.tg_seq_int_mask_start 	= 127; // //Same as AvgFrame Count
	this->reg.tg_seq_int_mask_end 	= 127; // //Same as AvgFrame Count

	this->reg.hdr_thr_high = 25500; // //High Threshold
	this->reg.hdr_thr_low = 5875; // //Low Threshold
	this->reg.en_adaptive_hdr = 1; // //Enables adaptive HDR feature

	this->reg.illum_dac_h_tx0 = 31; // //High Current settings [173.6mA:5.6mA X 31]
	this->reg.illum_scale_h_tx0 = 0; // //Illum scale for H [173.6mA:5.6mA X 31]

	this->reg.illum_dac_l_tx0 = 31; // //High Current settings [043.4mA:1.4mA X 31]
	this->reg.illum_scale_l_tx0 = 3; // //Illum scale for H [043.4mA:1.4mA X 31]

	this->reg.illum_dac_h_tx1 = 31; // //High Current settings [173.6mA:5.6mA X 31]
	this->reg.illum_scale_h_tx1 = 0; // //Illum scale for H [173.6mA:5.6mA X 31]

	this->reg.illum_dac_l_tx1 = 31; // //High Current settings [043.4mA:1.4mA X 31]
	this->reg.illum_scale_l_tx1 = 3; // //Illum scale for H [043.4mA:1.4mA X 31]

	this->reg.illum_dac_h_tx2 = 31; // //High Current settings [173.6mA:5.6mA X 31]
	this->reg.illum_scale_h_tx2 = 0; // //Illum scale for H [173.6mA:5.6mA X 31]

	this->reg.illum_dac_l_tx2 = 31; // //High Current settings [043.4mA:1.4mA X 31]
	this->reg.illum_scale_l_tx2 = 3; // //Illum scale for H [043.4mA:1.4mA X 31]

	this->reg.tx_seq_reg   = 2340; // //Setting TX Switching order
	this->reg.en_tx_switch = 1 ; // //Enable TX Switching order

	this->reg.tg_en = 1; // //Enables Timing Generator

	this->configurationFlags_xtalkFilterTau =  3; // //This is not a register but a settings flag for the SDK
	this->configurationFlags_monoshotMode =  false; // //This is not a register but a settings flag for the SDK
	this->configurationFlags_xtalkSettlingOneTauInMilliSeconds      =  512; // //This is not a register but a settings flag for the SDK
	this->configurationFlags_xtalkSettlingOneTauInDataReadyCounts   =  8; // //This is not a register but a settings flag for the SDK
	this->configurationFlags_frameTimeInMilliSeconds                =  64; // //This is not a register but a settings flag for the SDK
	this->configurationFlags_avgFrameCountExponentOfTwo             =  7; // //This is not a register but a settings flag for the SDK

	// tx2 xtalk compensation
	//this->reg.EN_CTALK_FB_CLK=1;
	//this->reg.EN_CALIB_CLK=1;
	//this->reg.calib_curr1_en_I=1;
	//this->reg.calib_curr1_gain_sel=0;
	//this->reg.calib_curr1_DAC_I=4;
	//this->reg.calib_curr1_inv_CLK_I=0;

}

OPT3101::device::device(void):
configurationFlags_isTXChannelActive{true,true,true},
configurationFlags_isRegisterSetActive{true,true} {}

OPT3101::calibrationC::calibrationC(void) : calibrationC(true) {
	this->recordLength           = 6; // //This configuration requires 6 crosstalk and other configuration record(s)
	this->registerAddressListSize= 43; // //This configuration requires 43 registers [1376] bits to be stored for calibration
	this->EEPROM_connected       =  true; // //This configuration helps configure EEPROM
	this->extTempSensor_connected=  false; // //This configuration helps configure Ext temp sensor
	this->registerAddressList[0] = 0x2f; // //Address for register(s) iphase_xtalk_reg_hdr0_tx0,temp_coeff_main_hdr1_tx1
	this->registerAddressList[1] = 0x30; // //Address for register(s) qphase_xtalk_reg_hdr0_tx0,temp_coeff_main_hdr1_tx1
	this->registerAddressList[2] = 0x38; // //Address for register(s) temp_coeff_xtalk_iphase_hdr0_tx0,qphase_xtalk_reg_hdr0_tx2
	this->registerAddressList[3] = 0x39; // //Address for register(s) temp_coeff_xtalk_qphase_hdr0_tx0,iphase_xtalk_reg_hdr1_tx2
	this->registerAddressList[4] = 0x42; // //Address for register(s) phase_offset_hdr0_tx0
	this->registerAddressList[5] = 0x47; // //Address for register(s) tmain_calib_hdr0_tx0,tillum_calib_hdr0_tx0
	this->registerAddressList[6] = 0x45; // //Address for register(s) temp_coeff_main_hdr0_tx0,tmain_calib_hdr1_tx2
	this->registerAddressList[7] = 0x46; // //Address for register(s) temp_coeff_illum_hdr0_tx0,tillum_calib_hdr1_tx2
	this->registerAddressList[8] = 0x31; // //Address for register(s) iphase_xtalk_reg_hdr1_tx0,temp_coeff_main_hdr0_tx2
	this->registerAddressList[9] = 0x32; // //Address for register(s) qphase_xtalk_reg_hdr1_tx0,temp_coeff_main_hdr0_tx2
	this->registerAddressList[10] = 0x5e; // //Address for register(s) temp_coeff_xtalk_iphase_hdr1_tx0,temp_coeff_xtalk_iphase_hdr0_tx1
	this->registerAddressList[11] = 0x60; // //Address for register(s) temp_coeff_xtalk_qphase_hdr1_tx0,temp_coeff_xtalk_qphase_hdr0_tx1,temp_coeff_xtalk_qphase_hdr1_tx1
	this->registerAddressList[12] = 0x51; // //Address for register(s) phase_offset_hdr1_tx0,temp_coeff_illum_hdr1_tx0
	this->registerAddressList[13] = 0x48; // //Address for register(s) tmain_calib_hdr1_tx0,tillum_calib_hdr1_tx0
	this->registerAddressList[14] = 0x2d; // //Address for register(s) temp_coeff_main_hdr1_tx0,temp_coeff_main_hdr0_tx1
	this->registerAddressList[15] = 0x52; // //Address for register(s) temp_coeff_illum_hdr1_tx0,phase_offset_hdr0_tx1
	this->registerAddressList[16] = 0x33; // //Address for register(s) iphase_xtalk_reg_hdr0_tx1,temp_coeff_main_hdr1_tx2
	this->registerAddressList[17] = 0x34; // //Address for register(s) qphase_xtalk_reg_hdr0_tx1,temp_coeff_main_hdr1_tx2
	this->registerAddressList[18] = 0x49; // //Address for register(s) tmain_calib_hdr0_tx1,tillum_calib_hdr0_tx1
	this->registerAddressList[19] = 0x53; // //Address for register(s) temp_coeff_illum_hdr0_tx1,phase_offset_hdr1_tx1
	this->registerAddressList[20] = 0x54; // //Address for register(s) temp_coeff_illum_hdr0_tx1,phase_offset_hdr0_tx2
	this->registerAddressList[21] = 0x35; // //Address for register(s) iphase_xtalk_reg_hdr1_tx1
	this->registerAddressList[22] = 0x36; // //Address for register(s) qphase_xtalk_reg_hdr1_tx1
	this->registerAddressList[23] = 0x5f; // //Address for register(s) temp_coeff_xtalk_iphase_hdr1_tx1,temp_coeff_xtalk_iphase_hdr0_tx2,temp_coeff_xtalk_iphase_hdr1_tx2
	this->registerAddressList[24] = 0x41; // //Address for register(s) tmain_calib_hdr1_tx1
	this->registerAddressList[25] = 0x43; // //Address for register(s) tillum_calib_hdr1_tx1,en_phase_corr,en_temp_corr,scale_phase_temp_coeff
	this->registerAddressList[26] = 0x55; // //Address for register(s) temp_coeff_illum_hdr1_tx1,phase_offset_hdr1_tx2
	this->registerAddressList[27] = 0x56; // //Address for register(s) temp_coeff_illum_hdr1_tx1
	this->registerAddressList[28] = 0x37; // //Address for register(s) iphase_xtalk_reg_hdr0_tx2
	this->registerAddressList[29] = 0x61; // //Address for register(s) temp_coeff_xtalk_qphase_hdr0_tx2,temp_coeff_xtalk_qphase_hdr1_tx2
	this->registerAddressList[30] = 0x3f; // //Address for register(s) tmain_calib_hdr0_tx2,tillum_calib_hdr0_tx2
	this->registerAddressList[31] = 0x57; // //Address for register(s) temp_coeff_illum_hdr0_tx2
	this->registerAddressList[32] = 0x58; // //Address for register(s) temp_coeff_illum_hdr0_tx2
	this->registerAddressList[33] = 0x3a; // //Address for register(s) qphase_xtalk_reg_hdr1_tx2,scale_temp_coeff_xtalk
	this->registerAddressList[34] = 0x59; // //Address for register(s) temp_coeff_illum_hdr1_tx2
	this->registerAddressList[35] = 0x5a; // //Address for register(s) temp_coeff_illum_hdr1_tx2
	this->registerAddressList[36] = 0x2e; // //Address for register(s) illum_xtalk_reg_scale
	this->registerAddressList[37] = 0x71; // //Address for register(s) shift_illum_phase
	this->registerAddressList[38] = 0xb5; // //Address for register(s) scale_amb_phase_corr_coeff
	this->registerAddressList[39] = 0x0c; // //Address for register(s) amb_phase_corr_pwl_coeff0
	this->registerAddressList[40] = 0xb4; // //Address for register(s) amb_phase_corr_pwl_coeff1,amb_phase_corr_pwl_coeff2,amb_phase_corr_pwl_coeff3
	this->registerAddressList[41] = 0xb8; // //Address for register(s) amb_phase_corr_pwl_x0,amb_phase_corr_pwl_x1
	this->registerAddressList[42] = 0xb9; // //Address for register(s) amb_phase_corr_pwl_x2

}

