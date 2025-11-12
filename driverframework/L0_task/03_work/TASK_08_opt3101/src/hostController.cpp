/*!
* \file hostController.cpp
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
* This file contains the hostController class methods
*/

#include "hostController.h"
#include "string.h"
#include <stdarg.h>

#ifdef _WIN32
#ifdef OPT3101_USE_SERIALLIB
#define WINPAUSE system("pause")
#endif
#endif


hostController host;


#ifdef OPT3101_USE_SERIALLIB
/** \brief Serial Command Port  declaration
This global variable declaration with name OPT3101commandPort of class serial::Serial is used by class like OPT3101::deviceRegister for I2C read and writes.
*/
//serial::Serial OPT3101commandPort("COM4", 9600, serial::Timeout::simpleTimeout(1000));
#ifdef linux
serial::Serial OPT3101I2CCommandPort("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(1000));
#endif

#ifdef _WIN32
serial::Serial OPT3101I2CCommandPort("COM20", 9600, serial::Timeout::simpleTimeout(1000));
#endif //WIN32
#endif //SERIALLIB


#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
volatile uint8_t bCDCDataReceived_event = false; // Indicates data has been rx'ed without an open rx operation
volatile uint8_t bDataReceiveCompleted_event = false;
volatile uint8_t bDataSendCompleted_event = false;
#endif

hostController::hostController(void){
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	this->initialize();
#endif;
}

void hostController::writeI2C(uint8_t address, uint32_t data) {
	/// <b>Algorithm of the method is as follows</b>
#if defined(OPT3101_USE_STDIOLIB) && defined(OPT3101_USE_SERIALLIB)
	std::string returnValue;
	char writeData[20];
	sprintf(writeData, "REGWx%02xx%06x\r", address, data); /// * Creates WRITE I2C command to send to h/w with address and data specified in arguments
	OPT3101I2CCommandPort.write((uint8_t*)writeData, strlen(writeData)); /// * Writes the WRITE I2C command to h/w
	returnValue = OPT3101I2CCommandPort.readline();
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	this->i2c.write(address,data);
#endif

}
uint32_t hostController::readI2C(uint8_t address) {
	uint32_t i2cReadValue=0;
#if defined(OPT3101_USE_STDIOLIB) && defined(OPT3101_USE_SERIALLIB)
	char writeData[10];
	std::string returnValue;
	uint8_t c0;
	
	/// <b>Algorithm of the method is as follows</b>
	sprintf(writeData, "REGRx%02x\r", address); /// * Creates READ I2C command to send to h/w with address and data specified in arguments

	i2cReadValue = 0;

	OPT3101I2CCommandPort.write((uint8_t*)writeData, strlen(writeData)); /// * Writes the READ I2C command to h/w
	returnValue = OPT3101I2CCommandPort.readline(); /// * Waits and receives response from h/w
	if (returnValue.length() == 18) {
		returnValue = returnValue.substr(10, 6);
		c0 = 0;
		for (std::string::iterator it = returnValue.begin(); it != returnValue.end(); ++it) { /// * Converters the received string output from h/w to uint32_t value
			if (*it >= '0' && *it <= '9')
				i2cReadValue += ((*it) - 48) << ((5 - c0) * 4);
			else if (*it >= 'A' && *it <= 'F')
				i2cReadValue += ((*it) - 65 + 10) << ((5 - c0) * 4);
			else if (*it >= 'a' && *it <= 'f')
				i2cReadValue += ((*it) - 97 + 10) << ((5 - c0) * 4);
			c0++;
		}
	}
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	this->i2c.read(address,&i2cReadValue);
#endif
	return i2cReadValue; /// * Returns the data in uint32_t format
}

void hostController::sleep(uint32_t timeInMilliSeconds) {
	/// <b>Algorithm of the method is as follows</b>

#if defined(HOST_PC) && defined(_WIN32) && defined(OPT3101_USE_SERIALLIB)	
	Sleep(timeInMilliSeconds);/// * Sleeps for the time specified in the argument timeInMilliSeconds
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	uint32_t c;
	for(c=0;c<timeInMilliSeconds;c++)
		__delay_cycles(24000);

#endif

}
void hostController::sleepDataReadyCounts(uint16_t dataReadyCounts) {
	/// <b>Algorithm of the method is as follows</b>
	/// * Currently empty function needs to be implemented by user. Based on interrupts from data ready signal from OPT3101, the host needs to count those pulses and wait until dataReadyCounts have reached. 
}

void hostController::pause()
{
	/// <b>Algorithm of the method is as follows</b>
	this->printf("Press any Key to continue:\r\n");

#if defined(HOST_PC) && defined(_WIN32) && defined(OPT3101_USE_SERIALLIB)
	WINPAUSE; 	/// * Pause for user input
#endif

#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
    while(!bCDCDataReceived_event);
	bCDCDataReceived_event=false;
    if(USBCDC_getBytesInUSBBuffer(CDC0_INTFNUM))
        USBCDC_rejectData(CDC0_INTFNUM);

#endif

}


void hostController::printfSetColor(uint8_t color){
#ifdef PRINT_COLOR_TERMINAL
	switch(color){
		case 0xFF:
			host.printf("\u001b[0m");
			break;
		case 0b000:
			host.printf("\u001b[30m");
			break;
		case 0b100:
			host.printf("\u001b[31m");
			break;
		case 0b010:
			host.printf("\u001b[32m");
			break;
		case 0b001:
			host.printf("\u001b[34m");
			break;
		case 0b110:
			host.printf("\u001b[33m");
			break;
		case 0b101:
			host.printf("\u001b[35m");
			break;
		case 0b011:
			host.printf("\u001b[36m");
			break;
		case 0b111:
			host.printf("\u001b[37m");
			break;
		default:
			host.printf("\u001b[0m");
			break;
	}
#endif
}



void hostController::resetDevice() {
	// These comments rest the device on power-up
#if defined(HOST_PC) && defined(OPT3101_USE_STDIOLIB) && defined(OPT3101_USE_SERIALLIB)
	char writeData[10];
	/// <b>Algorithm of the method is as follows</b>
	sprintf(writeData, "DEVR\r"); /// * Creates a command which specifies host to send RESET Pulse to OPT3101 h/w 
	OPT3101I2CCommandPort.write((uint8_t*)writeData, strlen(writeData)); /// * Send the RESET command to the h/w
#endif
#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	this->gpio.rstz=0;
	this->sleep(1);
	this->gpio.rstz=1;
#endif

}

void hostController::initialize(){
#if defined(HOST_PC) && defined(OPT3101_USE_STDIOLIB) && defined(OPT3101_USE_SERIALLIB)
	char writeData[16];
	/// <b>Algorithm of the method is as follows</b>
	sprintf(writeData, "DEVAx%02x\r", OPT3101_I2C_SLAVEADDRESS); /// * Creates a command which specifies host to send RESET Pulse to OPT3101 h/w 
	OPT3101I2CCommandPort.write((uint8_t*)writeData, strlen(writeData)); /// * Send the RESET command to the h/w
#endif

#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	WDT_A_hold(WDT_A_BASE);
	USB_setup(true,true); // Enables the USP with event handling
	PMM_setVCore(PMM_CORE_LEVEL_3); // Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
	USBHAL_initClocks(24000000);	 // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
	UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ); // Turns the XT2 crystal clock ON
	UCS_turnOnSMCLK(); // Turns the SMCLK On
	UCS_initClockSignal(
					 UCS_ACLK,
					 UCS_XT2CLK_SELECT,
					 UCS_CLOCK_DIVIDER_32); // Enabling clock to come on pin 14 P1.0/ACLK. This enables 4MHz/32 which is 125KHz on the ACLK pin
	this->i2c.init();
	this->gpio.init();
	this->gpio.rstz=1;

	__enable_interrupt();  // Enable interrupts globally
#endif


}


void hostController::printf(const char *fmt, ...){
#ifdef VERBOSE_MODE
    va_list args;
	va_start(args, fmt);
	char oBuffer[256];
	vsnprintf(oBuffer, sizeof oBuffer, fmt, args);

#ifdef TIMSP430F5529_LAUNCHPAD_CALIBRATION_TOOL
	USBCDC_sendDataInBackground((uint8_t*)oBuffer,strlen(oBuffer),CDC0_INTFNUM,0xFF);
	__delay_cycles(24000);
#endif
#ifdef HOST_PC
	std::printf("%s",oBuffer);
#endif
	va_end(args);
#endif // VERBOSE MODE

}

