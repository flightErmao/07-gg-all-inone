/*!
* \file OPT3101PhaseOffset.cpp
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
* This file contains the method definitions for class OPT3101::phaseOffsetC
*/

#include "OPT3101PhaseOffset.h"
#include "hostController.h"


OPT3101::phaseOffsetC::phaseOffsetC()
{
	/// <b>Algorithm of the method is as follows</b>
	this->phaseOffset = 0; ///* Initializes all members to 0
	this->referenceDistanceInCodes = 0; 
	this->illumScale = 0; 
	this->illumDac=0; 
	this->illumDCdac = 0; 
	this->freqCount = 0; 
	this->shiftIllumPhase=0; 
}

void OPT3101::phaseOffsetC::report()
{
	host.printf("-------------------------\r\n");
	host.printf("Phase Offset Class Report\r\n");
	host.printf("-------------------------\r\n"); ///* Prints all the members and values of members on screen.
	host.printf("phaseOffset=%d\r\n", this->phaseOffset);
	host.printf("referenceDistanceInCodes=%d\r\n", this->referenceDistanceInCodes);
	host.printf("illumScale=%d\r\n", this->illumScale);
	host.printf("illumDac=%d\r\n", this->illumDac);
	host.printf("illumDCdac=%d\r\n", this->illumDCdac);
	host.printf("freqCount=%d\r\n", this->freqCount);
	host.printf("shiftIllumPhase=%d\r\n", this->shiftIllumPhase);
	host.printf("data=\r\n");
	this->data.report();
	host.printf("-------------------------\r\n"); ///* Prints all the members and values of members on screen.
}

void OPT3101::phaseOffsetC::printHeader()
{
	host.printfSetColor(0b001);
	host.printf("\r\nPhOfft,");
	host.printf("rfDCod,");
	host.printf("ilmScl,");
	host.printf("ilmDac,");
	host.printf(" fqCnt\r\n");
	host.printfSetColor(0xFF);
}
void OPT3101::phaseOffsetC::print()
{
	host.printfSetColor(0b001);
	host.printf("%06u,",this->phaseOffset);
	host.printf("%06u,",this->referenceDistanceInCodes);
	host.printf("     %01u,",this->illumScale);
	host.printf("    %02u,",this->illumDac);
	host.printf(" %05u,\r\n",this->freqCount);
	host.printfSetColor(0xFF);
	this->data.printHeader();
	this->data.print();
}


#ifdef OPT3101_USE_STDIOLIB
void OPT3101::phaseOffsetC::storeToFile(char * fileName)
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
void OPT3101::phaseOffsetC::loadFromFile(char * fileName)
{
#ifdef OPT3101_USE_STREAMLIB
	/// <b>Algorithm of the method is as follows</b>
	std::ifstream ifs(fileName);
	ifs >> this;
	ifs.close(); ///* User needs to implement file load/restore based on host. 
#endif
}
#endif

#ifdef OPT3101_USE_STREAMLIB
std::ostream & OPT3101::operator<<(std::ostream & os, const phaseOffsetC * data)
{
	/// <b>Algorithm of the method is as follows</b>
	os << data->phaseOffset<<'\n'; ///* Serializes all the members and returns to the stream 
	os << data->referenceDistanceInCodes << '\n';
	os << data->illumScale << '\n';
	os << data->illumDac << '\n';
	os << data->illumDCdac << '\n';
	os << data->freqCount << '\n';
	os << data->shiftIllumPhase << '\n';
	os << &data->data << '\n';
	return os;
}
#endif

#ifdef OPT3101_USE_STREAMLIB
std::istream & OPT3101::operator>>(std::istream & is, phaseOffsetC * data)
{
	/// <b>Algorithm of the method is as follows</b>
	is >> data->phaseOffset; ///* Reads from stream and de-serializes all the members 
	is >> data->referenceDistanceInCodes;
	is >> data->illumScale;
	is >> data->illumDac;
	is >> data->illumDCdac;
	is >> data->freqCount;
	is >> data->shiftIllumPhase;
	is >> &data->data;
	return is;
}
#endif
