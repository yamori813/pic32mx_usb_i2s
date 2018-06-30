/********************************************************************
 FileName:	i2s.c (ak4645a.c)
 Dependencies:  See INCLUDES section
 Processor:	PIC32 USB Microcontrollers
 Hardware:	This demo is natively intended to be used on Microchip USB demo
 		boards supported by the MCHPFSUSB stack.  See release notes for
 		support matrix. This demo can be modified for use on other
		hardware platforms.
 Complier:  	Pinguino gcc
 Company:	Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
********************************************************************/

#include <string.h>

#include "HardwareProfile.h"

#define	I2S_I2C_MODULE	I2C1

INT I2CWrite(BYTE addr, BYTE reg, BYTE data);

INT I2CWrite(BYTE addr, BYTE reg, BYTE data)
{

	I2CEnable(I2S_I2C_MODULE, TRUE);
	
	while(I2CBusIsIdle(I2S_I2C_MODULE) == FALSE);

	if (I2CStart(I2S_I2C_MODULE) != I2C_SUCCESS)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(I2S_I2C_MODULE, FALSE); 
		return(-1);
	}

	while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_START));  

	if (I2CSendByte(I2S_I2C_MODULE,addr << 1) != I2C_SUCCESS)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP));  
		I2CEnable(I2S_I2C_MODULE, FALSE);
		return(-1);
	}

	while(!I2CTransmissionHasCompleted(I2S_I2C_MODULE));

	if (I2CByteWasAcknowledged(I2S_I2C_MODULE) == FALSE)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(I2S_I2C_MODULE, FALSE); 
		return(-1);
	}

	if (I2CSendByte(I2S_I2C_MODULE,reg) != I2C_SUCCESS)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(I2S_I2C_MODULE, FALSE); 
		return(-1);
	}

	while(!I2CTransmissionHasCompleted(I2S_I2C_MODULE));

	if (I2CByteWasAcknowledged(I2S_I2C_MODULE) == FALSE)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(I2S_I2C_MODULE, FALSE); 
		return(-1);
	}

	if (I2CSendByte(I2S_I2C_MODULE,data) != I2C_SUCCESS)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP));  
		I2CEnable(I2S_I2C_MODULE, FALSE);
		return(-1);
	}

	while(!I2CTransmissionHasCompleted(I2S_I2C_MODULE));

	if (I2CByteWasAcknowledged(I2S_I2C_MODULE) == FALSE)
	{
		I2CStop(I2S_I2C_MODULE);
		while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP));  
		I2CEnable(I2S_I2C_MODULE, FALSE);
		return(-1);
	}

	I2CStop(I2S_I2C_MODULE);
	while(!(I2CGetStatus(I2S_I2C_MODULE) & I2C_STOP));  

	I2CEnable(I2S_I2C_MODULE, FALSE);
	
	return(1);	

}

