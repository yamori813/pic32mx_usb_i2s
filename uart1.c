/*

UART1 Driver File for PIC24.

********************************************************************************
 FileName:        uart1.c
 Dependencies:    HardwareProfile.h
 Processor:       PIC24
 Compiler:        MPLAB C30
 Linker:          MPLAB LINK30
 Company:         Microchip Technology Incorporated

Author                Date      Comment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
KO                 12-Feb-2008  Modified to use HardwareProfile.h
KO                 11-Oct-2006  v1.0
Anton Alkhimenok   18-Oct-2005
Anton Alkhimenok   17-Feb-2009  Added UART1Char2Hex(), UART1Hex2Char(), 
									  UART1ClearError(), UART1DataReceived()
PAT				   27-Jan-2010  Added UART1GetBaudError() for dynamic checking
								of baud rate percentage error.

********************************************************************************
Software License Agreement

Microchip Technology Inc. ("Microchip") licenses to you the right to use, copy,
modify and distribute the software - including source code - only for use with
Microchip microcontrollers or Microchip digital signal controllers; provided
that no open source or free software is incorporated into the Source Code
without Microchip�s prior written consent in each instance.

The software is owned by Microchip and its licensors, and is protected under
applicable copyright laws.  All rights reserved.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING
BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

********************************************************************************
*/

#include "Compiler.h"
#include "HardwareProfile.h"
#include "uart1.h"

//******************************************************************************
// Constants
//******************************************************************************

//U1BRG register value and baudrate mistake calculation

#if defined (__PIC32MX__)
    #define BAUDRATEREG2        ((GetPeripheralClock()+(BRG_DIV2/2*BAUDRATE2))/BRG_DIV2/BAUDRATE2-1)
#else
    #error Cannot calculate BRG value
#endif    

#if defined (__PIC32MX__)
    #define BAUD_ACTUAL         (GetPeripheralClock()/BRG_DIV2/(BAUDRATEREG2+1))
#else
    #error Cannot calculate actual baud rate
#endif 

	#define BAUD_ERROR              ((BAUD_ACTUAL > BAUDRATE2) ? BAUD_ACTUAL-BAUDRATE2 : BAUDRATE2-BAUD_ACTUAL)
	#define BAUD_ERROR_PERCENT      ((BAUD_ERROR*100+BAUDRATE2/2)/BAUDRATE2)
	
/*******************************************************************************
Function: UART1GetBaudError()

Precondition:
    None.

Overview:
    This routine checks the UART baud rate error percentage and returns it.

Input: None.

Output: Returns the baud rate error in percent.

*******************************************************************************/
char UART1GetBaudError()
{
    unsigned int errorPercent = 0;

	errorPercent = ((BAUD_ERROR*100+BAUDRATE2/2)/BAUDRATE2);
    return (char)errorPercent;
}


/*******************************************************************************
Function: UART1GetChar()

Precondition:
    UART1Init must be called prior to calling this routine.

Overview:
    This routine waits for a byte to be received.  It then returns that byte.

Input: None.

Output: Byte received.

*******************************************************************************/
char UART1GetChar()
{
    char Temp;

    while(IFS1bits.U1RXIF == 0);

    Temp = U1RXREG;
    IFS1bits.U1RXIF = 0;
    return Temp;
}

/*	----------------------------------------------------------------------------
	SystemUnlock() perform a system unlock sequence
	--------------------------------------------------------------------------*/

void SystemUnlock()
{
	SYSKEY = 0;				// ensure OSCCON is locked
	SYSKEY = 0xAA996655;	// Write Key1 to SYSKEY
	SYSKEY = 0x556699AA;	// Write Key2 to SYSKEY
}

/*	----------------------------------------------------------------------------
	SystemLock() relock OSCCON by relocking the SYSKEY
	--------------------------------------------------------------------------*/

void SystemLock()
{
	SYSKEY = 0x12345678;	// Write any value other than Key1 or Key2
}

#define	__SERIAL__	Yes


// All Analog Pins as Digital IOs
static	void IOsetDigital()
{
	#ifdef __32MX220F032B__
	DDPCONbits.JTAGEN=0;		// check : already in system.c
	ANSELA = 0;
	ANSELB = 0;
//	ANSELC = 0;
	#else
	AD1PCFG = 0xFFFF;
	#endif
}

//	PinguinoX.3��MX220��TX/RX��PORT DIRECTION��ݒ肵�Ă��Ȃ�.
//	X.4�ł͏C������Ă���.

void PORT_DIR_init()
{
	TRISBbits.TRISB4 = 0;	//OUTPUT;	// RB4 / U1TX output
	TRISAbits.TRISA4 = 1;	//INPUT;	// RA4 / U1RX input
}




static void IO_Remap()
{
	SystemUnlock();
	CFGCONbits.IOLOCK=0;			// unlock configuration
	CFGCONbits.PMDLOCK=0;
	#ifdef __SERIAL__
		U1RXRbits.U1RXR=2;			// Define U1RX as RA4 ( UEXT SERIAL )
		RPB4Rbits.RPB4R=1;			// Define U1TX as RB4 ( UEXT SERIAL )
	#endif
	#ifdef __SPI__
		SDI1Rbits.SDI1R=5;			// Define SDI1 as RA8 ( UEXT SPI )
		RPA9Rbits.RPA9R=3;			// Define SDO1 as RA9 ( UEXT SPI )
	#endif
	#ifdef __PWM__
		RPC2Rbits.RPC2R  =0b0101;	// PWM0 = OC3 as D2  = RC2
		RPC3Rbits.RPC3R  =0b0101;	// PWM1 = OC4 as D3  = RC3
		RPB5Rbits.RPB5R  =0b0101;	// PWM2 = OC2 as D11 = RB5
		RPB13Rbits.RPB13R=0b0110;	// PWM3 = OC5 as D12 = RB13
		RPB15Rbits.RPB15R=0b0101;	// PWM4 = OC1 as D13 = RB15
	#endif
	CFGCONbits.IOLOCK=1;			// relock configuration
	CFGCONbits.PMDLOCK=1;	
	SystemLock();
}
/*******************************************************************************
Function: UART1Init()

Precondition: None.

Overview:
    This routine sets up the UART1 module.

Input: None.

Output: None.

Notes:
    Allow the peripheral to set the I/O pin directions.  If we set the TRIS
    bits manually, then when we disable the UART, the shape of the stop bit
    changes, and some terminal programs have problems.
*******************************************************************************/
void UART1Init()
{
	IOsetDigital();
	IO_Remap();
	PORT_DIR_init();

    U1BRG = BAUDRATEREG2;
    U1MODE = 0;
    U1MODEbits.BRGH = BRGH2;
    U1STA = 0;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    IFS1bits.U1RXIF = 0;
    
    #if defined (__PIC32MX__)
        U1STAbits.URXEN = 1;
    #endif
}

/*******************************************************************************
Function: UART1IsPressed()

Precondition:
    UART1Init must be called prior to calling this routine.

Overview:
    This routine checks to see if there is a new byte in UART reception buffer.

Input: None.

Output:
    0 : No new data received.
    1 : Data is in the receive buffer

*******************************************************************************/
char UART1IsPressed()
{
    if(IFS1bits.U1RXIF == 1)
        return 1;
    return 0;
}

/*******************************************************************************
Function: UART1PrintString( char *str )

Precondition:
    UART1Init must be called prior to calling this routine.

Overview:
    This function prints a string of characters to the UART.

Input: Pointer to a null terminated character string.

Output: None.

*******************************************************************************/
void UART1PrintString( char *str )
{
    unsigned char c;

    while( (c = *str++) )
        UART1PutChar(c);
}

/*******************************************************************************
Function: UART1PutChar( char ch )

Precondition:
    UART1Init must be called prior to calling this routine.

Overview:
    This routine writes a character to the transmit FIFO, and then waits for the
    transmit FIFO to be empty.

Input: Byte to be sent.

Output: None.

*******************************************************************************/
void UART1PutChar( char ch )
{
    U1TXREG = ch;
    #if !defined(__PIC32MX__)
        Nop();
        Nop();
    #endif
    while(U1STAbits.TRMT == 0);
}

/*******************************************************************************
Function: UART1PutDec(unsigned char dec)

Precondition:
    UART1Init must be called prior to calling this routine.

Overview:
    This function converts decimal data into a string and outputs it to UART.

Input: Binary data.

Output: None.

*******************************************************************************/
void  UART1PutDec(unsigned char dec)
{
    unsigned char res;
    unsigned char printed_already = 0;

    res = dec;

    if (res/100)
    {
        UART1PutChar( res/100 + '0' );
        printed_already = 1;
    }
    res = res - (res/100)*100;

    if ((res/10) || (printed_already == 1))
    {
        UART1PutChar( res/10 + '0' );
    }
    res = res - (res/10)*10;

    UART1PutChar( res + '0' );
}

/*******************************************************************************
Function: UART1PutHex

Precondition:
    UART1Init must be called prior to calling this routine.

Overview:
    This function converts hex data into a string and outputs it to UART.

Input: Binary data.

Output: None.

*******************************************************************************/

const unsigned char CharacterArray[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void UART1PutHex( int toPrint )
{
    int printVar;

    printVar = toPrint;
    toPrint = (toPrint>>4) & 0x0F;
    UART1PutChar( CharacterArray[toPrint] );

    toPrint = printVar & 0x0F;
    UART1PutChar( CharacterArray[toPrint] );

    return;
}

void UART1PutHex4( int d )
{
	UART1PutHex(d>>24);
	UART1PutHex(d>>16);
	UART1PutHex(d>>8);
	UART1PutHex(d);
}

/*********************************************************************
Function: void UART1ClrError(void)

PreCondition: none

Input: none

Output: character received

Side Effects: none

Overview: wait for character

Note: none

********************************************************************/
void UART1ClrError(void){
    // Clear error flag
    if(U1STAbits.OERR)
		U1STAbits.OERR = 0;
}

