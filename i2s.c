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

#include "i2s.h"

INT I2SControl(I2SState* pCodecHandle, I2S_REGISTER controlRegister, INT command);

volatile INT 	data_index_tx=0;
volatile INT 	data_index_value_tx=0;
volatile INT	pllkUpdate=0;
volatile INT 	data_available_count=0;

I2SState* pCodecHandlePriv = NULL;

I2SState*
I2SOpen()
{
	UINT spiFlags=0;
	
	pCodecHandlePriv = (I2SState*)malloc(sizeof(I2SState));
	if (pCodecHandlePriv == NULL)
		return(NULL);

		pCodecHandlePriv->txBuffer =
			(AudioStereo*)malloc(I2S_TX_BUFFER_SIZE_BYTES);
		if (pCodecHandlePriv->txBuffer == NULL)
			return(NULL);
		memset(pCodecHandlePriv->txBuffer, 0,
			I2S_TX_BUFFER_SIZE_BYTES);
		
		DmaChnConfigure(I2S_SPI_TX_DMA_CHANNEL,
			DMA_CHN_PRI0, DMA_CONFIG_DEFAULT);		
		DmaChnSetEventControl(I2S_SPI_TX_DMA_CHANNEL, 
			DMA_EV_START_IRQ_EN |
			DMA_EV_START_IRQ(I2S_SPI_MODULE_TX_IRQ));
													
		DmaChnSetTxfer(	I2S_SPI_TX_DMA_CHANNEL,
			(void*)pCodecHandlePriv->txBuffer,
			(void*)&I2S_SPI_MODULE_BUFFER,
			I2S_TX_BUFFER_SIZE_BYTES>>1, 
			sizeof(UINT16),
			sizeof(UINT16)	);	
	
		DmaChnSetEvEnableFlags(I2S_SPI_TX_DMA_CHANNEL,
			DMA_EV_BLOCK_DONE);
		
		//INTSetVectorPriority(I2S_SPI_TX_DMA_VECTOR, I2S_SPI_TX_DMA_INT_PRI_LEVEL);
		//INTSetSubPriority(I2S_SPI_TX_DMA_VECTOR, I2S_SPI_TX_DMA_INT_SPRI_LEVEL);
		IPC10bits.DMA0IP=I2S_SPI_TX_DMA_INT_PRI_LEVEL;
		IPC10bits.DMA0IS=I2S_SPI_TX_DMA_INT_SPRI_LEVEL;
	
		INTClearFlag(I2S_SPI_TX_DMA_INTERRUPT);		
							
    // http://chipkit.net/forum/viewtopic.php?f=6&t=3137&start=10
    /* The following code example will initialize the SPI1 Module in I2S Master mode. */
    /* It assumes that none of the SPI1 input pins are shared with an analog input. */
    unsigned int rData;
    IEC0CLR = 0x03800000; // disable all interrupts
    IFS1bits.SPI1TXIF = 0;
    SPI1CON = 0; // Stops and resets the SPI1.
    SPI1CON2 = 0; // Reset audio settings
    SPI1BRG = 0; // Reset Baud rate register
    rData = SPI1BUF; // clears the receive buffer
    
    SPI1STATCLR = 0x40; // clear the Overflow
    SPI1CON2 = 0x00000080; // I2S Mode, AUDEN = 1, AUDMON = 0
    SPI1CON2bits.IGNROV = 1; // Ignore Receive Overflow bit (for Audio Data Transmissions)
    SPI1CON2bits.IGNTUR = 1; //  Ignore Transmit Underrun bit (for Audio Data Transmissions) 1 = A TUR is not a critical error and zeros are transmitted until thSPIxTXB is not empty 0 = A TUR is a critical error which stop SPI operation
    
    SPI1CONbits.ENHBUF = 1; // 1 = Enhanced Buffer mode is enabled
    SPI1CON = 0x00000060; // Master mode, SPI ON, CKP = 1, 16-bit audio channel
    SPI1CONbits.STXISEL = 0b11;
    SPI1CONbits.DISSDI = 1; // 0 = Disable SDI bit
    SPI1CONSET = 0x00008000;
    
    IFS1CLR = 0x000000f0;
    IPC7CLR = 0x1F000000;
    IPC7SET = 0x1C000000;

    // REFCLK is used by the Baud Rate Generator
    SPI1CONbits.MCLKSEL = 1;
    // 16-bit Data, 16-bit FIFO, 32-bit Channel/64-bit Frame
    SPI1CONbits.MODE32 = 0; 
    SPI1CONbits.MODE16 = 1; 
    // Baud Rate Generator
    SPI1BRG = 1;
    
    IEC1bits.SPI1TXIE = 0;

    // data, 32 bits per frame
    // from here, the device is ready to receive and transmit data
    /* Note: A few of bits related to frame settings are not required to be set in the SPI1CON */
    /* register during audio mode operation. Please refer to the notes in the SPIxCON2 register.*/
    PPSUnLock;
    PPSOutput(3, RPA4, REFCLKO);	// MCLK
    PPSOutput(1, RPB4, SS1);		// LRCLK
    PPSOutput(2, RPA1,SDO1);		// SDATA
    // RB14 is BCLK
    PPSLock;

	return(pCodecHandlePriv);
}

void I2SStartAudio(I2SState *pCodecHandle, BOOL enable)
{

	if (enable == TRUE) {

		pllkUpdate=0;
		data_available_count=0;		
		
		data_index_tx=0;
		data_index_value_tx=0;
		
		pCodecHandle->activeTxBuffer 	= PP_BUFF0;
		pCodecHandle->statusTxBuffer[0] = TRUE;
		pCodecHandle->statusTxBuffer[1] = FALSE;
		pCodecHandle->countTxBuffer[0] 	= DMA_PP_BUFFER_SIZE;
		pCodecHandle->countTxBuffer[1] 	= DMA_PP_BUFFER_SIZE;
		DmaChnClrEvFlags(I2S_SPI_TX_DMA_CHANNEL, DMA_EV_BLOCK_DONE);
		//INTEnable( I2S_SPI_TX_DMA_INTERRUPT, INT_ENABLED);
		IEC1bits.DMA0IE=1;

		SpiChnEnable(I2S_SPI_MODULE, TRUE);
		DmaChnEnable(I2S_SPI_TX_DMA_CHANNEL);
		while(!DCH0CONbits.CHEN)
			DCH0CONbits.CHEN=1;

    DMACONCLR = 0x8000; // disable entire DMA.
    IEC1bits.DMA0IE = 1;
    IFS1bits.DMA0IF = 0;
    IPC10bits.DMA0IP = 7;   // Setting DMA0 at highest priority.
    IPC10bits.DMA0IS = 3;   // Setting DMA0 at highest sub-priority.
    DMACONSET = 0x8000; // enable DMA.
    DCH0CON = 0x0000;
    DCRCCON = 0x00; // 
    DCH0INTCLR = 0xff00ff; // clear DMA0 interrupts register.
    DCH0INTbits.CHSDIE = 1; // DMA0 Interrupts when source done enabled.
    DCH0ECON = 0x00;
    DCH0ECONbits.CHSIRQ = _SPI1_TX_IRQ; // DMA0 transfer triggered by which interrupt? (On PIC32MX - it is by _IRQ suffix!)
    DCH0ECONbits.AIRQEN = 0; // do not enable DMA0 transfer abort interrupt.
    DCH0ECONbits.SIRQEN = 1; // enable DMA0 transfer start interrupt.
    DCH0CONbits.CHAEN = 0; // DMA Channel 0 is always disabled right after the transfer.
    DCH0CONbits.CHEN = 1;  // DMA Channel 0 is enabled. 

	} else if (enable == FALSE) {
		DmaClrGlobalFlags(DMA_GFLG_ON);
		DmaChnDisable(I2S_SPI_TX_DMA_CHANNEL);
		SpiChnEnable(I2S_SPI_MODULE, FALSE);
	}
}


INT I2SControl(I2SState* pCodecHandle, I2S_REGISTER controlRegister, INT command)
{

	BYTE byte1  = (BYTE) (controlRegister & 0xFF);
	BYTE byte2  = (BYTE) (command & 0xFF);


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

	if (I2CSendByte(I2S_I2C_MODULE,I2S_I2C_ADDRESS) != I2C_SUCCESS)
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

	if (I2CSendByte(I2S_I2C_MODULE,byte1) != I2C_SUCCESS)
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

	if (I2CSendByte(I2S_I2C_MODULE,byte2) != I2C_SUCCESS)
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

UINT I2SWritePPBuffer(I2SState* pCodecHandle, AudioStereo* data, UINT nStereoSamples)
{
	UINT i, nWrittenSamples = 0;
	PINGPONG_BUFFN usePPBuffer;
	AudioStereo* dest;

	if (nStereoSamples == 0) return(0);
	
	if (data_index_value_tx<pCodecHandle->bufferSize)
		data_index_value_tx += nStereoSamples;
	else
		data_index_value_tx = 0;
	
	usePPBuffer = (pCodecHandle->activeTxBuffer == PP_BUFF0) ?
	    PP_BUFF1 : PP_BUFF0;
	if (pCodecHandle->statusTxBuffer[usePPBuffer] == TRUE)
		return(0);

	nWrittenSamples = (nStereoSamples > DMA_PP_BUFFER_SIZE) ?
	    DMA_PP_BUFFER_SIZE : nStereoSamples;

	dest = (usePPBuffer == PP_BUFF0) ? &pCodecHandle->txBuffer[0]
	    : &pCodecHandle->txBuffer[DMA_PP_BUFFER_SIZE];

	int overflow = 0;
	for(i = 0; i < nWrittenSamples; i++){	
		dest[data_index_tx].audioWord = data[i].audioWord;
		data_index_tx++;
		if (data_index_tx == pCodecHandle->bufferSize){
			if (dest ==  &pCodecHandle->txBuffer[0])
				dest = &pCodecHandle->txBuffer[DMA_PP_BUFFER_SIZE];
			else
				dest = &pCodecHandle->txBuffer[0];
			data_index_tx = 0;
			overflow = 1;
		}
	}

	if (overflow) {
		data_index_value_tx=pCodecHandle->bufferSize+data_index_tx;
		pCodecHandle->countTxBuffer[usePPBuffer] = pCodecHandle->bufferSize;	  
		pCodecHandle->statusTxBuffer[usePPBuffer] = TRUE;
	}

	return(nWrittenSamples);	
}

UINT I2SWrite(I2SState* pCodecHandle, AudioStereo* data, UINT nStereoSamples) 
{
	UINT writtenSamples = 0;
	writtenSamples = I2SWritePPBuffer(pCodecHandle, &data[0], nStereoSamples);
	return(writtenSamples);
}


INT I2SBufferClear(I2SState* pCodecHandle){
	if (pCodecHandlePriv->txBuffer != NULL)		
		memset(pCodecHandlePriv->txBuffer, 0, I2S_TX_BUFFER_SIZE_BYTES);

	return(1);
}

void I2SAdjustSampleRateTx(I2SState* pCodecHandle)
{
	
	data_available_count=(pCodecHandle->bufferSize-(volatile INT)(DmaChnGetSrcPnt(I2S_SPI_TX_DMA_CHANNEL)>>2))+data_index_value_tx;

	if (data_available_count>pCodecHandle->underrunLimit && data_available_count<pCodecHandle->overrunLimit){
			
		if (data_available_count<pCodecHandle->underrunCount)
			I2STuneSampleRate(pCodecHandle, DEC_TUNE);
		else if (data_available_count>pCodecHandle->overrunCount)
			I2STuneSampleRate(pCodecHandle, INC_TUNE);
	}
		
}


INT I2STuneSampleRate(I2SState* pCodecHandle, TUNE_STEP tune_step)
{
	 
	INT command;
	INT result=-1;

	if ((pllkUpdate-pCodecHandle->pllkValue)<(-1*pCodecHandle->pllkTuneLimit))
		pllkUpdate=pCodecHandle->pllkValue-pCodecHandle->pllkTuneLimit+pCodecHandle->pllkTune;	
	else if ((pllkUpdate-pCodecHandle->pllkValue)>pCodecHandle->pllkTuneLimit)
		pllkUpdate=pCodecHandle->pllkValue+pCodecHandle->pllkTuneLimit-pCodecHandle->pllkTune;
		

	pllkUpdate=(tune_step==INC_TUNE)?(pllkUpdate-pCodecHandle->pllkTune):(pllkUpdate+pCodecHandle->pllkTune);
	

	REFOTRIM=(pllkUpdate<<23);
	REFOCONSET=0x00000200;

	return(1);
}


INT I2SSetSampleRate(I2SState* pCodecHandle, I2S_SAMPLERATE sampleRate)
{
	I2SStartAudio(pCodecHandle, FALSE);
	pllkUpdate=0;
	data_available_count=0;		
	
	data_index_tx=0;
	data_index_value_tx=0;
		
	switch(sampleRate){
		case SAMPLERATE_32000HZ:
			pCodecHandle->samplingFreq=32000;
		   	pCodecHandle->frameSize=32;
   			pllkUpdate=440;
			REFOCONbits.OE = 0;
			REFOCONbits.ON = 0;
			REFOCONbits.RODIV = 5;
			REFOTRIM=(pllkUpdate<<23);		
			REFOCONSET=0x00000200;	
			REFOCONbits.OE = 1;
			REFOCONbits.ON = 1;
   			pCodecHandle->pllkTune=1;
   			pCodecHandle->pllkTuneLimit=8;
   			pCodecHandle->pllkValue=pllkUpdate;
  			
 			break;
 						
		case SAMPLERATE_48000HZ:
			pCodecHandle->samplingFreq=48000;
			pCodecHandle->frameSize=48;
   			pllkUpdate=464;
			REFOCONbits.OE = 0;
			REFOCONbits.ON = 0;
			REFOCONbits.RODIV = 3;
			REFOTRIM=(pllkUpdate<<23);
			REFOCONSET=0x00000200;	
			REFOCONbits.OE = 1;
			REFOCONbits.ON = 1;
   			pCodecHandle->pllkTune=1;
   			pCodecHandle->pllkTuneLimit=4;
   			pCodecHandle->pllkValue=pllkUpdate;
					
			break;
			
		case SAMPLERATE_96000HZ:
			pCodecHandle->samplingFreq=96000;
			pCodecHandle->frameSize=96;
   			pllkUpdate=488;
			REFOCONbits.OE = 0;
			REFOCONbits.ON = 0;
			REFOCONbits.RODIV = 1;
			REFOTRIM=(pllkUpdate<<23);
			REFOCONSET=0x00000200;	
			REFOCONbits.OE = 1;
			REFOCONbits.ON = 1;
   			pCodecHandle->pllkTune=1;
   			pCodecHandle->pllkTuneLimit=4;
   			pCodecHandle->pllkValue=pllkUpdate;
					
			break;
			
		default: 
			break;
	}

	pCodecHandle->bufferSize=pCodecHandle->frameSize*BUFFER_DEPTH;
	pCodecHandle->underrunCount=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)-pCodecHandle->frameSize/4;
	pCodecHandle->overrunCount=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)+pCodecHandle->frameSize/4;
	pCodecHandle->underrunLimit=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)-pCodecHandle->frameSize/2;
	pCodecHandle->overrunLimit=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)+pCodecHandle->frameSize/2;	
	
	I2SStartAudio(pCodecHandle, TRUE);

	return(1);

}

void __attribute__((interrupt(), nomips16))_DmaInterruptHandlerTx(void)
{

	AudioStereo * dest = pCodecHandlePriv->txBuffer;

	INT size;
	data_index_tx=data_index_value_tx=0;
	

	//INTClearFlag(I2S_SPI_TX_DMA_INTERRUPT);
	IFS1bits.DMA0IF=0;
	
	DmaChnClrEvFlags(I2S_SPI_TX_DMA_CHANNEL, DMA_EV_BLOCK_DONE);

	PINGPONG_BUFFN ppFlag = pCodecHandlePriv->activeTxBuffer;
	pCodecHandlePriv->statusTxBuffer[ppFlag] = FALSE;
	
	ppFlag = (ppFlag == PP_BUFF0) ? PP_BUFF1 : PP_BUFF0;
	pCodecHandlePriv->statusTxBuffer[ppFlag] = TRUE;
	dest = (ppFlag == PP_BUFF0) ? dest : &dest[DMA_PP_BUFFER_SIZE];
	size = pCodecHandlePriv->countTxBuffer[ppFlag] * sizeof(AudioStereo);

	DmaChnSetTxfer(	I2S_SPI_TX_DMA_CHANNEL,
					(void*)dest,
					(void *)(&I2S_SPI_MODULE_BUFFER),
					size, 
					sizeof(UINT16),
					sizeof(UINT16)	);

	pCodecHandlePriv->activeTxBuffer = ppFlag;
	DmaChnEnable(I2S_SPI_TX_DMA_CHANNEL);
	
#ifdef USE_DEBUG_LED
//	PORTToggleBits(IOPORT_B, BIT_0);
#endif
}
