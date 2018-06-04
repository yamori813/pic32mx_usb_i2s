/********************************************************************
 FileName:	ak4645a.c
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

#include "AK4645A.h"

INT AK4645AControl(AK4645AState* pCodecHandle, AK4645A_REGISTER controlRegister, INT command);

volatile INT 	data_index_tx=0;
volatile INT 	data_index_value_tx=0;
volatile INT 	data_index_rx=0;
volatile INT 	data_index_value_rx=0;
volatile INT	pllkUpdate=0;
volatile INT 	data_available_count=0;
volatile INT 	data_available_count_rx=0;

AK4645AState* pCodecHandlePriv = NULL;

AK4645AState*
AK4645AOpen(OPEN_MODE mode)
{
	UINT spiFlags=0;
	
	pCodecHandlePriv = (AK4645AState*)malloc(sizeof(AK4645AState));
	if (pCodecHandlePriv == NULL)
		return(NULL);

	if ((mode == O_RD) || (mode == O_RDWR)){	
	
		pCodecHandlePriv->rxBuffer = 
			(AudioStereo*)malloc(AK4645A_RX_BUFFER_SIZE_BYTES);
		if (pCodecHandlePriv->rxBuffer == NULL)
			return(NULL);
		memset(pCodecHandlePriv->rxBuffer, 0,
			AK4645A_RX_BUFFER_SIZE_BYTES);
		
		DmaChnConfigure(AK4645A_SPI_RX_DMA_CHANNEL, DMA_CHN_PRI0,
			DMA_CONFIG_DEFAULT);		
		DmaChnSetEventControl(	AK4645A_SPI_RX_DMA_CHANNEL, 
			DMA_EV_START_IRQ_EN |
			DMA_EV_START_IRQ(AK4645A_SPI_MODULE_RX_IRQ));	

		DmaChnSetTxfer(	AK4645A_SPI_RX_DMA_CHANNEL,
			(void*)&AK4645A_SPI_MODULE_BUFFER,
			(void*)pCodecHandlePriv->rxBuffer,
			sizeof(UINT16),
			AK4645A_RX_BUFFER_SIZE_BYTES>>1, 
			sizeof(UINT16));		
		DmaChnSetEvEnableFlags(AK4645A_SPI_RX_DMA_CHANNEL,
			DMA_EV_BLOCK_DONE);
		
		//INTSetVectorPriority(AK4645A_SPI_RX_DMA_VECTOR, AK4645A_SPI_RX_DMA_INT_PRI_LEVEL);
		//INTSetSubPriority(AK4645A_SPI_RX_DMA_VECTOR, AK4645A_SPI_RX_DMA_INT_SPRI_LEVEL);
		IPC10bits.DMA1IP=AK4645A_SPI_RX_DMA_INT_PRI_LEVEL;
		IPC10bits.DMA1IS=AK4645A_SPI_RX_DMA_INT_SPRI_LEVEL;
	
		INTClearFlag(AK4645A_SPI_RX_DMA_INTERRUPT);		
	}
		
	if ((mode == O_WR) || (mode == O_RDWR)){	
	
		pCodecHandlePriv->txBuffer =
			(AudioStereo*)malloc(AK4645A_TX_BUFFER_SIZE_BYTES);
		if (pCodecHandlePriv->txBuffer == NULL)
			return(NULL);
		memset(pCodecHandlePriv->txBuffer, 0,
			AK4645A_TX_BUFFER_SIZE_BYTES);
		
		DmaChnConfigure(AK4645A_SPI_TX_DMA_CHANNEL,
			DMA_CHN_PRI0, DMA_CONFIG_DEFAULT);		
		DmaChnSetEventControl(AK4645A_SPI_TX_DMA_CHANNEL, 
			DMA_EV_START_IRQ_EN |
			DMA_EV_START_IRQ(AK4645A_SPI_MODULE_TX_IRQ));
													
		DmaChnSetTxfer(	AK4645A_SPI_TX_DMA_CHANNEL,
			(void*)pCodecHandlePriv->txBuffer,
			(void*)&AK4645A_SPI_MODULE_BUFFER,
			AK4645A_TX_BUFFER_SIZE_BYTES>>1, 
			sizeof(UINT16),
			sizeof(UINT16)	);	
	
		DmaChnSetEvEnableFlags(AK4645A_SPI_TX_DMA_CHANNEL,
			DMA_EV_BLOCK_DONE);
		
		//INTSetVectorPriority(AK4645A_SPI_TX_DMA_VECTOR, AK4645A_SPI_TX_DMA_INT_PRI_LEVEL);
		//INTSetSubPriority(AK4645A_SPI_TX_DMA_VECTOR, AK4645A_SPI_TX_DMA_INT_SPRI_LEVEL);
		IPC10bits.DMA0IP=AK4645A_SPI_TX_DMA_INT_PRI_LEVEL;
		IPC10bits.DMA0IS=AK4645A_SPI_TX_DMA_INT_SPRI_LEVEL;
	
		INTClearFlag(AK4645A_SPI_TX_DMA_INTERRUPT);		
							
	}

	/*
	I2CConfigure(AK4645A_I2C_MODULE,0);
	I2CSetFrequency(AK4645A_I2C_MODULE, GetPeripheralClock(),
		AK4645A_I2C_BAUD);

	SPI1CON2bits.AUDEN=1;
	SPI1CON2bits.AUDMOD=0;
	SPI1CON2bits.IGNROV=1;
	SPI1CON2bits.IGNTUR=1;
	SPI1CON2bits.SPIROVEN=0;
	SPI1CON2bits.SPITUREN=0;
	SPI1CON2bits.FRMERREN=0;
	
	spiFlags =  SPI_OPEN_SLVEN |	// Slave mode enable 
		SPI_OPEN_SSEN |		// Enable slave select function
		SPI_OPEN_CKP_HIGH |	// Clock polarity Idle High Actie Low
		SPI_OPEN_FSP_CLK1 |
		SPI_OPEN_FSP_IN |	// Frame Sync Pulse is input
		SPI_OPEN_FSP_HIGH;	// Frame Sync Pulse is active high
	
	SpiChnConfigure(AK4645A_SPI_MODULE, spiFlags);	
	SPI1CON = 0x00000060;
	SPI1CONbits.STXISEL = 0b11;
	SPI1CONbits.DISSDI = 1;
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
*/

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

void AK4645AStartAudio(AK4645AState *pCodecHandle, BOOL enable)
{

	if (enable == TRUE) {

		pllkUpdate=0;
		data_available_count=0;		
		
		data_index_tx=0;
		data_index_value_tx=0;
		data_index_rx=0;
		data_index_value_rx=0;		
		
		pCodecHandle->activeTxBuffer 	= PP_BUFF0;
		pCodecHandle->statusTxBuffer[0] = TRUE;
		pCodecHandle->statusTxBuffer[1] = FALSE;
		pCodecHandle->countTxBuffer[0] 	= DMA_PP_BUFFER_SIZE;
		pCodecHandle->countTxBuffer[1] 	= DMA_PP_BUFFER_SIZE;
		DmaChnClrEvFlags(AK4645A_SPI_TX_DMA_CHANNEL, DMA_EV_BLOCK_DONE);
		//INTEnable( AK4645A_SPI_TX_DMA_INTERRUPT, INT_ENABLED);
		IEC1bits.DMA0IE=1;

#if 0
		pCodecHandle->activeRxBuffer 	= PP_BUFF0;
		pCodecHandle->statusRxBuffer[0] = TRUE;
		pCodecHandle->statusRxBuffer[1] = FALSE;
		pCodecHandle->countRxBuffer[0] 	= DMA_PP_BUFFER_SIZE;
		pCodecHandle->countRxBuffer[1] 	= DMA_PP_BUFFER_SIZE;
		DmaChnClrEvFlags(AK4645A_SPI_RX_DMA_CHANNEL, DMA_EV_BLOCK_DONE);
		//INTEnable( AK4645A_SPI_RX_DMA_INTERRUPT, INT_ENABLED);
		IEC1bits.DMA1IE=1;
#endif

		SpiChnEnable(AK4645A_SPI_MODULE, TRUE);
		DmaChnEnable(AK4645A_SPI_TX_DMA_CHANNEL);
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

#if 0
		DmaChnEnable(AK4645A_SPI_RX_DMA_CHANNEL);
		while(!DCH1CONbits.CHEN)
			DCH1CONbits.CHEN=1; 
		DmaWriteGlobalFlags(DMA_GFLG_ON);
#endif
	} else if (enable == FALSE) {
		DmaClrGlobalFlags(DMA_GFLG_ON);
		DmaChnDisable(AK4645A_SPI_TX_DMA_CHANNEL);
		DmaChnDisable(AK4645A_SPI_RX_DMA_CHANNEL);
		SpiChnEnable(AK4645A_SPI_MODULE, FALSE);
	}
}


INT AK4645AControl(AK4645AState* pCodecHandle, AK4645A_REGISTER controlRegister, INT command)
{

	BYTE byte1  = (BYTE) (controlRegister & 0xFF);
	BYTE byte2  = (BYTE) (command & 0xFF);


	I2CEnable(AK4645A_I2C_MODULE, TRUE);
	
	while(I2CBusIsIdle(AK4645A_I2C_MODULE) == FALSE);

	if (I2CStart(AK4645A_I2C_MODULE) != I2C_SUCCESS)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(AK4645A_I2C_MODULE, FALSE); 
		return(-1);
	}

	while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_START));  

	if (I2CSendByte(AK4645A_I2C_MODULE,AK4645A_I2C_ADDRESS) != I2C_SUCCESS)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP));  
		I2CEnable(AK4645A_I2C_MODULE, FALSE);
		return(-1);
	}

	while(!I2CTransmissionHasCompleted(AK4645A_I2C_MODULE));

	if (I2CByteWasAcknowledged(AK4645A_I2C_MODULE) == FALSE)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(AK4645A_I2C_MODULE, FALSE); 
		return(-1);
	}

	if (I2CSendByte(AK4645A_I2C_MODULE,byte1) != I2C_SUCCESS)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(AK4645A_I2C_MODULE, FALSE); 
		return(-1);
	}

	while(!I2CTransmissionHasCompleted(AK4645A_I2C_MODULE));

	if (I2CByteWasAcknowledged(AK4645A_I2C_MODULE) == FALSE)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP)); 
		I2CEnable(AK4645A_I2C_MODULE, FALSE); 
		return(-1);
	}

	if (I2CSendByte(AK4645A_I2C_MODULE,byte2) != I2C_SUCCESS)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP));  
		I2CEnable(AK4645A_I2C_MODULE, FALSE);
		return(-1);
	}

	while(!I2CTransmissionHasCompleted(AK4645A_I2C_MODULE));

	if (I2CByteWasAcknowledged(AK4645A_I2C_MODULE) == FALSE)
	{
		I2CStop(AK4645A_I2C_MODULE);
		while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP));  
		I2CEnable(AK4645A_I2C_MODULE, FALSE);
		return(-1);
	}

	I2CStop(AK4645A_I2C_MODULE);
	while(!(I2CGetStatus(AK4645A_I2C_MODULE) & I2C_STOP));  

	I2CEnable(AK4645A_I2C_MODULE, FALSE);
	
	return(1);	

}

UINT AK4645AWritePPBuffer(AK4645AState* pCodecHandle, AudioStereo* data, UINT nStereoSamples)
{
	UINT i, nWrittenSamples = 0;
	PINGPONG_BUFFN usePPBuffer;
	AudioStereo* dest;

	if (nStereoSamples == 0) return(0);
	
	if (data_index_value_tx<pCodecHandle->bufferSize)
		data_index_value_tx += nStereoSamples;
	else
		data_index_value_tx = 0;
	
	usePPBuffer = (pCodecHandle->activeTxBuffer == PP_BUFF0) ? PP_BUFF1 : PP_BUFF0;
	if (pCodecHandle->statusTxBuffer[usePPBuffer] == TRUE)
	return(0);
	nWrittenSamples = (nStereoSamples > DMA_PP_BUFFER_SIZE) ? DMA_PP_BUFFER_SIZE : nStereoSamples;
	dest = (usePPBuffer == PP_BUFF0) ? 	&pCodecHandle->txBuffer[0]
									 :	&pCodecHandle->txBuffer[DMA_PP_BUFFER_SIZE];

	for(i = 0; i < nWrittenSamples; i++){	
	dest[data_index_tx].audioWord = data[i].audioWord;
		data_index_tx++;
	}

	
	if (data_index_tx >= pCodecHandle->bufferSize){
		data_index_value_tx=data_index_tx;
		data_index_tx=0;
		pCodecHandle->countTxBuffer[usePPBuffer] = pCodecHandle->bufferSize;	  
		pCodecHandle->statusTxBuffer[usePPBuffer] = TRUE;
	}

	return(nWrittenSamples);	
}

UINT AK4645AWrite(AK4645AState* pCodecHandle, AudioStereo* data, UINT nStereoSamples) 
{
	UINT writtenSamples = 0;
	writtenSamples = AK4645AWritePPBuffer(pCodecHandle, &data[0], nStereoSamples);
	return(writtenSamples);
}


UINT AK4645AReadPPBuffer(AK4645AState* pCodecHandle, AudioStereo* data, UINT nStereoSamples)
{
	UINT i, nReadSamples = 0;
	PINGPONG_BUFFN usePPBuffer;
	AudioStereo* src;

	if (nStereoSamples == 0) return(0);

	if (data_index_value_rx>pCodecHandle->frameSize)
		data_index_value_rx -= nStereoSamples;
	else
		data_index_value_rx = 0;
			
	usePPBuffer = (pCodecHandle->activeRxBuffer == PP_BUFF0) ? PP_BUFF1 : PP_BUFF0;
	if (pCodecHandle->statusRxBuffer[usePPBuffer] == TRUE)
	return(0);
	nReadSamples = (nStereoSamples > DMA_PP_BUFFER_SIZE) ? DMA_PP_BUFFER_SIZE : nStereoSamples;
	src = (usePPBuffer == PP_BUFF0) ? 	&pCodecHandle->rxBuffer[0]
									 :	&pCodecHandle->rxBuffer[DMA_PP_BUFFER_SIZE];

	for(i = 0; i < nReadSamples; i++){	
	data[i].audioWord = src[pCodecHandle->bufferSize-data_index_rx].audioWord;
		data_index_rx--;
	}

	
	if (data_index_rx <= 0){
		data_index_value_rx = data_index_rx;
		data_index_rx = pCodecHandle->bufferSize>>1;
		pCodecHandle->countRxBuffer[usePPBuffer] = pCodecHandle->bufferSize;	  
		pCodecHandle->statusRxBuffer[usePPBuffer] = TRUE;
	}

	return(nReadSamples);	
}

UINT AK4645ARead(AK4645AState* pCodecHandle, AudioStereo* data, UINT nStereoSamples) 
{
	UINT readSamples = 0;
	readSamples = AK4645AReadPPBuffer(pCodecHandle, &data[0], nStereoSamples);
	return(readSamples);
}



INT AK4645ASetADCDACOptions(AK4645AState* pCodecHandle, BOOL enable)
{

	if (AK4645AControl(pCodecHandle, AK4645A_REG_PWR_MGMT1, PWRMGMT1_PMADL_UP | PWRMGMT1_PMDAC_UP | PWRMGMT1_PMLO_UP | PWRMGMT1_PMMIN_UP | PWRMGMT1_PMVCM_UP ) < 0)
		return(-1);
		
	if (AK4645AControl(pCodecHandle, AK4645A_REG_PWR_MGMT2, PWRMGMT2_MS_MASTER | PWRMGMT2_PMHPR_UP | PWRMGMT2_PMHPL_UP | PWRMGMT2_HPMTN_NORMAL /*| PWRMGMT2_HPZ_200K*/) < 0)
		return(-1);
	
	if (AK4645AControl(pCodecHandle, AK4645A_REG_SIG_SLCT1, SIGSLCT1_PMMP_UP | SIGSLCT1_DACL_ON) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_SIG_SLCT2, SIGSLCT2_LOVL_2DB8) < 0)
		return(-1);		

	if (AK4645AControl(pCodecHandle, AK4645A_REG_MODE_CTRL1, MODECTRL1_DIF_I2S) < 0)
		return(-1);	

	if (AK4645AControl(pCodecHandle, AK4645A_REG_MODE_CTRL2, MODECTRL2_FS_MCK256FS) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_LDAC_VOL, LDACVOL_DVL(0x60)) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_RDAC_VOL, RDACVOL_DVL(0x60)) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_ALC_CTRL1, ALCCTRL1_ALC_ENABLE) < 0)
		return(-1);
	
	if (AK4645AControl(pCodecHandle, AK4645A_REG_MODE_CTRL3, MODECTRL3_DEM_OFF | MODECTRL3_BST_OFF) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_MODE_CTRL4, MODECTRL4_DAC_ON) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_PWR_MGMT3, PWRMGMT3_PMADR_UP | PWRMGMT3_MDIF1_DIFFIN | PWRMGMT3_MDIF2_DIFFIN) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_PWR_MGMT4, PWRMGMT4_PMMICL_UP | PWRMGMT4_PMMICR_UP) < 0)
		return(-1);

	return(1);
}

INT AK4645ASetInput(AK4645AState* pCodecHandle, BOOL enable)
{
	if (enable){
		if (AK4645AControl(pCodecHandle, AK4645A_REG_PWR_MGMT3, PWRMGMT3_PMADR_UP | PWRMGMT3_MDIF1_DIFFIN | PWRMGMT3_MDIF2_DIFFIN/*PWRMGMT3_HPG_3P6DB*/) < 0)
			return(-1);
 	}   	
	else{	
		if (AK4645AControl(pCodecHandle, AK4645A_REG_PWR_MGMT3, PWRMGMT3_PMADR_UP | PWRMGMT3_INL0_SET | PWRMGMT3_INR0_SET | PWRMGMT3_INL1_SET | PWRMGMT3_INR1_SET) < 0)
			return(-1);		
	}
}

INT AK4645ABufferClear(AK4645AState* pCodecHandle){
	if (pCodecHandlePriv->rxBuffer != NULL)	
		memset(pCodecHandlePriv->rxBuffer, 0, AK4645A_RX_BUFFER_SIZE_BYTES);
	if (pCodecHandlePriv->txBuffer != NULL)		
		memset(pCodecHandlePriv->txBuffer, 0, AK4645A_TX_BUFFER_SIZE_BYTES);

	return(1);
}

INT AK4645ASetDACVolume(AK4645AState* pCodecHandle, BYTE volumeDAC)
{
	INT volCtrl = 0x15 + ((UINT32)((100-volumeDAC)*(0x80-0x15)))/100;
	
	if (volumeDAC<3)
		volCtrl = 0xF1;
		
	if (AK4645AControl(pCodecHandle, AK4645A_REG_LDAC_VOL, LDACVOL_DVL(volCtrl)) < 0)
		return(-1);

	if (AK4645AControl(pCodecHandle, AK4645A_REG_RDAC_VOL, RDACVOL_DVL(volCtrl)) < 0)
		return(-1);
			
								
	return(1);
}

INT AK4645ADACMute(AK4645AState* pCodecHandle, BOOL enable){
	if (AK4645AControl(pCodecHandle, AK4645A_REG_MODE_CTRL3, (enable ? MODECTRL3_SMUTE_ENABLE : 0)) < 0)	
		return(-1);
		
	return(1);
}

void AK4645AAdjustSampleRateRx(AK4645AState* pCodecHandle)
{

	data_available_count_rx=(volatile INT)(DmaChnGetSrcPnt(AK4645A_SPI_RX_DMA_CHANNEL)>>2)-data_index_value_rx;

	if (data_available_count_rx>pCodecHandle->underrunLimit && data_available_count<pCodecHandle->overrunLimit){
			
		if (data_available_count_rx<pCodecHandle->underrunCount)
			AK4645ATuneSampleRate(pCodecHandle, DEC_TUNE);
		else if (data_available_count_rx>pCodecHandle->overrunCount)
			AK4645ATuneSampleRate(pCodecHandle, INC_TUNE);
	}
		
}

void AK4645AAdjustSampleRateTx(AK4645AState* pCodecHandle)
{
	
	data_available_count=(pCodecHandle->bufferSize-(volatile INT)(DmaChnGetSrcPnt(AK4645A_SPI_TX_DMA_CHANNEL)>>2))+data_index_value_tx;

	if (data_available_count>pCodecHandle->underrunLimit && data_available_count<pCodecHandle->overrunLimit){
			
		if (data_available_count<pCodecHandle->underrunCount)
			AK4645ATuneSampleRate(pCodecHandle, DEC_TUNE);
		else if (data_available_count>pCodecHandle->overrunCount)
			AK4645ATuneSampleRate(pCodecHandle, INC_TUNE);
	}
		
}


INT AK4645ATuneSampleRate(AK4645AState* pCodecHandle, TUNE_STEP tune_step)
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


INT AK4645ASetSampleRate(AK4645AState* pCodecHandle, AK4645A_SAMPLERATE sampleRate)
{
	AK4645AStartAudio(pCodecHandle, FALSE);
	pllkUpdate=0;
	data_available_count=0;		
	
	data_index_tx=0;
	data_index_value_tx=0;
	data_index_rx=0;
	data_index_value_rx=0;	
		
		
	switch(sampleRate){
		case SAMPLERATE_32000HZ:

			pCodecHandle->samplingFreq=32000;
		   	pCodecHandle->frameSize=32;
		pCodecHandle->bufferSize=pCodecHandle->frameSize*BUFFER_DEPTH;
		pCodecHandle->bufferSize=pCodecHandle->frameSize*BUFFER_DEPTH;
			pCodecHandle->underrunCount=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)-pCodecHandle->frameSize/4;
			pCodecHandle->overrunCount=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)+pCodecHandle->frameSize/4;
			pCodecHandle->underrunLimit=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)-pCodecHandle->frameSize/2;
			pCodecHandle->overrunLimit=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)+pCodecHandle->frameSize/2;	
 
			REFOCONbits.OE = 0;
			REFOCONbits.ON = 0;
			REFOCONbits.RODIV = 5;
			REFOTRIM=(440<<23);		
			REFOCONSET=0x00000200;	
			REFOCONbits.OE = 1;
			REFOCONbits.ON = 1;
   			pCodecHandle->pllkTune=1;
   			pCodecHandle->pllkTuneLimit=8;
   			pCodecHandle->pllkValue=440;
   			pllkUpdate=440;
  			
 			break;
 						
		case SAMPLERATE_48000HZ:
			pCodecHandle->samplingFreq=48000;
			pCodecHandle->frameSize=48;
		pCodecHandle->bufferSize=pCodecHandle->frameSize*BUFFER_DEPTH;
			pCodecHandle->underrunCount=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)-pCodecHandle->frameSize/4;
			pCodecHandle->overrunCount=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)+pCodecHandle->frameSize/4;
			pCodecHandle->underrunLimit=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)-pCodecHandle->frameSize/2;
			pCodecHandle->overrunLimit=pCodecHandle->bufferSize+((int)pCodecHandle->frameSize>>1)+pCodecHandle->frameSize/2;	
 
			REFOCONbits.OE = 0;
			REFOCONbits.ON = 0;
			REFOCONbits.RODIV = 3;
			REFOTRIM=0xE8000000;
			REFOCONSET=0x00000200;	
			REFOCONbits.OE = 1;
			REFOCONbits.ON = 1;
   			pCodecHandle->pllkTune=1;
   			pCodecHandle->pllkTuneLimit=4;
   			pCodecHandle->pllkValue=0x1D0;
   			pllkUpdate=0x1D0;
					
			break;
			
		default: 
			break;
	}
	

//	AK4645AStartAudio(pCodecHandle, TRUE);
	
	return(1);

}

void __attribute__((interrupt(), nomips16))_DmaInterruptHandlerTx(void)
{

	AudioStereo * dest = pCodecHandlePriv->txBuffer;

	INT size;
	data_index_tx=data_index_value_tx=0;
	

	//INTClearFlag(AK4645A_SPI_TX_DMA_INTERRUPT);
	IFS1bits.DMA0IF=0;
	
	DmaChnClrEvFlags(AK4645A_SPI_TX_DMA_CHANNEL, DMA_EV_BLOCK_DONE);

	PINGPONG_BUFFN ppFlag = pCodecHandlePriv->activeTxBuffer;
	pCodecHandlePriv->statusTxBuffer[ppFlag] = FALSE;
	
	ppFlag = (ppFlag == PP_BUFF0) ? PP_BUFF1 : PP_BUFF0;
	pCodecHandlePriv->statusTxBuffer[ppFlag] = TRUE;
	dest = (ppFlag == PP_BUFF0) ? dest : &dest[DMA_PP_BUFFER_SIZE];
	size = pCodecHandlePriv->countTxBuffer[ppFlag] * sizeof(AudioStereo);

	DmaChnSetTxfer(	AK4645A_SPI_TX_DMA_CHANNEL,
					(void*)dest,
					(void *)(&AK4645A_SPI_MODULE_BUFFER),
					size, 
					sizeof(UINT16),
					sizeof(UINT16)	);

	pCodecHandlePriv->activeTxBuffer = ppFlag;
	DmaChnEnable(AK4645A_SPI_TX_DMA_CHANNEL);
	
#ifdef USE_DEBUG_LED
//	PORTToggleBits(IOPORT_B, BIT_0);
//	mLED_1_Toggle();
#endif
}

void __attribute__((interrupt, nomips16))_DmaInterruptHandlerRx(void)
{

	AudioStereo * src = pCodecHandlePriv->rxBuffer;

	INT size;
	data_index_rx=data_index_value_rx=pCodecHandlePriv->bufferSize;
	//INTClearFlag(AK4645A_SPI_RX_DMA_INTERRUPT);
	IFS1bits.DMA1IF=0;
	
	DmaChnClrEvFlags(AK4645A_SPI_RX_DMA_CHANNEL, DMA_EV_BLOCK_DONE);

	PINGPONG_BUFFN ppFlag = pCodecHandlePriv->activeRxBuffer;
	pCodecHandlePriv->statusRxBuffer[ppFlag] = FALSE;
	
	ppFlag = (ppFlag == PP_BUFF0) ? PP_BUFF1 : PP_BUFF0;
	pCodecHandlePriv->statusRxBuffer[ppFlag] = TRUE;
	src = (ppFlag == PP_BUFF0) ? src : &src[DMA_PP_BUFFER_SIZE];
	size = pCodecHandlePriv->countRxBuffer[ppFlag] * sizeof(AudioStereo);

	DmaChnSetTxfer(	AK4645A_SPI_RX_DMA_CHANNEL,
					(void *)(&AK4645A_SPI_MODULE_BUFFER),
					(void*)src,
					sizeof(UINT16),
					size, 
					sizeof(UINT16)	);

	pCodecHandlePriv->activeRxBuffer = ppFlag;
	DmaChnEnable(AK4645A_SPI_RX_DMA_CHANNEL);

#ifdef USE_DEBUG_LED
//	PORTToggleBits(IOPORT_B, BIT_0);
#endif
}
