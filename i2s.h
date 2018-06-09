
#include <p32xxxx.h>
#include <GenericTypeDefs.h>
#include <plib.h>

#ifndef GetPeripheralClock
#define GetPeripheralClock() 		40000000
#endif

//#define BUFFER_DEPTH			6
#define BUFFER_DEPTH			3
#define I2S_TX_BUFFER_SIZE_STEREO_WORD	(49*BUFFER_DEPTH*2)
#ifdef SAMPLE24
#define I2S_TX_BUFFER_SIZE_BYTES  	(I2S_TX_BUFFER_SIZE_STEREO_WORD * sizeof(UINT32) * 2)
#else
#define I2S_TX_BUFFER_SIZE_BYTES  	(I2S_TX_BUFFER_SIZE_STEREO_WORD * sizeof(UINT16) * 2)
#endif
#define DMA_PP_BUFFER_SIZE 		(I2S_TX_BUFFER_SIZE_STEREO_WORD>>1)

#define I2S_I2C_ADDRESS 		((0x09<<2) | 0x02)
#define I2S_I2C_MODULE			I2C1
#define I2S_I2C_BAUD	  		100000

#define I2S_SPI_MODULE			SPI_CHANNEL1
#define I2S_SPI_MODULE_BUFFER	 	SPI1BUF
#define I2S_SPI_MODULE_TX_IRQ	 	_SPI1_TX_IRQ

#define I2S_SPI_TX_DMA_CHANNEL		DMA_CHANNEL0
#define I2S_SPI_TX_DMA_VECTOR		INT_DMA_0_VECTOR
#define I2S_SPI_TX_DMA_INTERRUPT	INT_DMA0
#define I2S_SPI_TX_DMA_INT_PRI_LEVEL	INT_PRIORITY_LEVEL_5
#define I2S_SPI_TX_DMA_INT_SPRI_LEVEL	INT_SUB_PRIORITY_LEVEL_0

typedef enum{
	PP_BUFF0=0,
	PP_BUFF1,
	PP_BUFFN
}PINGPONG_BUFFN;

typedef enum{
	DEC_TUNE=0,
	INC_TUNE,
} TUNE_STEP;

#ifdef SAMPLE24
typedef union{
	struct{
		unsigned char rightChannel[4];
		unsigned char leftChannel[4];
	};
}AudioStereo;
#else
typedef union{
	struct{
		INT16 rightChannel;
		INT16 leftChannel;
	};
	UINT32 audioWord;
}AudioStereo;
#endif

typedef struct __I2S_state{
	AudioStereo			*txBuffer;
	volatile PINGPONG_BUFFN		activeTxBuffer;
	volatile UINT			countTxBuffer[PP_BUFFN];
	volatile UINT			sizeTxBuffer[PP_BUFFN];
	volatile BOOL			runDMA;

	volatile UINT	 		frameSize;	// one Isochronous size
	volatile INT 			pllkValue;
	volatile INT 			pllkTune;
	volatile INT			pllkTuneLimit;
	
	volatile UINT32 		masterClk;
	volatile UINT32		 	samplingFreq;

} I2SState;

typedef enum{
       I2S_REG_RESERVED=0,
}I2S_REGISTER;


typedef enum{
	SAMPLERATE_96000HZ,
	SAMPLERATE_48000HZ,
	SAMPLERATE_44100HZ,
	SAMPLERATE_32000HZ,
	SAMPLERATE_24000HZ,
	SAMPLERATE_16000HZ,
	SAMPLERATE_8000HZ,
	SAMPLERATE_PERSIST	
}I2S_SAMPLERATE;

I2SState* I2SOpen();
void I2SStartAudio(I2SState *pCodecHandle, BOOL enable);
INT I2SSetSampleRate(I2SState* pCodecHandle, I2S_SAMPLERATE sampleRate);
void I2SWrite(I2SState* pCodecHandle, unsigned char* data, UINT nStereoSamples);
void I2SAdjustSampleRateTx(I2SState* pCodecHandle);
INT I2SSetSampleRate(I2SState* pCodecHandle, I2S_SAMPLERATE sampleRate);
INT I2SControl(I2SState* pCodecHandle, I2S_REGISTER controlRegister, INT command);

