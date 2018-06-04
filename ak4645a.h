
#include <p32xxxx.h>
#include <GenericTypeDefs.h>
#include <plib.h>

#ifndef GetPeripheralClock
#define GetPeripheralClock() 					40000000
#endif

#define BUFFER_DEPTH						6
#define PINGPONG_DEPTH						(BUFFER_DEPTH<<1)
#define AK4645A_TX_BUFFER_SIZE_STEREO_WORD  (48*PINGPONG_DEPTH)
#define AK4645A_RX_BUFFER_SIZE_STEREO_WORD  (48*PINGPONG_DEPTH)
#define AK4645A_TX_BUFFER_SIZE_BYTES  		(AK4645A_TX_BUFFER_SIZE_STEREO_WORD * sizeof(UINT32))
#define AK4645A_RX_BUFFER_SIZE_BYTES  		(AK4645A_RX_BUFFER_SIZE_STEREO_WORD * sizeof(UINT32))
#define DMA_PP_BUFFER_SIZE 					(AK4645A_TX_BUFFER_SIZE_STEREO_WORD>>1)

#define AK4645A_I2C_ADDRESS 					((0x09<<2) | 0x02)
#define AK4645A_I2C_MODULE    					I2C1
#define AK4645A_I2C_BAUD      					100000

#define AK4645A_SPI_MODULE            			SPI_CHANNEL1
#define AK4645A_SPI_MODULE_BUFFER     	    	SPI1BUF
#define AK4645A_SPI_MODULE_TX_IRQ     	    	_SPI1_TX_IRQ
#define AK4645A_SPI_MODULE_RX_IRQ     	    	_SPI1_RX_IRQ
                                                
#define AK4645A_SPI_TX_DMA_CHANNEL            	DMA_CHANNEL0
#define AK4645A_SPI_TX_DMA_VECTOR             	INT_DMA_0_VECTOR
#define AK4645A_SPI_TX_DMA_INTERRUPT          	INT_DMA0
#define AK4645A_SPI_TX_DMA_INT_PRI_LEVEL      	INT_PRIORITY_LEVEL_5
#define AK4645A_SPI_TX_DMA_INT_SPRI_LEVEL     	INT_SUB_PRIORITY_LEVEL_0
                                                
#define AK4645A_SPI_RX_DMA_CHANNEL    	      	DMA_CHANNEL1
#define AK4645A_SPI_RX_DMA_VECTOR             	INT_DMA_1_VECTOR
#define AK4645A_SPI_RX_DMA_INTERRUPT          	INT_DMA1
#define AK4645A_SPI_RX_DMA_INT_PRI_LEVEL      	INT_PRIORITY_LEVEL_5
#define AK4645A_SPI_RX_DMA_INT_SPRI_LEVEL     	INT_SUB_PRIORITY_LEVEL_1


typedef enum{
    O_RD,   
    O_WR,   
    O_RDWR,      
}OPEN_MODE;

typedef enum{
	PP_BUFF0,
	PP_BUFF1,
	PP_BUFFN
}PINGPONG_BUFFN;

typedef enum{
DEC_TUNE=0,INC_TUNE,
} TUNE_STEP;

typedef union{
    struct{
        INT16 rightChannel;
        INT16 leftChannel;
    };
    UINT32 audioWord;
}AudioStereo;

typedef struct __AK4645A_state{
	AudioStereo		*txBuffer;
	AudioStereo		*rxBuffer;
	volatile PINGPONG_BUFFN	activeRxBuffer;
	volatile BOOL			statusRxBuffer[PP_BUFFN];
	volatile UINT			countRxBuffer[PP_BUFFN];
	volatile PINGPONG_BUFFN	activeTxBuffer;
	volatile BOOL			statusTxBuffer[PP_BUFFN];
	volatile UINT			countTxBuffer[PP_BUFFN];

    volatile UINT/*float*/ 			frameSize;    
    volatile UINT 			bufferSize;
    volatile UINT 			underrunCount;
    volatile UINT 			overrunCount;
    volatile UINT 			underrunLimit;
    volatile UINT 			overrunLimit;
    volatile INT 			pllkValue;
    volatile INT 			pllkTune;
    volatile INT			pllkTuneLimit;
	
	volatile UINT32 		masterClk;
	volatile UINT32		 	samplingFreq;

} AK4645AState;

typedef enum{
								
	AK4645A_REG_PWR_MGMT1=0,	//0		<--
	AK4645A_REG_PWR_MGMT2,		//1
	AK4645A_REG_SIG_SLCT1,		//2
	AK4645A_REG_SIG_SLCT2,		//3
	AK4645A_REG_MODE_CTRL1,		//4
	AK4645A_REG_MODE_CTRL2,		//5
	AK4645A_REG_TMR_SLCT,		//6
	AK4645A_REG_ALC_CTRL1,      //7
	AK4645A_REG_ALC_CTRL2,	    //8
	AK4645A_REG_LINPGAGAIN_CTRL,//9
	AK4645A_REG_LDAC_VOL,       //A
	AK4645A_REG_ALC_CTRL3,      //B
	AK4645A_REG_RINPGAGAIN_CTRL,//C
	AK4645A_REG_RDAC_VOL,       //D
	AK4645A_REG_MODE_CTRL3,		//E
	AK4645A_REG_MODE_CTRL4,		//F
	AK4645A_REG_PWR_MGMT3,      //10	
	AK4645A_REG_DFLTR_SLCT,     //11
	AK4645A_REG_FLTR30_SET,   	//12	
	AK4645A_REG_FLTR31_SET,     //13
	AK4645A_REG_FLTR32_SET,     //14
	AK4645A_REG_FLTR33_SET,     //15	
	AK4645A_REG_EQ0_SET,   		//16	
	AK4645A_REG_EQ1_SET,        //17
	AK4645A_REG_EQ2_SET,        //18
	AK4645A_REG_EQ3_SET,        //19
	AK4645A_REG_EQ4_SET,		//1A
	AK4645A_REG_EQ5_SET,		//1B
	AK4645A_REG_FLTR10_SET,   	//1C	
	AK4645A_REG_FLTR11_SET,     //1D
	AK4645A_REG_FLTR12_SET,     //1E
	AK4645A_REG_FLTR13_SET,     //1F
	AK4645A_REG_PWR_MGMT4,      //20
	AK4645A_REG_MODE_CTRL5,		//21
	AK4645A_REG_OUTMIXER_CTRL,  //22
	AK4645A_REG_HPMIXER_CTRL,   //23
	AK4645A_REG_RESERVED,  		//24
	
}AK4645A_REGISTER;

typedef enum{
    SAMPLERATE_48000HZ,
    SAMPLERATE_44100HZ,
    SAMPLERATE_32000HZ,
    SAMPLERATE_24000HZ,
    SAMPLERATE_16000HZ,
    SAMPLERATE_8000HZ,
    SAMPLERATE_PERSIST    
}AK4645A_SAMPLERATE;


/* Register bit fields. */
#define PWRMGMT1_PMADL_UP    	(1 << 0)
#define PWRMGMT1_PMDAC_UP	    (1 << 2)
#define PWRMGMT1_PMLO_UP	    (1 << 3)
#define PWRMGMT1_PMMIN_UP	    (1 << 5)
#define PWRMGMT1_PMVCM_UP	    (1 << 6)

#define PWRMGMT2_MS_MASTER	    (1 << 3)
#define PWRMGMT2_PMHPR_UP	    (1 << 4)
#define PWRMGMT2_PMHPL_UP	    (1 << 5)
#define PWRMGMT2_HPMTN_NORMAL   (1 << 6)
#define PWRMGMT2_HPZ_200K	    (1 << 7)

#define SIGSLCT1_MGAIN0_SET		(1 << 0)
#define SIGSLCT1_PMMP_UP		(1 << 2)
#define SIGSLCT1_DACL_ON		(1 << 4)

#define SIGSLCT2_MINL_ON		(1 << 2)
#define SIGSLCT2_MGAIN1_SET		(1 << 5)
#define SIGSLCT2_LOPS_PWRSAV	(1 << 6)
#define SIGSLCT2_LOVL_2DB8		(1 << 7)

#define MODECTRL1_DIF_DSP		(0 << 0)
#define MODECTRL1_DIF_LJ		(1 << 0)
#define MODECTRL1_DIF_RJ		(2 << 0)
#define MODECTRL1_DIF_I2S		(3 << 0)
#define MODECTRL1_BCKO_64FS		(1 << 3)

#define MODECTRL2_FS_MCK256FS	(0 << 0)
#define MODECTRL2_FS_MCK1024FS	(1 << 0)
#define MODECTRL2_FS_MCK384FS	(2 << 0)
#define MODECTRL2_FS_MCK512FS	(3 << 0)
#define MODECTRL2_BCKP_MODE2	(1 << 3)
#define MODECTRL2_MSBS_MODE2	(1 << 4)

#define TMRSLCT_RFST_4X			(0 << 0)	
#define TMRSLCT_RFST_8X			(1 << 0)
#define TMRSLCT_RFST_16X		(2 << 0)
#define TMRSLCT_WTM_128FS		(0 << 2)
#define TMRSLCT_WTM_256FS		(1 << 2)
#define TMRSLCT_WTM_512FS		(2 << 2)
#define TMRSLCT_WTM_1024FS		(3 << 2)
#define TMRSLCT_WTM_2048FS		(16 << 2)
#define TMRSLCT_WTM_4096FS		(17 << 2)
#define TMRSLCT_WTM_8192FS		(18 << 2)
#define TMRSLCT_WTM_16384FS		(19 << 2)
#define TMRSLCT_ZTM_128FS		(0 << 4)
#define TMRSLCT_ZTM_256FS		(1 << 4)
#define TMRSLCT_ZTM_512FS		(2 << 4)
#define TMRSLCT_ZTM_1024FS		(3 << 4)
#define TMRSLCT_DVTM_256FS		(1 << 7)

#define ALCCTRL1_LMTH0_SET		(1 << 0)
#define ALCCTRL1_RGAIN0_SET		(1 << 1)
#define ALCCTRL1_LMAT_0P375		(0 << 2)
#define ALCCTRL1_LMAT_0P750		(1 << 2)
#define ALCCTRL1_LMAT_1P125		(2 << 2)
#define ALCCTRL1_LMAT_1P500		(3 << 2)
#define ALCCTRL1_ZELMN_DISABLE	(1 << 4)
#define ALCCTRL1_ALC_ENABLE		(1 << 5)

#define ALCCTRL2_REF(n)			(n)

#define LINPGAGAIN_IVL(n)		(n)
#define RINPGAGAIN_IVL(n)		(n)

#define LDACVOL_DVL(n)			(n)
#define RDACVOL_DVL(n)			(n)

#define ALCCTRL3_VBAT_0P64AVDD	(1 << 1)
#define ALCCTRL3_LMTH1_SET		(1 << 6)
#define ALCCTRL3_RGAIN1_SET		(1 << 7)

#define MODECTRL3_DEM_44K1HZ	(0 << 0)
#define MODECTRL3_DEM_OFF		(1 << 0)
#define MODECTRL3_DEM_48KHZ		(2 << 0)
#define MODECTRL3_DEM_32KHZ		(3 << 0)
#define MODECTRL3_BST_OFF		(0 << 2)
#define MODECTRL3_BST_MIN		(1 << 2)
#define MODECTRL3_BST_MID		(2 << 2)
#define MODECTRL3_BST_MAX		(3 << 2)
#define MODECTRL3_DVOLC_DEP		(1 << 4)
#define MODECTRL3_SMUTE_ENABLE	(1 << 5)
#define MODECTRL3_LOOP_ENABLE	(1 << 6)

#define MODECTRL4_DAC_ON		(1 << 0)
#define MODECTRL4_MINH_ON		(1 << 1)
#define MODECTRL4_HPM_MONO		(1 << 2)
#define MODECTRL4_IVOLC_DEP		(1 << 3)

#define PWRMGMT3_PMADR_UP	    (1 << 0)
#define PWRMGMT3_INL0_SET	    (1 << 1)
#define PWRMGMT3_INR0_SET	    (1 << 2)
#define PWRMGMT3_MDIF1_DIFFIN   (1 << 3)
#define PWRMGMT3_MDIF2_DIFFIN   (1 << 4)
#define PWRMGMT3_HPG_3P6DB	    (1 << 5)
#define PWRMGMT3_INL1_SET	    (1 << 6)
#define PWRMGMT3_INR1_SET	    (1 << 7)

#define DFLTRSLCT_FIL3_ENABLE	(1 << 2)
#define DFLTRSLCT_EQ_ENABLE		(1 << 3)
#define DFLTRSLCT_FIL1_ENABLE	(1 << 4)
#define DFLTRSLCT_GN_0DB		(0 << 6)
#define DFLTRSLCT_GN_12DB		(1 << 6)
#define DFLTRSLCT_GN_24DB		(2 << 6)


#define FLTR30SET_F3A(nA)		(0xFF & nA)

#define FLTR31SET_F3A(nA)		((0x3F00 & nA) >> 8)
#define FLTR31SET_F3AS_LPF		(1 << 7)

#define FLTR32SET_F3B(nB)		(0xFF & nB)

#define FLTR33SET_F3B(nB)		((0x3F00 & nB) >> 8)


#define EQ0SET_EQA(nA)			(0xFF & nA)	

#define EQ1SET_EQA(nA)			((0xFF00 & nA) >> 8)

#define EQ2SET_EQB(nB)			(0xFF & nB)

#define EQ3SET_EQB(nB)			((0x3F00 & nB) >> 8)

#define EQ4SET_EQC(nC)			(0xFF & nC)	

#define EQ5SET_EQC(nC)			((0xFF00 & nC) >> 8)


#define FLTR10SET_F1A(nA)		(0xFF & nA)

#define FLTR11SET_F1A(nA)		((0x3F00 & nA) >> 8)
#define FLTR11SET_F1AS_LPF		(1 << 7)

#define FLTR12SET_F1B(nB)		(0xFF & nB)

#define FLTR13SET_F1B(nB)		((0x3F00 & nB) >> 8)


#define PWRMGMT4_PMMICL_UP	    (1 << 0)
#define PWRMGMT4_PMMICR_UP	    (1 << 1)
#define PWRMGMT4_PMAINL2_UP	    (1 << 2)
#define PWRMGMT4_PMAINR2_UP	    (1 << 3)
#define PWRMGMT4_PMAINL3_UP	    (1 << 4)
#define PWRMGMT4_PMAINR3_UP	    (1 << 5)
#define PWRMGMT4_PMAINL4_UP	    (1 << 6)
#define PWRMGMT4_PMAINR4_UP	    (1 << 7)

#define MODECTRL5_LODIF_DIFFOUT	(1 << 0)
#define MODECTRL5_AIN3_STEREOIN	(1 << 1)
#define MODECTRL5_MIX_MONO		(1 << 2)
#define MODECTRL5_L4DIF_DIFFIN	(1 << 3)
#define MODECTRL5_MICL3_MICAMP	(1 << 4)
#define MODECTRL5_MICR3_MICAMP	(1 << 5)

#define OUTMIXERCTRL_LINL2_ON	(1 << 0)
#define OUTMIXERCTRL_RINL2_ON	(1 << 1)
#define OUTMIXERCTRL_LINL3_ON	(1 << 2)
#define OUTMIXERCTRL_RINL3_ON	(1 << 3)
#define OUTMIXERCTRL_LINL4_ON	(1 << 4)
#define OUTMIXERCTRL_RINL4_ON	(1 << 5)
#define OUTMIXERCTRL_LOM3_MONO	(1 << 6)
#define OUTMIXERCTRL_LOM_MONO	(1 << 7)

#define HPMIXERCTRL_LINH2_ON	(1 << 0)		
#define HPMIXERCTRL_RINH2_ON	(1 << 1)
#define HPMIXERCTRL_LINH3_ON	(1 << 2)		
#define HPMIXERCTRL_RINH3_ON	(1 << 3)
#define HPMIXERCTRL_LINH4_ON	(1 << 4)		
#define HPMIXERCTRL_RINH4_ON	(1 << 5)
#define HPMIXERCTRL_HPM3_MONO	(1 << 6)		
