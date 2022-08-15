This is USB I2S converter on PIX32MX.  

I2S connection  

MCLK - RA4(PPS)  
LRCLK - RB4(PPS)  
SDATA - RA1(PPS)  
BCLK - RB14  

LED configuration

RB15 - LED1

Swtich configuration

RB5 - AUDMOD bit 0 (Invert)  
RB7 - AUDMOD bit 1 (Invert)  
RB8 - 256fs / 512fs

AUDMOD

11 = PCM/DSP mode  
10 = Right-Justified mode  
01 = Left-Justified mode  
00 = I2S mode  

Currently I2S support 48KHz,32KHz 16bit.

Test on Mac OS X 10.4 and 10.5 by AK4382, FN1242, PCM5201.
