This is USB I2S converter on PIX32MX.  

I2S connection  

MCLK - RA4(PPS)  
LRCLK - RB4(PPS)  
SDATA - RA1(PPS)  
BCLK - RB14  

LED configuration  

RB15 - LED1  

Swtich configuration  

RB8 - AUDMOD bit 0  
RB7 - AUDMOD bit 1  

AUDMOD  

11 = PCM/DSP mode  
10 = Right-Justified mode  
01 = Left-Justified mode  
00 = I2S mode  

Currently I2S only support 48KHz 16bit  
