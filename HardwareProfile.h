// HardwareProfile.h

#ifndef _HARDWARE_PROFILE_H_
#define _HARDWARE_PROFILE_H_

	#include "HardwareProfile_MX220F032B.h"

    #define USB_A0_SILICON_WORK_AROUND

    // Clock values
    #define MILLISECONDS_PER_TICK       10                  // -0.000% error
    #define TIMER_PRESCALER             TIMER_PRESCALER_8   // At 60MHz
    #define TIMER_PERIOD                37500               // At 60MHz

#if defined (__PIC32MX__)
    #define BAUDRATE2       57600UL
    #define BRG_DIV2        4 
    #define BRGH2           1 
#endif

    #include <p32xxxx.h>
    #include <plib.h>
    #include <uart1.h>


/** TRIS ***********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0

#define CLOCK_FREQ 40000000

    #ifdef AUDIO_SAMPLING_FREQUENCY_48000
        #define NO_OF_SAMPLES_IN_A_USB_FRAME 48
        #define PWM_PERIOD      (CLOCK_FREQ/48000)-1
    #elif defined AUDIO_SAMPLING_FREQUENCY_32000
        #define NO_OF_SAMPLES_IN_A_USB_FRAME 32
        #define PWM_PERIOD      (CLOCK_FREQ/32000)-1
    #elif defined AUDIO_SAMPLING_FREQUENCY_44100
        #define NO_OF_SAMPLES_IN_A_USB_FRAME 44
        #define PWM_PERIOD      (CLOCK_FREQ/44100)-1
    #endif


#define mInitTimerInterrupt() { INTEnableSystemMultiVectoredInt();INTEnableInterrupts;}
#define mInitTimer()	{OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, PWM_PERIOD);ConfigIntTimer2(T2_INT_OFF | T2_INT_PRIOR_7);}

#endif  

