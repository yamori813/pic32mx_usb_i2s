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

#endif  

