#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    DIG_IO_ENTRY(test_in,     GPIOB, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(led_out,     GPIOE, GPIO2,  PinMode::OUTPUT)      \
	DIG_IO_ENTRY(led_out2,    GPIOA, GPIO5,  PinMode::OUTPUT)      \
	DIG_IO_ENTRY(led_out3,    GPIOC, GPIO13, PinMode::OUTPUT)      \
	DIG_IO_ENTRY(lin_cs,      GPIOA, GPIO8,  PinMode::OUTPUT)      \
	DIG_IO_ENTRY(lin_nslp,    GPIOD, GPIO11, PinMode::OUTPUT)      \

#endif // PinMode_PRJ_H_INCLUDED


    //DIG_IO_ENTRY(led_out,     GPIOE, GPIO2, PinMode::OUTPUT)      \ ZombieV
	//DIG_IO_ENTRY(led_out2,    GPIOA, GPIO5, PinMode::OUTPUT)      \ Nucleo F103
	//DIG_IO_ENTRY(led_out3,    GPIOC, GPIO13, PinMode::OUTPUT)      \ Bluepill
	//DIG_IO_ENTRY(lin_nslp,    GPIOD, GPIO11, PinMode::OUTPUT)      \ ZombieV
	//DIG_IO_ENTRY(lin_cs,      GPIOA, GPIO8,  PinMode::OUTPUT)      \ others