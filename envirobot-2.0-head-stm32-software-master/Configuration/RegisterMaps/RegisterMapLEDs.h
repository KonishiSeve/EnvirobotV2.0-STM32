#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_LEDS_SET			0x0300 // uint8_t	array 2		write only	(0=ID, 1=value)
#define REG_LEDS_TIMINGS		0x0301 // uint8_t   vector      write only  (0=ID, ..timing.., last=period)
#define REG_LEDS_BLINK			0x0302 // uint8_t				write only	(0=ID)
#define REG_LEDS_BLINK_ONCE		0x0303 // uint8_t				write only	(0=ID)
#define REG_LEDS_SET_RGB		0x0304 // uint8_t 	array 4		write only 	(0=ID, 1=R, 2=G, 3=B)
#define REG_LEDS_SET_BRIGHNESS	0x0305 // uint8_t 	array 2		write only 	(0=ID, 1=brightness)
