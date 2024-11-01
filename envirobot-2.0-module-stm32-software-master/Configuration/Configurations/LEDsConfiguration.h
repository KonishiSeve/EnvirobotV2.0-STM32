#include "Configurations/CommunicationConfiguration.h"

///////////////////
// Configuration //
///////////////////
#define USE_SMD_LEDs
//#define USE_I2C_LEDs

/////////////////////
// LEDs Parameters //
/////////////////////
#define DEFAULT_LEDS_PERIOD	100 // ms
#define TEST_LEDS

//////////////
// LEDs IDs //
//////////////
// Communication LEDs have the same ID than in CommunicationProtocol.hpp to simplify the program the program
#define LED_CANFD1			CANFD1
#define LED_CANFD2			CANFD2
#define LED_UART_FRONT		UART_FRONT
#define LED_UART_BACK		UART_BACK
#define LED_UART_RS485		UART_RS485
#define LED_STATUS 			7
#define LED_CONTROLLER 		8
#define LED_FAULT 			9
#define LED_USER1 			10
#define LED_USER2 			11
#define LED_USER3 			12
#define LED_RGB_TOP 		13
#define LED_TOP_EDGE 		14

//////////////////////////////////////
// Communication LEDs Configuration //
//////////////////////////////////////
#define USE_COM_LED_REGISTER_ACCESS
#define USE_COM_LED_SUBSCRIBER_INPUT
#define USE_COM_LED_SERVICES_RECEPTION
#define USE_COM_LED_SEND // for register feedback, publisher broadcast, services request and user send
