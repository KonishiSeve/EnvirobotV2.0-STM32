#include "Configurations/SensorsConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"
#include "Definitions/CommunicationDefinition.h"

///////////////////
// Publisher IDs //
///////////////////
#define PUBLISHER_VOLTAGES 						VOLTAGES
#define PUBLISHER_MOTION_GENERATOR				1

/////////////////////////////////
// DEFAULT INTERFACE PUBLISHER //
/////////////////////////////////
#define PUBLISHER_VOLTAGES_INTERFACE			UART_EXTENSION
#define PUBLISHER_MOTION_GENERATOR_INTERFACE	UART_EXTENSION

/////////////////////////////
// DEFAULT PUBLISH ADDRESS //
/////////////////////////////
#define PUBLISHER_VOLTAGES_ADDRESS				ALL
#define PUBLISHER_MOTION_GENERATOR_ADDRESS		ALL

//////////////////////////
// PUBLISHER PRESCALERs //
//////////////////////////
#define PUBLISHER_VOLTAGES_PRESCALER			1
#define PUBLISHER_MOTION_GENERATOR_PRESCALER	1

////////////////////////////////
// DEFAULT ACTIVED PUBLISHERS //
////////////////////////////////
//#define PUBLISHER_VOLTAGES_ACTIVE
//#define PUBLISHER_MOTION_GENERATOR_ACTIVE

////////////////////////////
// DEFAULT ACTIVED TOPICs //
////////////////////////////
// PUBLISHER_VOLTAGES
#define PUBLISH_VOLTAGES_TIMEBASE
//#define PUBLISH_VOLTAGES_3_3V
//#define PUBLISH_VOLTAGES_5V

// PUBLISHER_MOTION_GENERATOR
#define PUBLISH_MOTION_GENERATOR_TIMEBASE
//#define PUBLISH_MOTION_GENERATOR_SETPOINTS

