#include "Configurations/CommunicationConfiguration.h"

///////////////////
// Configuration //
///////////////////
//#define GENERATOR_ACTIVE
//#define GENERATOR_AUTO_MODULE_CONFIGURATION				// only if AUTO_ID_ALLOCATION defined
//#define GENERATOR_AUTO_MODULE_CONTROLLER_ACTIVATION		// only if AUTO_ID_ALLOCATION defined

////////////////////////////////
/// Communication Parameters ///
////////////////////////////////
#define GENERATOR_INTERFACE				CANFD1
#define GENERATOR_CONFIGURATON_DELAY	10		// ms
#define DEFAULT_GENERATOR_PERIOD		10 		// ms
#define DEFAULT_GENERATOR_NB_NODULES	0
#define DEFAULT_MODULE_LENGTH			0.123f	// m
#define DEFAULT_GENERATOR_AMPLITUDE		30.0f 	// °
#define DEFAULT_GENERATOR_FREQUENCY		1.0f 	// Hz
#define DEFAULT_GENERATOR_WAVELENGTH	2.0f	// m
#define DEFAULT_GENERATOR_PHASE			0		// rad
