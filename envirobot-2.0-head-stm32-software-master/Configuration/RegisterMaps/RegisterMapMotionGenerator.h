#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_GEN_CONFIGURE_MODULES			0x0400 // uint8_t				write only	(unused input)
#define REG_GEN_MODULES_CONTROLLER_STATUS	0x0401 // uint8_t				write only	(val>0 = active)
#define REG_GEN_GENERATOR_STATUS			0x0402 // uint8_t							(val>0 = active)
#define REG_GEN_SETPOINTS					0x0403 // float		vector		read only
#define REG_GEN_PERIOD						0x0404 // uint32_t
#define REG_GEN_NB_MODULES					0x0405 // uint8_t
#define REG_GEN_GENERATOR_OFFSET			0x0406 // float 	array 2 	write only	(0=index, 1=offset)
#define REG_GEN_RESET_GENERATOR_OFFSETS		0x0407 // uint8_t				write only
#define REG_GEN_POSITION_OFFSET				0x0408 // float 	array 2 	write only	(0=index, 1=offset)
#define REG_GEN_RESET_POSITION_OFFSETS		0x0409 // uint8_t
#define REG_GEN_MODULE_LENGTH				0x040A // float
#define REG_GEN_AMPLITUDE					0x040B // float
#define REG_GEN_FREQUENCY					0x040C // float
#define REG_GEN_WAVELENGTH_INVERSE			0x040D // float
#define REG_GEN_PHASE						0x040E // float
