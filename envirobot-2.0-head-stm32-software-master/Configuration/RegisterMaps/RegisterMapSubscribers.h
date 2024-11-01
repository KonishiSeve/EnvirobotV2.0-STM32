#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_SUB_SET_STATUS			0x0020 // uint8_t	array 2		write only	(0=index, 1=value)
#define REG_SUB_ADD_FILTER			0x0021 // uint8_t	array 2		write only	(0=index, 1=address)
#define REG_SUB_CLEAR_FILTERS		0x0022 // uint8_t				write only	(0=index)
