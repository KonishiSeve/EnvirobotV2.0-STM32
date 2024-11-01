#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_COM_ID_PROPAGATION			0X0001 // uint8_t			write only
#define REG_COM_ADDRESS					0x0002 // uint8_t
#define REG_COM_ADD_GROUP_ADDRESS		0X0003 // uint8_t			write only
#define REG_COM_REMOVE_GROUP_ADDRESS	0X0004 // uint8_t			write only
#define REG_COM_CLEAR_GROUP_ADDRESS		0X0005 // uint8_t 						(unused input)
#define REG_COM_GET_GROUP_ADDRESS		0x0006 // uint8_t 	vector	read only
#define REG_COM_FORWARD_MESSAGE			0x0007 // uint8_t 	vector	write only	(0=interface, 1=target_address, ..payload..)
