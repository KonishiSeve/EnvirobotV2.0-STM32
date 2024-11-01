#include "Definitions/RegistersDefinition.h"

// Registers
#define REG_PUB_SET_STATUS			0x0010 // uint8_t	array 2		write only	(0=index, 1=value)
#define REG_PUB_SET_TOPIC_STATUS	0x0011 // uint16_t	array 3		write only	(0=index, 1=topic, 2=value)
#define REG_PUB_SET_PRESCALER		0x0012 // uint16_t	array 2		write only	(0=index, 1=value)
#define REG_PUB_STOP_TOPICS			0x0013 // uint8_t				write only 	(0=index)
#define REG_PUB_LINK_INTERFACE		0x0014 // uint8_t	array 2 	write only	(0=index, 1=interface)
#define REG_PUB_UNLINK_INTERFACE	0x0015 // uint8_t	array 2 	write only	(0=index, 1=interface)
#define REG_PUB_CLEAR_INTERFACES	0x0016 // uint8_t				write only	(0=index)
#define REG_PUB_SET_TARGET_ADDRESS	0x0017 // uint8_t	array 3		write only 	(0=index, 1=interface, 2=publish_address)
