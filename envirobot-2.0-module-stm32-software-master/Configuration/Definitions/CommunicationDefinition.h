// Addresses
#define CM4_HEAD			0x00
#define STM32_HEAD			0x01
#define UNKNOWN				0xFD
#define PICS				0xFE
#define ALL					0xFF

// Message header
#define ACK_MASK 			0b10000000
#define COMMAND_MASK 		0b01000000
#define ACCESS_MASK 		0b00100000
#define WRITE 				0
#define READ 				1

// Response
#define OK 					0x00
#define UNKNOWN_REGISTER 	0x0F
#define TIMEOUT				0x7F
#define ERROR 				0xFF
