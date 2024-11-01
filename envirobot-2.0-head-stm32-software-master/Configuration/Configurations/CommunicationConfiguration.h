///////////////////
// Configuration //
///////////////////
#define USE_FDCAN1
//#define USE_FDCAN2
//#define USE_RS485
#define USE_UART_CM4
#define USE_UART_BACK
//#define USE_UART_EXTENSION
#define USE_UART_PIC

///////////////////
// Interface IDs //
///////////////////
#define CANFD1		 		0
#define CANFD2 				1
#define UART_CM4 			2
#define UART_BACK 			3
#define UART_RS485 			4
#define UART_PIC 			5
#define UART_EXTENSION 		6
#define ALL_INTERFACES 		7

//////////////////////////
/// Communication Type ///
//////////////////////////
//#define USE_UINT8_COMMUNICATION
//#define USE_UINT16_COMMUNICATION
//#define USE_UINT32_COMMUNICATION
//#define USE_UINT64_COMMUNICATION
#define USE_INT8_COMMUNICATION
//#define USE_INT16_COMMUNICATION
//#define USE_INT32_COMMUNICATION
//#define USE_INT64_COMMUNICATION
#define USE_FLOAT_COMMUNICATION
//#define USE_DOUBLE_COMMUNICATION

////////////////////////////////
/// Communication Parameters ///
////////////////////////////////
#define DEFAULT_COMMUNICATION_PERIOD	100 // ms
