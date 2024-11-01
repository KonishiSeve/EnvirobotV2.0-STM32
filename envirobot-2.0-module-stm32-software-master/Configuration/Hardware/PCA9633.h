#define PCA9633_ADDRESS					0xC4 // 0x62 << 1

#define PCA9633_MODE1 					0x00 // Mode register 1
#define PCA9633_MODE2 					0x01 // Mode register 2
#define PCA9633_PWM0 					0x02 // brightness control LED0
#define PCA9633_PWM1 					0x03 // brightness control LED1
#define PCA9633_PWM2 					0x04 // brightness control LED2
#define PCA9633_PWM3 					0x05 // brightness control LED3
#define PCA9633_GRPPWM 					0x06 // group duty cycle control
#define PCA9633_GRPFREQ 				0x07 // group frequency
#define PCA9633_LEDOUT 					0x08 // LED output state
#define PCA9633_SUBADR1 				0x09 // I2C-bus subaddress 1
#define PCA9633_SUBADR2 				0x0A // I2C-bus subaddress 2
#define PCA9633_SUBADR3 				0x0B // I2C-bus subaddress 3
#define PCA9633_ALLCALLADR 				0x0C // LED All Call I2C-bus address

// PCA9633_MODE1
#define PCA9633_MODE1_AI2				0b10000000 // Register Auto-Increment
#define PCA9633_MODE1_AI1				0b01000000 // Auto-Increment bit1
#define PCA9633_MODE1_AI0				0b00100000 // Auto-Increment bit0
#define PCA9633_MODE1_SLEEP				0b00010000 // 0: Normal mode, 1: Low power mode. Oscillator off
#define PCA9633_MODE1_SUB1				0b00001000 // 0: PCA9633 does not respond to I2C-bus subaddress 1,  PCA9633 responds to I2C-bus subaddress 1
#define PCA9633_MODE1_SUB2				0b00000100 // 0: PCA9633 does not respond to I2C-bus subaddress 2,  PCA9633 responds to I2C-bus subaddress 2
#define PCA9633_MODE1_SUB3				0b00000010 // 0: PCA9633 does not respond to I2C-bus subaddress 3,  PCA9633 responds to I2C-bus subaddress 3
#define PCA9633_MODE1_ALLCALL			0b00000001 // 0: PCA9633 does not respond to LED All Call I2C-bus address,  PCA9633 responds to LED All Call I2C-bus address

// PCA9633_MODE2
#define PCA9633_MODE2_DMBLNK			0b00100000 // 0: Group control = dimming, Group control = blinking
#define PCA9633_MODE2_INVRT				0b00010000 // 0: Output logic state not inverted, 1: Output logic state inverted
#define PCA9633_MODE2_OCH				0b00001000 // 0: Outputs change on STOP command, 1: Outputs change on ACK
#define PCA9633_MODE2_OUTDRV			0b00000100 // 0: Open-Drain structure, 1: Totem Pole structure
#define PCA9633_MODE2_OUTNE				0b00000011 // 0x00 : LEDn = 0, 0x01: LEDn = 1 when OUTDRV = 1 and LEDn = high-impedance when OUTDRV = 0 (same as OUTNE[1:0] = 10), 0x10: LEDn = high-impedance

// PCA9633_LEDOUT
#define PCA9633_LEDOUT_LDR3_BASE		6 // LED3 output state control
#define PCA9633_LEDOUT_LDR2_BASE		4 // LED2 output state control
#define PCA9633_LEDOUT_LDR1_BASE		2 // LED1 output state control
#define PCA9633_LEDOUT_LDR0_BASE		0 // LED0 output state control

#define PCA9633_LEDOUT_LDRx_OFF			0x00 // LED driver x is off
#define PCA9633_LEDOUT_LDRx_ON			0x01 // LED driver x is fully on
#define PCA9633_LEDOUT_LDRx_PWM			0x10 // LED driver x individual brightness can be controlled through its PWMx register
#define PCA9633_LEDOUT_LDRx_PWM_GRPPWM	0x11 //  LED driver x individual brightness and group dimming/blinking can be controlled through its PWMx register and the GRPPWM registers
