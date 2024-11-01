// NOTE: This file is compatible with the sensor MS5803-14BA from TE-Connectivity

#define MS5803_ADDRESS_1 			0xEC // 0x76 << 1
#define MS5803_ADDRESS_2 			0xEE // 0x77 << 1

// Register map
#define MS5803_RESET				0x1E // Software reset of sensor
#define MS5803_OSR_256_P			0x40 // Start pressure conversion with over-sampling of 256
#define MS5803_OSR_512_P			0x42 // Start pressure conversion with over-sampling of 512
#define MS5803_OSR_1024_P			0x44 // Start pressure conversion with over-sampling of 1024
#define MS5803_OSR_2048_P			0x46 // Start pressure conversion with over-sampling of 2048
#define MS5803_OSR_4096_P			0x48 // Start pressure conversion with over-sampling of 4096
#define MS5803_OSR_256_T			0x50 // Start temperature conversion with over-sampling of 256
#define MS5803_OSR_512_T			0x52 // Start temperature conversion with over-sampling of 512
#define MS5803_OSR_1024_T			0x54 // Start temperature conversion with over-sampling of 1024
#define MS5803_OSR_2048_T			0x56 // Start temperature conversion with over-sampling of 2048
#define MS5803_OSR_4096_T			0x58 // Start temperature conversion with over-sampling of 4096
#define MS5803_ADC_READ				0x00 // Prepare an I2C read of the converted value on 3 bytes
#define MS5803_PROM_READ			0xA0 // Start Read of PROM

// Size
#define MS5803_DATA_SIZE			3 	// bytes

// Delays
#define MS5803_OSR_4096_DELAY		10 	//ms
#define MS5803_OSR_2048_DELAY		5 	//ms
#define MS5803_OSR_1024_DELAY		3 	//ms
#define MS5803_OSR_512_DELAY		2 	//ms
#define MS5803_OSR_256_DELAY		1 	//ms

// PROM
#define MS5803_PROM_C1_SENS			0x02 // Pressure sensitivity
#define MS5803_PROM_C2_OFF			0x04 // Pressure offset
#define MS5803_PROM_C3_TCS			0x06 // Temperature coefficient of pressure sensitivity
#define MS5803_PROM_C4_TCO			0x08 // Temperature coefficient of pressure offset
#define MS5803_PROM_C5_TREF			0x0A // Reference temperature
#define MS5803_PROM_C6_TEMPSENS		0x0C // Temperature coefficient of the temperature
