#define LTC2947_ADDRESS					0xB8 // 0x5C << 1

// Accumulated Results
#define LTC2947_C1						0x00 // Charge1
#define LTC2947_E1						0x06 // Energy1
#define LTC2947_TB1						0x0C // Time1
#define LTC2947_C2						0x10 // Charge2
#define LTC2947_E2						0x16 // Energy2
#define LTC2947_TB2						0x1C // Time2

#define LTC2947_CHARGE1_SIZE			6 // in bytes
#define LTC2947_ENERGY1_SIZE			6
#define LTC2947_TIME1_SIZE				4
#define LTC2947_CHARGE2_SIZE			6
#define LTC2947_ENERGY2_SIZE			6
#define LTC2947_TIME2_SIZE				4

// Tracking
#define LTC2947_IMAX					0x40 // Maximum Current
#define LTC2947_IMIN					0x42 // Minimum Current
#define LTC2947_PMAX					0x44 // Maximum Power
#define LTC2947_PMIN					0x46 // Minimum Power
#define LTC2947_VMAX					0x50 // Maximum Voltage VD
#define LTC2947_VMIN					0x52 // Minimum Voltage VD
#define LTC2947_TEMPMAX					0x54 // Maximum Temperature
#define LTC2947_TEMPMIN					0x56 // Minimum Temperature
#define LTC2947_VDVCCMAX				0x58 // Maximum Voltage at DVCC
#define LTC2947_VDVCCMIN				0x5A // Minimum Voltage at DVCC

#define LTC2947_IMAX_SIZE				2
#define LTC2947_IMIN_SIZE				2
#define LTC2947_PMAX_SIZE				2
#define LTC2947_PMIN_SIZE				2
#define LTC2947_VMAX_SIZE				2
#define LTC2947_VMIN_SIZE				2
#define LTC2947_TEMPMAX_SIZE			2
#define LTC2947_TEMPMIN_SIZE			2
#define LTC2947_VDVCCMAX_SIZE			2
#define LTC2947_VDVCCMIN_SIZE			2

// GPIO
#define LTC2947_GPIOSTATCTL				0x67 // GPIO Status and Control

#define LTC2947_GPIOSTATCTL_SIZE		1

// LTC2947_GPIOSTATCTL
#define LTC2947_GPIOSTATCTL_GPOEN		0b00000001 // 0: GPIO Input, 1: GPIO Output
#define LTC2947_GPIOSTATCTL_GPI			0b00010000 // 0: Logical level 0 at pin GPIO, 1: Logical level 1 at pin GPIO
#define LTC2947_GPIOSTATCTL_GPO			0b00100000 // 0: Pin GPIO is set to 0 if set as output, 1: Pin GPIO is set to 1 if set as output
#define LTC2947_GPIOSTATCTL_FANEN		0b01000000 // 0: GPIO level controlled by GPO, 1: GPIO level controlled by temperature measurement against fan temperature threshold high/low registers TEMPFANH (page1.0x9C) and TEMPFANL (page1.0x9E)
#define LTC2947_GPIOSTATCTL_FANPOL		0b10000000 // 0: GPIO is low active, 1: GPIO is high active

// Status
#define LTC2947_STATUS					0x80 // Status

#define LTC2947_STATUS_SIZE				1

#define LTC2947_STATUS_UVLOA			0b00000001 // 1: Undervoltage in the analog domain including ADCs during a conversion
#define LTC2947_STATUS_PORA				0b00000010 // 1: Power-on reset has occurred due to undervoltage in the analog domain
#define LTC2947_STATUS_UVLOSTBY			0b00000100 // 1: Undervoltage in the standby domain
#define LTC2947_STATUS_UVLOD			0b00001000 // 1: Undervoltage in the digital domain
#define LTC2947_STATUS_UPDATE			0b00010000 // 1: Result registers have been updated
#define LTC2947_STATUS_ADCERR			0b00100000 // 1: The ADC conversion is not valid due to undervoltage during a conversion
#define LTC2947_STATUS_TBERR			0b01000000 // 1: Overflow of the internal timebase register. The values of accumulated result registers are invalid


// Threshold And Overflow Alerts
#define LTC2947_STATVT					0x81 // Voltage, Temperature Threshold Alerts
#define LTC2947_STATIP					0x82 // Current, Power Threshold Alerts
#define LTC2947_STATC					0x83 // Charge Threshold Alerts
#define LTC2947_STATE					0x84 // Energy Threshold Alerts
#define LTC2947_STATCEOF				0x85 // Charge, Energy Overflow Alerts
#define LTC2947_STATTB					0x86 // Timebase Alerts
#define LTC2947_STATVDVCC				0x87 // VDVCC Threshold Alerts

#define LTC2947_STATVT_SIZE				1
#define LTC2947_STATIP_SIZE				1
#define LTC2947_STATC_SIZE				1
#define LTC2947_STATE_SIZE				1
#define LTC2947_STATCEOF_SIZE			1
#define LTC2947_STATTB_SIZE				1
#define LTC2947_STATVDVCC_SIZE			1

// LTC2947_STATVT
#define LTC2947_STATVT_VH				0b00000001 // 1: Voltage VD high threshold exceeded
#define LTC2947_STATVT_VL				0b00000010 // 1: Voltage VD low threshold exceeded
#define LTC2947_STATVT_TEMPH			0b00000100 // 1: Temperature high threshold exceeded
#define LTC2947_STATVT_TEMPL			0b00001000 // 1: Temperature low threshold exceeded
#define LTC2947_STATVT_FANH				0b00010000 // 1: Fan high temperature threshold exceeded
#define LTC2947_STATVT_FANL				0b00100000 // 1: Fan low temperature threshold exceeded

// LTC2947_STATIP
#define LTC2947_STATIP_IH				0b00000001 // 1: Current high threshold exceeded
#define LTC2947_STATIP_IL				0b00000010 // 1: Current low threshold exceeded
#define LTC2947_STATIP_PH				0b00000100 // 1: Power high threshold exceeded
#define LTC2947_STATIP_PL				0b00001000 // 1: Power low threshold exceeded

// LTC2947_STATC
#define LTC2947_STATC_C1H				0b00000001 // 1: Charge1 high threshold exceeded
#define LTC2947_STATC_C1L				0b00000010 // 1: Charge1 low threshold exceeded
#define LTC2947_STATC_C2H				0b00000100 // 1: Charge2 high threshold exceeded
#define LTC2947_STATC_C2L				0b00001000 // 1: Charge2 low threshold exceeded

// LTC2947_STATE
#define LTC2947_STATE_E1H				0b00000001 // 1: Energy1 high threshold exceeded
#define LTC2947_STATE_E1L				0b00000010 // 1: Energy1 low threshold exceeded
#define LTC2947_STATE_E2H				0b00000100 // 1: Energy2 high threshold exceeded
#define LTC2947_STATE_E2L				0b00001000 // 1: Energy2 low threshold exceeded

// LTC2947_STATCEOF
#define LTC2947_STATEOF_C1OF			0b00000001 // 1: Charge1 overflow alert
#define LTC2947_STATEOF_C2OF			0b00000010 // 1: Charge2 overflow alert
#define LTC2947_STATEOF_E1OF			0b00010000 // 1: Energy1 overflow alert
#define LTC2947_STATEOF_E2OF			0b00100000 // 1: Energy2 overflow alert

// LTC2947_STATTB
#define LTC2947_STATTB_TB1TH			0b00000001 // 1: Time1 threshold exceeded
#define LTC2947_STATTB_TB2TH			0b00000010 // 1: Time2 threshold exceeded
#define LTC2947_STATTB_TB1OF			0b00010000 // 1: Time1 overflow
#define LTC2947_STATTB_TB2OF			0b00100000 // 1: Time2 overflow

// LTC2947_STATVDVCC
#define LTC2947_STATVDVCC_VDVCCH		0b00000001 // 1: Voltage at DVCC high threshold exceeded
#define LTC2947_STATVDVCC_VDVCCL		0b00000010 // 1: Voltage at DVCC low threshold exceeded

// Mask
#define LTC2947_STATUSM					0x88 // Status Mask
#define LTC2947_STATVTM					0x89 // Voltage, Temperature Threshold Alert Mask
#define LTC2947_STATIPM					0x8A // Current, Power Threshold Alert Mask
#define LTC2947_STATCM					0x8B // Charge Threshold Alerts Mask
#define LTC2947_STATEM					0x8C // Energy Threshold Alerts Mask
#define LTC2947_STATCEOFM				0x8D // Charge, Energy Overflow Alerts Mask
#define LTC2947_STATTBM					0x8E // Timebase Alerts Mask
#define LTC2947_STATVDVCCM				0x8F // VDVCC Threshold Alerts Mask

#define LTC2947_STATUSM_SIZE			1
#define LTC2947_STATVTM_SIZE			1
#define LTC2947_STATIPM_SIZE			1
#define LTC2947_STATCM_SIZE				1
#define LTC2947_STATEM_SIZE				1
#define LTC2947_STATCEOFM_SIZE			1
#define LTC2947_STATTBM_SIZE			1
#define LTC2947_STATVDVCCM_SIZE			1

// Non Accumulated Results
#define LTC2947_I						0x90 // Current
#define LTC2947_P						0x93 // Power
#define LTC2947_V						0xA0 // Voltage
#define LTC2947_TEMP					0xA2 // Temperature
#define LTC2947_VDVCC					0xA4 // Voltage at DVCC
#define LTC2947_IH1						0xB0 // Current History 1
#define LTC2947_IH2						0xB3 // Current History 2
#define LTC2947_IH3						0xB6 // Current History 3
#define LTC2947_IH4						0xB9 // Current History 4
#define LTC2947_IH5						0xBC // Current History 5

#define LTC2947_I_SIZE					3
#define LTC2947_P_SIZE					3
#define LTC2947_V_SIZE					2
#define LTC2947_TEMP_SIZE				2
#define LTC2947_VDVCC_SIZE				2
#define LTC2947_IH1_SIZE				3
#define LTC2947_IH2_SIZE				3
#define LTC2947_IH3_SIZE				3
#define LTC2947_IH4_SIZE				3
#define LTC2947_IH5_SIZE				3

// Control
#define LTC2947_ACCICTL					0xE1 // Accumulator Control Current Polarity
#define LTC2947_ACCGPCTL				0xE3 // Accumulator Control GPIO
#define LTC2947_ACCIDB					0xE4 // Accumulation Deadband
#define LTC2947_ALERTBCTL				0xE8 // Alert Master Control Enable
#define LTC2947_TBCTL					0xE9 // Timebase Control
#define LTC2947_OPCTL					0xF0 // Operation Control
#define LTC2947_PGCTL					0xFF // Page Control

#define LTC2947_ACCICTL_SIZE			1
#define LTC2947_ACCGPCTL_SIZE			1
#define LTC2947_ACCIDB_SIZE				1
#define LTC2947_ALERTBCTL_SIZE			1
#define LTC2947_TBCTL_SIZE				1
#define LTC2947_OPCTL_SIZE				1
#define LTC2947_PGCTL_SIZE				1

// LTC2947_ACCICTL
#define LTC2947_ACCICTL_ACC1I_BASE		0
#define LTC2947_ACCICTL_ACC2I_BASE		2

#define LTC2947_ACCICTL_ACC1I_MASK		0b00000011
#define LTC2947_ACCICTL_ACC2I_MASK		0b00001100

#define LTC2947_ACCICTL_ACCxI_ON		0x00 // Accumulation takes place always
#define LTC2947_ACCICTL_ACCxI_POS		0x01 // Only if the current is positive
#define LTC2947_ACCICTL_ACCxI_NEG		0x10 // Only if the current is negative
#define LTC2947_ACCICTL_ACCxI_OFF		0x11 // No accumulation takes place

// LTC2947_ACCGPCTL
#define LTC2947_ACCGPCTL_ACC1GP_BASE	0
#define LTC2947_ACCGPCTL_ACC2GP_BASE	2

#define LTC2947_ACCGPCTL_ACC1GP_MASK	0b00000011
#define LTC2947_ACCGPCTL_ACC2GP_MASK	0b00001100

#define LTC2947_ACCGPCTL_ACCxGP_ON			0x00 // Accumulation takes place always
#define LTC2947_ACCGPCTL_ACCxGP_GPIO_ON		0x01 // Only if pin GPIO is 1
#define LTC2947_ACCGPCTL_ACCxGP_GPIO_OFF	0x10 // Only if pin GPIO is 0

// LTC2947_TBCTL
#define LTC2947_TBCTL_PRE_BASE			0
#define LTC2947_TBCTL_DIV_BASE			3

#define LTC2947_TBCTL_PRE_MASK			0b00000111
#define LTC2947_TBCTL_DIV_MASK			0b11111000

// LTC2947_OPCTL
#define LTC2947_OPCTL_SHDN				0b00000001 // Shutdown
#define LTC2947_OPCTL_CLR				0b00000010 // Clear
#define LTC2947_OPCTL_SSHOT				0b00000100 // Single Shot Measurement
#define LTC2947_OPCTL_CONT				0b00001000 // Continuous measurement
#define LTC2947_OPCTL_RST				0b10000000 // Global Reset
