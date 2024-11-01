#include "PlatformLEDs/PlatformLEDs.hpp"

/**
 * @brief LEDs after initialization
 *
 * @param input leds: LEDS instance pointer
 */
void StartupLEDS(LEDS* leds) {
#ifdef USE_SMD_LEDs
	leds->ConfigureTimings(LED_STATUS, std::vector<uint8_t>{5}, 10);
	leds->Blink(LED_STATUS); 											// blink until ID allocated
#endif
#ifdef USE_I2C_LEDs
	leds->SetI2CRGB(LED_RGB_TOP, 255, 255, 255); 						// white till no ID allocated
	leds->SetLED(LED_TOP_EDGE, 0);
#endif
}

/**
 * @brief LEDs once the module address is set. MANDATORY if Communication class is used
 *
 * @param input leds: LEDS instance pointer
 */
void IDFoundLEDS(LEDS* leds) {
#ifdef USE_SMD_LEDs
	leds->SetLED(LED_STATUS, GPIO_PIN_SET);
#endif
#ifdef USE_I2C_LEDs
	leds->SetI2CRGB(LED_RGB_TOP, 0, 255, 0); // green when ID found
#endif
}

/**
 * @brief LEDs when a water leakage is detected
 *
 * @param input leds: LEDS instance pointer
 */
void WaterDetectedLEDS(LEDS* leds) {
#ifdef USE_SMD_LEDs
	leds->SetLED(LED_FAULT, GPIO_PIN_SET);
#endif
}

/**
 * @brief LEDs when the controller asserts a fault. MANDATORY if Controller class is used
 *
 * @param input leds: LEDS instance pointer
 */
void ControllerFaultLEDS(LEDS* leds) {
#ifdef USE_SMD_LEDs
	leds->ClearLED(LED_CONTROLLER);

	if (leds->UnusedLED(LED_FAULT)) {
		leds->ConfigureTimings(LED_FAULT, std::vector<uint8_t>{5}, 10);
		leds->Blink(LED_FAULT);
	}
#endif
}

/**
 * @brief LEDs when a low voltage is detected.
 *
 * @param input leds: LEDS instance pointer
 */
void VoltageLowLEDS(LEDS* leds) {
#ifdef USE_SMD_LEDs
	if (leds->UnusedLED(LED_FAULT)) {
		leds->ConfigureTimings(LED_FAULT, std::vector<uint8_t>{1,2,3,4}, 10);
		leds->Blink(LED_FAULT);
	}
#endif
#ifdef USE_I2C_LEDs
	leds->SetI2CRGB(LED_RGB_TOP, 255, 165, 0); // orange when battery low
#endif
}

/**
 * @brief LEDs when a communication occur on a input interface. MANDATORY if Communication, Services or Subscribers class is used
 *
 * @param input leds: LEDS instance pointer
 * @param input interface_ID: the interface from Communication that asks for a LED single blinking. The ID should be the same than the LED ID
 */
void CommunicationLED(LEDS* leds, uint8_t interface_ID) {
#ifdef USE_SMD_LEDs
	if (interface_ID == UART_PIC || interface_ID == UART_EXTENSION) return; // no LED for these interfaces

	leds->BlinkOnce(interface_ID);
#endif
}
