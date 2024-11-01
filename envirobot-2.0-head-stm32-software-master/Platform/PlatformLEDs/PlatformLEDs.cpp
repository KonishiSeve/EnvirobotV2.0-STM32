#include "PlatformLEDs/PlatformLEDs.hpp"

/**
 * @brief LEDs after initialization
 *
 * @param input leds: LEDS instance pointer
 */
void StartupLEDS(LEDS* leds) {
#ifdef USE_SMD_LEDs
	leds->ConfigureTimings(LED_STATUS, std::vector<uint8_t>{5}, 10);
	leds->Blink(LED_STATUS); // blink until ID allocated

	leds->SetRGB(LED_RGB, 0, 0, 0);
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
