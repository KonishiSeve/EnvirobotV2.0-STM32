/**
 * This file defines platform specific LED functions
 */

#pragma once

#include "LEDS/LEDS.hpp"

#include "Configuration.h"
#include "Configurations/LEDsConfiguration.h"
#include "Configurations/CommunicationConfiguration.h"

void StartupLEDS(LEDS* leds);
void IDFoundLEDS(LEDS* leds);
void WaterDetectedLEDS(LEDS* leds);
void CommunicationLED(LEDS* leds, uint8_t interface_ID);
