/*
 * LEDS.hpp
 *
 *  Created on: Dec 5, 2022
 *      Author: bignet
 */

#pragma once

#include <vector>
#include <stm32h750xx.h>
#include <stm32h7xx_hal.h>
#include "cmsis_os.h"

#include "GPIO/GPIO.hpp"
#include "Registers/Registers.hpp"

#include "RegisterMaps/RegisterMapLEDs.h"
#include "Configurations/LEDsConfiguration.h"

// LED struct
struct LED {
    uint8_t ID;							// LED ID
    GPIO gpio;							// GPIO used by the LED
    bool blink;							// true if the LED has to process the timings pattern
    bool repeat;						// true if has to blink repeatedly
    uint8_t counter;					// current counter to determine the LED state in the timings pattern
    std::vector<uint8_t> timings;		// LED blinking pattern
    uint8_t T;							// LED blinking period
};

struct RGBLED {
    uint8_t ID;
    uint32_t* pulse_red;
    uint32_t* pulse_green;
    uint32_t* pulse_blue;
};

struct I2CLED {
    uint8_t ID;
    bool present;
    I2C_HandleTypeDef* interface;
    uint16_t device_address;
    uint16_t memory_address_size;
    uint16_t address_size;
    uint16_t light_address;
};

struct I2CRGBLED {
    uint8_t ID;
    bool present;
    I2C_HandleTypeDef* interface;
    uint16_t device_address;
    uint16_t memory_address_size;
    uint16_t brightness_address;
    uint16_t R_address;
    uint16_t G_address;
    uint16_t B_address;
};

// LEDS class used to control the LED
class LEDS {
public:
    LEDS();
    void Init(Registers* registers);
    void AddRegisters(void);

    void Spin(void);

    // SMD LED
    bool AddLED(uint8_t ID, GPIO gpio);
    bool SetLED(uint8_t ID, GPIO_PinState state);
    bool UnusedLED(uint8_t ID);
    bool Blink(uint8_t ID);
    bool BlinkOnce(uint8_t ID);
    bool ClearLED(uint8_t ID);
    bool ConfigureTimings(uint8_t ID, std::vector<uint8_t> timings, uint8_t T);

    // RGB LED
    bool AddRGBLED(uint8_t ID, uint32_t* pulse_red, uint32_t* pulse_green, uint32_t* pulse_blue);
    bool SetRGB(uint8_t ID, uint8_t R, uint8_t G, uint8_t B);

    // I2C LED
    bool AddI2CLED(I2C_HandleTypeDef* interface, uint8_t ID, uint16_t device_address, uint16_t light_address);
    HAL_StatusTypeDef SetLED(uint8_t ID, uint8_t value, uint32_t timeout = HAL_MAX_DELAY);

    // I2C RGB LED
    bool AddI2CRGBLED(I2C_HandleTypeDef* interface, uint8_t ID, uint16_t device_address, uint16_t brightness_address, uint16_t R_address, uint16_t G_address, uint16_t B_address);
    HAL_StatusTypeDef SetBrightness(uint8_t ID, uint8_t brightness, uint32_t timeout = HAL_MAX_DELAY);
    HAL_StatusTypeDef SetI2CRGB(uint8_t ID, uint8_t R, uint8_t G, uint8_t B, uint32_t timeout = HAL_MAX_DELAY);

    bool SetI2CAddressSize(uint8_t ID, uint16_t size);
    HAL_StatusTypeDef ConfigureI2CLED(uint8_t ID, uint16_t memory_address, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
    HAL_StatusTypeDef ConfigureI2CLED(uint8_t ID, uint16_t memory_address, uint8_t* data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    void DetectI2CLEDs(uint32_t trials, uint32_t timeout = HAL_MAX_DELAY);

    void TestLEDs(void);

private:
    Registers* registers;

    // OS
    osSemaphoreId_t LEDsSemaphore;

    bool ExistingLED(uint8_t ID);
    uint8_t GetLED(uint8_t ID, bool* success);
    uint8_t GetRGBLED(uint8_t ID, bool* success);
    uint8_t GetI2CLED(uint8_t ID, bool* success);
    uint8_t GetI2CRGBLED(uint8_t ID, bool* success);

    std::vector<LED> LEDs;
    std::vector<RGBLED> RGB_LEDs;
    std::vector<I2CLED> I2C_LEDs;
    std::vector<I2CRGBLED> I2C_RGB_LEDs;
};
