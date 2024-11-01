/*
 * LEDS.cpp
 *
 *  Created on: Dec 5, 2022
 *      Author: bignet
 */

#include <LEDs/LEDS.hpp>

/**
 * @brief Class constructor
 */
LEDS::LEDS()
{
}

/**
 * @brief Initialize the class with references to other classes. Set default values.
 *
 * @param input registers_: the Registers instance
 */
void LEDS::Init(Registers* registers_) {
	registers = registers_;

	LEDsSemaphore = osSemaphoreNew(1,1,NULL);
	osSemaphoreRelease(LEDsSemaphore);
}

/**
 * @brief Add class related registers
 */
void LEDS::AddRegisters(void) {
	// Register to set a LED state, whether SMD or I2C
	registers->AddRegister<uint8_t>(REG_LEDS_SET);
	registers->SetRegisterAsArray(REG_LEDS_SET, 2);
	registers->AddWriteCallback<uint8_t>(REG_LEDS_SET, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			LEDS* self = (LEDS*) context;
			if (length != 2) return false;

			uint8_t ID = input[0];
			bool success;
			self->GetLED(ID, &success);
			if (success) {
				return self->SetLED(input[0], (input[1] == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
			}

			self->GetI2CLED(ID, &success);
			if (success) {
				return self->SetLED(input[0], input[1]) == HAL_OK;
			}

			return false;
		}
	);

	// Register to setup the LED timings of a SMD LED
	registers->AddRegister<uint8_t>(REG_LEDS_TIMINGS);
	registers->SetRegisterAsVector(REG_LEDS_TIMINGS);
	registers->AddWriteCallback<uint8_t>(REG_LEDS_TIMINGS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			LEDS* self = (LEDS*) context;
			if (length < 3) return false;

			std::vector<uint8_t> timings (&input[1], &input[length-1]);
			return self->ConfigureTimings(input[0], timings, input[length-1]);
		}
	);

	// Register to blink a SMD LED
	registers->AddRegister<uint8_t>(REG_LEDS_BLINK);
	registers->SetRegisterAsSingle(REG_LEDS_BLINK);
	registers->AddWriteCallback<uint8_t>(REG_LEDS_BLINK, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			LEDS* self = (LEDS*) context;
			return self->Blink(*input);
		}
	);

	// Register to blink a SMD LED once only
	registers->AddRegister<uint8_t>(REG_LEDS_BLINK_ONCE);
	registers->SetRegisterAsSingle(REG_LEDS_BLINK_ONCE);
	registers->AddWriteCallback<uint8_t>(REG_LEDS_BLINK_ONCE, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			LEDS* self = (LEDS*) context;
			return self->BlinkOnce(*input);
		}
	);

	// Register to set a RGB LED whether SMD or I2C
	registers->AddRegister<uint8_t>(REG_LEDS_SET_RGB);
	registers->SetRegisterAsArray(REG_LEDS_SET_RGB, 4);
	registers->AddWriteCallback<uint8_t>(REG_LEDS_SET_RGB, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			LEDS* self = (LEDS*) context;
			if (length != 4) return false;

			uint8_t ID = input[0];
			bool success;
			self->GetRGBLED(ID, &success);
			if (success) {
				return self->SetRGB(ID, input[1], input[2], input[3]);
			}

			self->GetI2CRGBLED(ID, &success);
			if (success) {
				return self->SetI2CRGB(ID, input[1], input[2], input[3]) == HAL_OK;
			}

			return false;
		}
	);

	// Register to set brightness of a I2C RGB LED
	registers->AddRegister<uint8_t>(REG_LEDS_SET_BRIGHNESS);
	registers->SetRegisterAsArray(REG_LEDS_SET_BRIGHNESS, 2);
	registers->AddWriteCallback<uint8_t>(REG_LEDS_SET_BRIGHNESS, (void*) this,
		[](void* context, uint16_t register_id, uint8_t* input, uint16_t length) -> bool {
			LEDS* self = (LEDS*) context;
			if (length != 2) return false;
			return self->SetBrightness(input[0], input[1]) == HAL_OK;
		}
	);

}

/**
 * @brief Function used to process the blinking patterns of SMD LEDs. Called repeatedly in a freeRTOS task
 */
void LEDS::Spin(void) {
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
    for (LED & led : LEDs) {
        if (led.blink) {
            bool state = (bool) GPIO_PIN_SET;
            for(uint8_t t : led.timings) {
                if (led.counter < t)
                    break;
                state =  not state;
            }
            HAL_GPIO_WritePin(led.gpio.port, led.gpio.pin, (GPIO_PinState) state);

            led.counter++;
            if (led.counter >= led.T) {
                led.counter = 0;
                if (not led.repeat)
                    led.blink = false;
            }
        }
    }
    osSemaphoreRelease(LEDsSemaphore);

    osDelay(DEFAULT_LEDS_PERIOD);
}

/**
 * @brief Register a SMD LED
 *
 * @param input ID: LED ID to register
 * @param input gpio: the LED GPIO informations
 * @return whether successful
 */
bool LEDS::AddLED(uint8_t ID, GPIO gpio) {
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	// Check LED not registered
	if (ExistingLED(ID)) {
		osSemaphoreRelease(LEDsSemaphore);
		return false;
	}

	// Setup LED
    LED led;
    led.ID = ID;
    led.gpio = gpio;
    led.blink = false;
    led.repeat = false;

    // Add LED
    LEDs.push_back(led);
    osSemaphoreRelease(LEDsSemaphore);

    // Reset pin
    HAL_GPIO_WritePin(led.gpio.port, led.gpio.pin, GPIO_PIN_RESET);
    return true;
}

/**
 * @brief Configure the blinking pattern of a SMD LED
 *
 * @param input ID: LED ID
 * @param input timings: the timing patterns. Initially HIGH, toggled after each element of the vector with respect to the Spin period
 * @param input T: the blinking period with respect to the Spin period
 * @return whether successful
 */
bool LEDS::ConfigureTimings(uint8_t ID, std::vector<uint8_t> timings, uint8_t T) {
    bool success;
    osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
    uint8_t index = GetLED(ID, &success);
    if (success) {
        LEDs[index].timings = timings;
        LEDs[index].T = T;
    }
    osSemaphoreRelease(LEDsSemaphore);
    return success;
}

/**
 * @brief Set a SMD LED state
 *
 * @param input ID: LED ID
 * @param input state: GPIO state
 * @return whether successful
 */
bool LEDS::SetLED(uint8_t ID, GPIO_PinState state) {
    bool success;
    uint8_t index = GetLED(ID, &success);
    if (success) {
    	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
        LEDs[index].blink = false;
        HAL_GPIO_WritePin(LEDs[index].gpio.port, LEDs[index].gpio.pin, state);
    }
    osSemaphoreRelease(LEDsSemaphore);
    return success;
}

/**
 * @brief Check the SMD LED is not used yet (not blinking and not turned on)
 *
 * @param input ID: LED ID
 * @return true if the LED is free
 */
bool LEDS::UnusedLED(uint8_t ID) {
    bool success;
    bool status = false;
    uint8_t index = GetLED(ID, &success);
    if (success) {
    	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
    	GPIO_PinState pin_state = HAL_GPIO_ReadPin(LEDs[index].gpio.port, LEDs[index].gpio.pin);
    	status = (pin_state == GPIO_PIN_SET) || LEDs[index].blink;
    }
    osSemaphoreRelease(LEDsSemaphore);
    return !status;
}

/**
 * @brief Start the blinking pattern of a SMD LED
 *
 * @param input ID: LED ID
 * @return whether successful
 */
bool LEDS::Blink(uint8_t ID) {
    bool success;
    osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
    uint8_t index = GetLED(ID, &success);
    if (success) {
        LEDs[index].blink = true;
        LEDs[index].repeat = true;
        LEDs[index].counter = 0;
    }
    osSemaphoreRelease(LEDsSemaphore);
    return success;
}

/**
 * @brief Start the blinking pattern of a SMD LED for one period only
 *
 * @param input ID: LED ID
 * @return whether successful
 */
bool LEDS::BlinkOnce(uint8_t ID) {
    bool success;
    osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
    uint8_t index = GetLED(ID, &success);
    if (success) {
        LEDs[index].blink = true;
        LEDs[index].repeat = false;
        LEDs[index].counter = 0;
    }
    osSemaphoreRelease(LEDsSemaphore);
    return success;
}

/**
 * @brief Switch off the LED and stop the blinking pattern. For any type of LED.
 *
 * @param input ID: LED ID
 * @return whether successful
 */
bool LEDS::ClearLED(uint8_t ID) {
    bool success;
    osSemaphoreAcquire(LEDsSemaphore, osWaitForever);

    // SMD LED
    uint8_t index = GetLED(ID, &success);
    if (success) {
        LEDs[index].blink = false;
        LEDs[index].repeat = false;
        LEDs[index].counter = 0;
        HAL_GPIO_WritePin(LEDs[index].gpio.port, LEDs[index].gpio.pin, GPIO_PIN_RESET);
        osSemaphoreRelease(LEDsSemaphore);
        return true;
    }

    // RGB SMD LED
    index = GetRGBLED(ID, &success);
	if (success) {
		osSemaphoreRelease(LEDsSemaphore);
		SetRGB(ID, 0, 0, 0);
		return true;
	}

	// I2C LED
    index = GetI2CLED(ID, &success);
    if (success) {
    	osSemaphoreRelease(LEDsSemaphore);
    	SetLED(ID, 0);
    	return true;
    }

    // I2C RGB LED
    index = GetI2CRGBLED(ID, &success);
    if (success) {
    	osSemaphoreRelease(LEDsSemaphore);
		SetI2CRGB(ID, 0, 0, 0);
		return true;
	}

    osSemaphoreRelease(LEDsSemaphore);
    return false;
}

/**
 * @brief Register a RGB LED based on timer PWM
 *
 * @param input ID: LED ID
 * @param input pulse_red_: PWM pulse pointer related to the RED channel of the LED
 * @param input pulse_green_: PWM pulse pointer related to the GREEN channel of the LED
 * @param input pulse_blue_: PWM pulse pointer related to the BLUE channel of the LED
 * @return whether successful
 */
bool LEDS::AddRGBLED(uint8_t ID, uint32_t* pulse_red_, uint32_t* pulse_green_, uint32_t* pulse_blue_) {
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	// Check the LED is not already registered
	if (ExistingLED(ID)) {
		osSemaphoreRelease(LEDsSemaphore);
		return false;
	}

	// Setup LED
	RGBLED led;
	led.ID = ID;
	led.pulse_red = pulse_red_;
	led.pulse_green = pulse_green_;
	led.pulse_blue = pulse_blue_;

	// Add LED
    RGB_LEDs.push_back(led);
    osSemaphoreRelease(LEDsSemaphore);

    // Clear LED
    *pulse_red_ = 0;
    *pulse_green_ = 0;
    *pulse_blue_ = 0;
    return true;
}

/**
 * @brief Set the RGB value of a RGB LED
 *
 * @param input ID: LED ID
 * @param input R: red value
 * @param input G: green value
 * @param input B: blue value
 * @return whether successful
 */
bool LEDS::SetRGB(uint8_t ID, uint8_t R, uint8_t G, uint8_t B) {
    bool success;
    uint8_t index = GetRGBLED(ID, &success);
    if (success) {
    	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
        RGBLED led = RGB_LEDs[index];
        *led.pulse_red = R;
        *led.pulse_green = G;
        *led.pulse_blue = B;
    }
    osSemaphoreRelease(LEDsSemaphore);
    return success;
}

/**
 * @brief Register an I2C LED
 *
 * @param input interface: I2C interface
 * @param input ID: LED ID
 * @param input device_address: I2C address of the I2C LED driver
 * @param input light_address: register address of the light register
 * @return whether successful
 */
bool LEDS::AddI2CLED(I2C_HandleTypeDef* interface, uint8_t ID, uint16_t device_address, uint16_t light_address) {
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	// Check LED is not already registered
	if (ExistingLED(ID)) {
		osSemaphoreRelease(LEDsSemaphore);
		return false;
	}

	// Setup LED
	I2CLED led;
	led.ID = ID;
	led.present = false;							// TODO automatic detection when registered
	led.interface = interface;
	led.device_address = device_address;
	led.memory_address_size = 1;
	led.light_address = light_address;

	// Add LED
	I2C_LEDs.push_back(led);
	osSemaphoreRelease(LEDsSemaphore);
	return true;
}

/**
 * @brief Set the light value of a I2C LED
 *
 * @param input ID: LED ID
 * @param input value: light value
 * @param input timeout: timeout to process the access. Default=HAL_MAX_DELAY
 * @return I2C access status
 */
HAL_StatusTypeDef LEDS::SetLED(uint8_t ID, uint8_t value, uint32_t timeout) {
	bool success;
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	uint8_t index = GetI2CLED(ID, &success);
	if (!success) {
		osSemaphoreRelease(LEDsSemaphore);
		return HAL_ERROR;
	}

	I2CLED* led = &(I2C_LEDs[index]);
	osSemaphoreRelease(LEDsSemaphore);

	return ConfigureI2CLED(ID, led->light_address, value, timeout);
}

/**
 * @brief Register a I2C RGB LED
 *
 * @param input interface: I2C interface
 * @param input ID: LED ID
 * @param input device_address: I2C address of the I2C LED driver
 * @param input brightness_address: register address of the brightness register
 * @param input R_address: register address of the red register
 * @param input G_address: register address of the green register
 * @param input B_address: register address of the blue register
 * @return whether successful
 */
bool LEDS::AddI2CRGBLED(I2C_HandleTypeDef* interface, uint8_t ID, uint16_t device_address, uint16_t brightness_address, uint16_t R_address, uint16_t G_address, uint16_t B_address) {
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	// Check LED is not registered yet
	if (ExistingLED(ID)) {
		osSemaphoreRelease(LEDsSemaphore);
		return false;
	}

	// Setup LED
	I2CRGBLED led;
	led.ID = ID;
	led.present = false;							// TODO automatic detection when registered
	led.interface = interface;
	led.device_address = device_address;
	led.memory_address_size = 1;
	led.brightness_address = brightness_address;
	led.R_address = R_address;
	led.G_address = G_address;
	led.B_address = B_address;

	// Add LED
	I2C_RGB_LEDs.push_back(led);
	osSemaphoreRelease(LEDsSemaphore);
	return true;
}

/**
 * @brief Set brightness register of a I2C RGB LED
 *
 * @param input ID: LED ID
 * @param input brightness: brightness value
 * @param input timeout: timeout to process the access. Default=HAL_MAX_DELAY
 * @return I2C access status
 */
HAL_StatusTypeDef LEDS::SetBrightness(uint8_t ID, uint8_t brightness, uint32_t timeout) {
	bool success;
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	uint8_t index = GetI2CRGBLED(ID, &success);
	if (!success) {
		osSemaphoreRelease(LEDsSemaphore);
		return HAL_ERROR;
	}

	I2CRGBLED* led = &(I2C_RGB_LEDs[index]);
	osSemaphoreRelease(LEDsSemaphore);

	return ConfigureI2CLED(ID, led->brightness_address, brightness, timeout);
}

/**
 * @brief Set RGB registers of a I2C RGB LED
 *
 * @param input ID: LED ID
 * @param input R: red value
 * @param input G: green value
 * @param input B: blue value
 * @param input timeout: timeout to process the access. Default=HAL_MAX_DELAY
 * @return I2C access status
 */
HAL_StatusTypeDef LEDS::SetI2CRGB(uint8_t ID, uint8_t R, uint8_t G, uint8_t B, uint32_t timeout) {
	bool success;
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	uint8_t index = GetI2CRGBLED(ID, &success);
	if (!success) {
		osSemaphoreRelease(LEDsSemaphore);
		return HAL_ERROR;
	}

	I2CRGBLED* led = &(I2C_RGB_LEDs[index]);
	osSemaphoreRelease(LEDsSemaphore);

	HAL_StatusTypeDef status;
	status = ConfigureI2CLED(ID, led->R_address, R, timeout);
	if (status != HAL_OK) return status;
	status = ConfigureI2CLED(ID, led->G_address, G, timeout);
	if (status != HAL_OK) return status;
	status = ConfigureI2CLED(ID, led->B_address, B, timeout);
	if (status != HAL_OK) return status;

	return HAL_OK;
}

/**
 * @brief Set the memory address size of the I2C device of a I2C LED, whether simple of RGB.
 *
 * @param input ID: LED ID
 * @param input size: the memory address size in bytes
 * @return whether successful
 */
bool LEDS::SetI2CAddressSize(uint8_t ID, uint16_t size) {
	bool success;
	uint8_t index;
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);

	// I2C RGB LED
	index = GetI2CRGBLED(ID, &success);
	if (success) {
		I2CRGBLED* led = &(I2C_RGB_LEDs[index]);
		osSemaphoreRelease(LEDsSemaphore);
		led->memory_address_size = size;
		return true;
	}

	// I2C LED
	index = GetI2CLED(ID, &success);
	if (success) {
		I2CLED* led = &(I2C_LEDs[index]);
		osSemaphoreRelease(LEDsSemaphore);
		led->memory_address_size = size;
		return true;
	}

	return false;
}

/**
 * @brief I2C write memory access to the I2C LED driver with a single byte
 *
 * @param input ID: LED ID
 * @param input memory_address: the register address to write to
 * @param input data: single byte to write
 * @param input timeout: timeout to process the access. Default=HAL_MAX_DELAY
 * @return I2C access status
 */
HAL_StatusTypeDef LEDS::ConfigureI2CLED(uint8_t ID, uint16_t memory_address, uint8_t data, uint32_t timeout) {
	uint8_t buffer = data;
	return ConfigureI2CLED(ID, memory_address, &buffer, 1, timeout);
}

/**
 * @brief I2C write memory access to the I2C LED driver with a byte array
 *
 * @param input ID: LED ID
 * @param input memory_address: the register address to write to
 * @param input data: byte array to write
 * @param input size: number of bytes to write
 * @param input timeout: timeout to process the access. Default=HAL_MAX_DELAY
 * @return I2C access status
 */
HAL_StatusTypeDef LEDS::ConfigureI2CLED(uint8_t ID, uint16_t memory_address, uint8_t* data, uint16_t size, uint32_t timeout) {
	uint32_t end_timestamp = HAL_GetTick() + timeout;
	HAL_StatusTypeDef status = HAL_BUSY;
	bool present;
	I2C_HandleTypeDef* interface;
	uint16_t device_address;
	uint16_t memory_address_size;

	bool success;
	uint8_t index;

	// Retrieve information from LED
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	index = GetI2CRGBLED(ID, &success);

	if (success) {
		I2CRGBLED* led = &(I2C_RGB_LEDs[index]);
		present = led->present;
		interface = led->interface;
		device_address = led->device_address;
		memory_address_size = led->memory_address_size;
	} else {
		index = GetI2CLED(ID, &success);
		if (!success) {
			osSemaphoreRelease(LEDsSemaphore);
			return HAL_ERROR;
		}

		I2CLED* led = &(I2C_LEDs[index]);
		present = led->present;
		interface = led->interface;
		device_address = led->device_address;
		memory_address_size = led->memory_address_size;
	}
	osSemaphoreRelease(LEDsSemaphore);

	// I2C Write Access
	if (present) {
		do {
			status = HAL_I2C_Mem_Write(interface, device_address, memory_address, memory_address_size, data, size, timeout);
		} while ((status == HAL_BUSY) && (HAL_GetTick() < end_timestamp)); // try until the bus is free
	}
	return status;
}

/**
 * @brief Detect I2C LEDs
 *
 * @param input trials: number of trials to detect a LED
 * @param input timeout: timeout to process the access. Default=HAL_MAX_DELAY
 */
void LEDS::DetectI2CLEDs(uint32_t trials, uint32_t timeout) {
	HAL_StatusTypeDef status;
	osSemaphoreAcquire(LEDsSemaphore, osWaitForever);
	for (I2CLED &led : I2C_LEDs) {
		status = HAL_I2C_IsDeviceReady(led.interface, led.device_address, trials, timeout);
		led.present = (status == HAL_OK);
	}
	for (I2CRGBLED &led : I2C_RGB_LEDs) {
		status = HAL_I2C_IsDeviceReady(led.interface, led.device_address, trials, timeout);
		led.present = (status == HAL_OK);
	}
	osSemaphoreRelease(LEDsSemaphore);
}

/**
 * @brief Check whether a LED ID is already registered or not
 *
 * @param input ID: LED ID
 * @return true if the LED ID has been found
 */
bool LEDS::ExistingLED(uint8_t ID) {
	bool success;
	GetLED(ID, &success);
	if (success) return true;

	GetRGBLED(ID, &success);
	if (success) return true;

	GetI2CLED(ID, &success);
	if (success) return true;

	GetI2CRGBLED(ID, &success);
	if (success) return true;

	return false;
}

/**
 * @brief Get the LED index based on the ID of a SMD LED
 *
 * @param input ID: LED ID
 * @param output success: true whether successful
 * @return the LED index
 */
uint8_t LEDS::GetLED(uint8_t ID, bool* success) {
    uint8_t index = 0;
    for (LED & led : LEDs) {
        if (led.ID == ID) {
            *success = true;
            return index;
        }
        index++;
    }
    *success = false;
    return 0;
}

/**
 * @brief Get the LED index based on the ID of a SMD RGB LED
 *
 * @param input ID: LED ID
 * @param output success: true whether successful
 * @return the LED index
 */
uint8_t LEDS::GetRGBLED(uint8_t ID, bool* success) {
    uint8_t index = 0;
    for (RGBLED & led : RGB_LEDs) {
        if (led.ID == ID) {
            *success = true;
            return index;
        }
        index++;
    }
    *success = false;
    return 0;
}

/**
 * @brief Get the LED index based on the ID of an I2C LED
 *
 * @param input ID: LED ID
 * @param output success: true whether successful
 * @return the LED index
 */
uint8_t LEDS::GetI2CLED(uint8_t ID, bool* success) {
    uint8_t index = 0;
    for (I2CLED & led : I2C_LEDs) {
        if (led.ID == ID) {
            *success = true;
            return index;
        }
        index++;
    }
    *success = false;
    return 0;
}

/**
 * @brief Get the LED index based on the ID of an I2C RGB LED
 *
 * @param input ID: LED ID
 * @param output success: true whether successful
 * @return the LED index
 */
uint8_t LEDS::GetI2CRGBLED(uint8_t ID, bool* success) {
    uint8_t index = 0;
    for (I2CRGBLED & led : I2C_RGB_LEDs) {
        if (led.ID == ID) {
            *success = true;
            return index;
        }
        index++;
    }
    *success = false;
    return 0;
}

/**
 * @brief Testing procedure of LEDs. Light them one after the other
 */
void LEDS::TestLEDs(void) {
#ifdef USE_SMD_LEDs
	for (LED led : LEDs) {
		SetLED(led.ID, GPIO_PIN_SET);
		HAL_Delay(100);
		SetLED(led.ID, GPIO_PIN_RESET);
	}
	for (RGBLED led : RGB_LEDs) {
		SetRGB(led.ID, 255, 0, 0);
		HAL_Delay(100);
		SetRGB(led.ID, 0, 255, 0);
		HAL_Delay(100);
		SetRGB(led.ID, 0, 0, 255);
		HAL_Delay(100);
		SetRGB(led.ID, 0, 0, 0);
	}
#endif
#ifdef USE_I2C_LEDs
	for (I2CLED led : I2C_LEDs) {
		SetLED(led.ID, 255);
		HAL_Delay(100);
		SetLED(led.ID, 0);
	}
	for (I2CRGBLED led : I2C_RGB_LEDs) {
		SetI2CRGB(led.ID, 255, 0, 0);
		HAL_Delay(100);
		SetI2CRGB(led.ID, 0, 255, 0);
		HAL_Delay(100);
		SetI2CRGB(led.ID, 0, 0, 255);
		HAL_Delay(100);
		SetI2CRGB(led.ID, 0, 0, 0);
	}
#endif
}

///**
// * @brief LEDs after initialization
// */
//void LEDS::StartupLEDS(void) {
//#ifdef USE_SMD_LEDs
//	ConfigureTimings(LED_STATUS, std::vector<uint8_t>{5}, 10);
//	Blink(LED_STATUS); // blink until ID allocated
//#endif
//#ifdef USE_I2C_LEDs
//	SetI2CRGB(LED_RGB_TOP, 255, 255, 255); // white till no ID allocated
//	SetLED(LED_TOP_EDGE, 0);
//#endif
//}
//
///**
// * @brief LEDs once the module address is set
// */
//void LEDS::IDFoundLEDS(void) {
//#ifdef USE_SMD_LEDs
//	SetLED(LED_STATUS, GPIO_PIN_SET);
//#endif
//#ifdef USE_I2C_LEDs
//	SetI2CRGB(LED_RGB_TOP, 0, 255, 0); // green when ID found
//#endif
//}
//
///**
// * @brief LEDs when a water leakage is detected
// */
//void LEDS::WaterDetectedLEDS(void) {
//#ifdef USE_SMD_LEDs
//	SetLED(LED_FAULT, GPIO_PIN_SET);
//#endif
//}
//
///**
// * @brief LEDs when the controller asserts a fault
// */
//void  LEDS::ControllerFaultLEDS(void) {
//#ifdef USE_SMD_LEDs
//	ClearLED(LED_CONTROLLER);
//
//	if (UnusedLED(LED_FAULT)) {
//		ConfigureTimings(LED_FAULT, std::vector<uint8_t>{5}, 10);
//		Blink(LED_FAULT);
//	}
//#endif
//}
//
///**
// * @brief LEDs when a low voltage is detected
// */
//void LEDS::VoltageLowLEDS(void) {
//#ifdef USE_SMD_LEDs
//	if (UnusedLED(LED_FAULT)) {
//		ConfigureTimings(LED_FAULT, std::vector<uint8_t>{1,2,3,4}, 10);
//		Blink(LED_FAULT);
//	}
//#endif
//#ifdef USE_I2C_LEDs
//	SetI2CRGB(LED_RGB_TOP, 255, 165, 0); // orange when battery low
//#endif
//}
//
///**
// * @brief LEDs when a communication occur on a input interface
// *
// * @param input interface_ID: the interface from Communication that asks for a LED single blinking. The ID should be the same than the LED ID
// */
//void LEDS::CommunicationLED(uint8_t interface_ID) {
//#ifdef USE_SMD_LEDs
//	if (interface_ID == UART_PIC || interface_ID == UART_EXTENSION) return; // no LED for these interfaces
//
//	BlinkOnce(interface_ID);
//#endif
//}
