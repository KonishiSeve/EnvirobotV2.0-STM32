################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Protocols/Payload/PayloadProtocol.cpp 

OBJS += \
./Src/Protocols/Payload/PayloadProtocol.o 

CPP_DEPS += \
./Src/Protocols/Payload/PayloadProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Protocols/Payload/%.o Src/Protocols/Payload/%.su: ../Src/Protocols/Payload/%.cpp Src/Protocols/Payload/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"D:/Code/Envirobot2_0_STM32_Module/Src" -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Protocols-2f-Payload

clean-Src-2f-Protocols-2f-Payload:
	-$(RM) ./Src/Protocols/Payload/PayloadProtocol.d ./Src/Protocols/Payload/PayloadProtocol.o ./Src/Protocols/Payload/PayloadProtocol.su

.PHONY: clean-Src-2f-Protocols-2f-Payload

