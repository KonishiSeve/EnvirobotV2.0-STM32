################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Protocols/Communication/CANProtocol.cpp \
../Src/Protocols/Communication/CommunicationProtocol.cpp \
../Src/Protocols/Communication/UARTProtocol.cpp 

OBJS += \
./Src/Protocols/Communication/CANProtocol.o \
./Src/Protocols/Communication/CommunicationProtocol.o \
./Src/Protocols/Communication/UARTProtocol.o 

CPP_DEPS += \
./Src/Protocols/Communication/CANProtocol.d \
./Src/Protocols/Communication/CommunicationProtocol.d \
./Src/Protocols/Communication/UARTProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Protocols/Communication/%.o Src/Protocols/Communication/%.su: ../Src/Protocols/Communication/%.cpp Src/Protocols/Communication/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"D:/Code/Envirobot2_0_STM32_Module/Src" -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Protocols-2f-Communication

clean-Src-2f-Protocols-2f-Communication:
	-$(RM) ./Src/Protocols/Communication/CANProtocol.d ./Src/Protocols/Communication/CANProtocol.o ./Src/Protocols/Communication/CANProtocol.su ./Src/Protocols/Communication/CommunicationProtocol.d ./Src/Protocols/Communication/CommunicationProtocol.o ./Src/Protocols/Communication/CommunicationProtocol.su ./Src/Protocols/Communication/UARTProtocol.d ./Src/Protocols/Communication/UARTProtocol.o ./Src/Protocols/Communication/UARTProtocol.su

.PHONY: clean-Src-2f-Protocols-2f-Communication

