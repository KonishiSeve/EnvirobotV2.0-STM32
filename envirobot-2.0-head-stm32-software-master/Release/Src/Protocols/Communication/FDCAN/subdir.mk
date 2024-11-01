################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Protocols/Communication/FDCAN/CANProtocol.cpp 

OBJS += \
./Src/Protocols/Communication/FDCAN/CANProtocol.o 

CPP_DEPS += \
./Src/Protocols/Communication/FDCAN/CANProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Protocols/Communication/FDCAN/%.o Src/Protocols/Communication/FDCAN/%.su: ../Src/Protocols/Communication/FDCAN/%.cpp Src/Protocols/Communication/FDCAN/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Src -I../Configuration -I../Platform -I../User -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Protocols-2f-Communication-2f-FDCAN

clean-Src-2f-Protocols-2f-Communication-2f-FDCAN:
	-$(RM) ./Src/Protocols/Communication/FDCAN/CANProtocol.d ./Src/Protocols/Communication/FDCAN/CANProtocol.o ./Src/Protocols/Communication/FDCAN/CANProtocol.su

.PHONY: clean-Src-2f-Protocols-2f-Communication-2f-FDCAN

