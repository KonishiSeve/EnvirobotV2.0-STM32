################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Protocols/Payload/Core/PayloadProtocol.cpp 

OBJS += \
./Src/Protocols/Payload/Core/PayloadProtocol.o 

CPP_DEPS += \
./Src/Protocols/Payload/Core/PayloadProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Protocols/Payload/Core/%.o Src/Protocols/Payload/Core/%.su: ../Src/Protocols/Payload/Core/%.cpp Src/Protocols/Payload/Core/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"D:/Code/Envirobot2_0_STM32_Module/Src" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Protocols-2f-Payload-2f-Core

clean-Src-2f-Protocols-2f-Payload-2f-Core:
	-$(RM) ./Src/Protocols/Payload/Core/PayloadProtocol.d ./Src/Protocols/Payload/Core/PayloadProtocol.o ./Src/Protocols/Payload/Core/PayloadProtocol.su

.PHONY: clean-Src-2f-Protocols-2f-Payload-2f-Core

