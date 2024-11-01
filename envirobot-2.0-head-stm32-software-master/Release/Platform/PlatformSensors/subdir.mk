################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Platform/PlatformSensors/PlatformSensors.cpp 

OBJS += \
./Platform/PlatformSensors/PlatformSensors.o 

CPP_DEPS += \
./Platform/PlatformSensors/PlatformSensors.d 


# Each subdirectory must supply rules for building sources it contributes
Platform/PlatformSensors/%.o Platform/PlatformSensors/%.su Platform/PlatformSensors/%.cyclo: ../Platform/PlatformSensors/%.cpp Platform/PlatformSensors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Src -I../Configuration -I../Platform -I../User -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Platform-2f-PlatformSensors

clean-Platform-2f-PlatformSensors:
	-$(RM) ./Platform/PlatformSensors/PlatformSensors.cyclo ./Platform/PlatformSensors/PlatformSensors.d ./Platform/PlatformSensors/PlatformSensors.o ./Platform/PlatformSensors/PlatformSensors.su

.PHONY: clean-Platform-2f-PlatformSensors

