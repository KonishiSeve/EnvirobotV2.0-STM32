################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Platform/PlatformLEDs/PlatformLEDs.cpp 

OBJS += \
./Platform/PlatformLEDs/PlatformLEDs.o 

CPP_DEPS += \
./Platform/PlatformLEDs/PlatformLEDs.d 


# Each subdirectory must supply rules for building sources it contributes
Platform/PlatformLEDs/%.o Platform/PlatformLEDs/%.su Platform/PlatformLEDs/%.cyclo: ../Platform/PlatformLEDs/%.cpp Platform/PlatformLEDs/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Src -I../User -I../Configuration -I../Configuration/Configurations -I../Configuration/Definitions -I../Configuration/RegisterMaps -I../Platform -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Platform-2f-PlatformLEDs

clean-Platform-2f-PlatformLEDs:
	-$(RM) ./Platform/PlatformLEDs/PlatformLEDs.cyclo ./Platform/PlatformLEDs/PlatformLEDs.d ./Platform/PlatformLEDs/PlatformLEDs.o ./Platform/PlatformLEDs/PlatformLEDs.su

.PHONY: clean-Platform-2f-PlatformLEDs

