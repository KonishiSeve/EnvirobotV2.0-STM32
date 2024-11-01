################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Subscribers/Example/ExampleSubscriber.cpp 

OBJS += \
./Src/Subscribers/Example/ExampleSubscriber.o 

CPP_DEPS += \
./Src/Subscribers/Example/ExampleSubscriber.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Subscribers/Example/%.o Src/Subscribers/Example/%.su Src/Subscribers/Example/%.cyclo: ../Src/Subscribers/Example/%.cpp Src/Subscribers/Example/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Src -I../Configuration -I../Platform -I../User -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Subscribers-2f-Example

clean-Src-2f-Subscribers-2f-Example:
	-$(RM) ./Src/Subscribers/Example/ExampleSubscriber.cyclo ./Src/Subscribers/Example/ExampleSubscriber.d ./Src/Subscribers/Example/ExampleSubscriber.o ./Src/Subscribers/Example/ExampleSubscriber.su

.PHONY: clean-Src-2f-Subscribers-2f-Example

