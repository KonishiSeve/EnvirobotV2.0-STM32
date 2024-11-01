################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Servomotors/Servomotors.cpp 

OBJS += \
./Src/Servomotors/Servomotors.o 

CPP_DEPS += \
./Src/Servomotors/Servomotors.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Servomotors/%.o Src/Servomotors/%.su Src/Servomotors/%.cyclo: ../Src/Servomotors/%.cpp Src/Servomotors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Src -I../User -I../Configuration -I../Configuration/Configurations -I../Configuration/Definitions -I../Configuration/RegisterMaps -I../Platform -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Servomotors

clean-Src-2f-Servomotors:
	-$(RM) ./Src/Servomotors/Servomotors.cyclo ./Src/Servomotors/Servomotors.d ./Src/Servomotors/Servomotors.o ./Src/Servomotors/Servomotors.su

.PHONY: clean-Src-2f-Servomotors

