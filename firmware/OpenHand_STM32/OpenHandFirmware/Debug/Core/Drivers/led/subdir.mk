################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Drivers/led/led.c 

OBJS += \
./Core/Drivers/led/led.o 

C_DEPS += \
./Core/Drivers/led/led.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Drivers/led/%.o Core/Drivers/led/%.su Core/Drivers/led/%.cyclo: ../Core/Drivers/led/%.c Core/Drivers/led/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Core/Drivers -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Drivers-2f-led

clean-Core-2f-Drivers-2f-led:
	-$(RM) ./Core/Drivers/led/led.cyclo ./Core/Drivers/led/led.d ./Core/Drivers/led/led.o ./Core/Drivers/led/led.su

.PHONY: clean-Core-2f-Drivers-2f-led

