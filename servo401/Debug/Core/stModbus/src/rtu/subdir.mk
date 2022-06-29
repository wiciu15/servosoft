################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/stModbus/src/rtu/mbutils.c 

OBJS += \
./Core/stModbus/src/rtu/mbutils.o 

C_DEPS += \
./Core/stModbus/src/rtu/mbutils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/stModbus/src/rtu/%.o Core/stModbus/src/rtu/%.su: ../Core/stModbus/src/rtu/%.c Core/stModbus/src/rtu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Core/stModbus/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-stModbus-2f-src-2f-rtu

clean-Core-2f-stModbus-2f-src-2f-rtu:
	-$(RM) ./Core/stModbus/src/rtu/mbutils.d ./Core/stModbus/src/rtu/mbutils.o ./Core/stModbus/src/rtu/mbutils.su

.PHONY: clean-Core-2f-stModbus-2f-src-2f-rtu

