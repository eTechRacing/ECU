################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ILI9488/lib/mainDash_lib.c 

OBJS += \
./Core/Src/ILI9488/lib/mainDash_lib.o 

C_DEPS += \
./Core/Src/ILI9488/lib/mainDash_lib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ILI9488/lib/%.o Core/Src/ILI9488/lib/%.su Core/Src/ILI9488/lib/%.cyclo: ../Core/Src/ILI9488/lib/%.c Core/Src/ILI9488/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ILI9488-2f-lib

clean-Core-2f-Src-2f-ILI9488-2f-lib:
	-$(RM) ./Core/Src/ILI9488/lib/mainDash_lib.cyclo ./Core/Src/ILI9488/lib/mainDash_lib.d ./Core/Src/ILI9488/lib/mainDash_lib.o ./Core/Src/ILI9488/lib/mainDash_lib.su

.PHONY: clean-Core-2f-Src-2f-ILI9488-2f-lib

