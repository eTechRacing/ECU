################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ILI9488/bitmaps/font16.c \
../Core/Src/ILI9488/bitmaps/font24.c \
../Core/Src/ILI9488/bitmaps/font32.c \
../Core/Src/ILI9488/bitmaps/font8.c \
../Core/Src/ILI9488/bitmaps/logo.c \
../Core/Src/ILI9488/bitmaps/race.c 

OBJS += \
./Core/Src/ILI9488/bitmaps/font16.o \
./Core/Src/ILI9488/bitmaps/font24.o \
./Core/Src/ILI9488/bitmaps/font32.o \
./Core/Src/ILI9488/bitmaps/font8.o \
./Core/Src/ILI9488/bitmaps/logo.o \
./Core/Src/ILI9488/bitmaps/race.o 

C_DEPS += \
./Core/Src/ILI9488/bitmaps/font16.d \
./Core/Src/ILI9488/bitmaps/font24.d \
./Core/Src/ILI9488/bitmaps/font32.d \
./Core/Src/ILI9488/bitmaps/font8.d \
./Core/Src/ILI9488/bitmaps/logo.d \
./Core/Src/ILI9488/bitmaps/race.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ILI9488/bitmaps/%.o Core/Src/ILI9488/bitmaps/%.su Core/Src/ILI9488/bitmaps/%.cyclo: ../Core/Src/ILI9488/bitmaps/%.c Core/Src/ILI9488/bitmaps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ILI9488-2f-bitmaps

clean-Core-2f-Src-2f-ILI9488-2f-bitmaps:
	-$(RM) ./Core/Src/ILI9488/bitmaps/font16.cyclo ./Core/Src/ILI9488/bitmaps/font16.d ./Core/Src/ILI9488/bitmaps/font16.o ./Core/Src/ILI9488/bitmaps/font16.su ./Core/Src/ILI9488/bitmaps/font24.cyclo ./Core/Src/ILI9488/bitmaps/font24.d ./Core/Src/ILI9488/bitmaps/font24.o ./Core/Src/ILI9488/bitmaps/font24.su ./Core/Src/ILI9488/bitmaps/font32.cyclo ./Core/Src/ILI9488/bitmaps/font32.d ./Core/Src/ILI9488/bitmaps/font32.o ./Core/Src/ILI9488/bitmaps/font32.su ./Core/Src/ILI9488/bitmaps/font8.cyclo ./Core/Src/ILI9488/bitmaps/font8.d ./Core/Src/ILI9488/bitmaps/font8.o ./Core/Src/ILI9488/bitmaps/font8.su ./Core/Src/ILI9488/bitmaps/logo.cyclo ./Core/Src/ILI9488/bitmaps/logo.d ./Core/Src/ILI9488/bitmaps/logo.o ./Core/Src/ILI9488/bitmaps/logo.su ./Core/Src/ILI9488/bitmaps/race.cyclo ./Core/Src/ILI9488/bitmaps/race.d ./Core/Src/ILI9488/bitmaps/race.o ./Core/Src/ILI9488/bitmaps/race.su

.PHONY: clean-Core-2f-Src-2f-ILI9488-2f-bitmaps

