################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32-Library/Src/74HC165.c \
../STM32-Library/Src/74HC595.c \
../STM32-Library/Src/AMS5915.c \
../STM32-Library/Src/DS3231.c \
../STM32-Library/Src/Flag.c \
../STM32-Library/Src/HX711.c \
../STM32-Library/Src/PCF8563.c \
../STM32-Library/Src/PID.c \
../STM32-Library/Src/RTC_Format.c \
../STM32-Library/Src/Servo.c \
../STM32-Library/Src/StringUtility.c \
../STM32-Library/Src/TB6600.c \
../STM32-Library/Src/UART_Utility.c 

OBJS += \
./STM32-Library/Src/74HC165.o \
./STM32-Library/Src/74HC595.o \
./STM32-Library/Src/AMS5915.o \
./STM32-Library/Src/DS3231.o \
./STM32-Library/Src/Flag.o \
./STM32-Library/Src/HX711.o \
./STM32-Library/Src/PCF8563.o \
./STM32-Library/Src/PID.o \
./STM32-Library/Src/RTC_Format.o \
./STM32-Library/Src/Servo.o \
./STM32-Library/Src/StringUtility.o \
./STM32-Library/Src/TB6600.o \
./STM32-Library/Src/UART_Utility.o 

C_DEPS += \
./STM32-Library/Src/74HC165.d \
./STM32-Library/Src/74HC595.d \
./STM32-Library/Src/AMS5915.d \
./STM32-Library/Src/DS3231.d \
./STM32-Library/Src/Flag.d \
./STM32-Library/Src/HX711.d \
./STM32-Library/Src/PCF8563.d \
./STM32-Library/Src/PID.d \
./STM32-Library/Src/RTC_Format.d \
./STM32-Library/Src/Servo.d \
./STM32-Library/Src/StringUtility.d \
./STM32-Library/Src/TB6600.d \
./STM32-Library/Src/UART_Utility.d 


# Each subdirectory must supply rules for building sources it contributes
STM32-Library/Src/%.o STM32-Library/Src/%.su STM32-Library/Src/%.cyclo: ../STM32-Library/Src/%.c STM32-Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"E:/LAB/Robocon-2024/NodeSwerve_F103/CAN/Inc" -I"E:/LAB/Robocon-2024/NodeSwerve_F103/Communication/Inc" -I"E:/LAB/Robocon-2024/NodeSwerve_F103/STM32-Library/Inc" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32-2d-Library-2f-Src

clean-STM32-2d-Library-2f-Src:
	-$(RM) ./STM32-Library/Src/74HC165.cyclo ./STM32-Library/Src/74HC165.d ./STM32-Library/Src/74HC165.o ./STM32-Library/Src/74HC165.su ./STM32-Library/Src/74HC595.cyclo ./STM32-Library/Src/74HC595.d ./STM32-Library/Src/74HC595.o ./STM32-Library/Src/74HC595.su ./STM32-Library/Src/AMS5915.cyclo ./STM32-Library/Src/AMS5915.d ./STM32-Library/Src/AMS5915.o ./STM32-Library/Src/AMS5915.su ./STM32-Library/Src/DS3231.cyclo ./STM32-Library/Src/DS3231.d ./STM32-Library/Src/DS3231.o ./STM32-Library/Src/DS3231.su ./STM32-Library/Src/Flag.cyclo ./STM32-Library/Src/Flag.d ./STM32-Library/Src/Flag.o ./STM32-Library/Src/Flag.su ./STM32-Library/Src/HX711.cyclo ./STM32-Library/Src/HX711.d ./STM32-Library/Src/HX711.o ./STM32-Library/Src/HX711.su ./STM32-Library/Src/PCF8563.cyclo ./STM32-Library/Src/PCF8563.d ./STM32-Library/Src/PCF8563.o ./STM32-Library/Src/PCF8563.su ./STM32-Library/Src/PID.cyclo ./STM32-Library/Src/PID.d ./STM32-Library/Src/PID.o ./STM32-Library/Src/PID.su ./STM32-Library/Src/RTC_Format.cyclo ./STM32-Library/Src/RTC_Format.d ./STM32-Library/Src/RTC_Format.o ./STM32-Library/Src/RTC_Format.su ./STM32-Library/Src/Servo.cyclo ./STM32-Library/Src/Servo.d ./STM32-Library/Src/Servo.o ./STM32-Library/Src/Servo.su ./STM32-Library/Src/StringUtility.cyclo ./STM32-Library/Src/StringUtility.d ./STM32-Library/Src/StringUtility.o ./STM32-Library/Src/StringUtility.su ./STM32-Library/Src/TB6600.cyclo ./STM32-Library/Src/TB6600.d ./STM32-Library/Src/TB6600.o ./STM32-Library/Src/TB6600.su ./STM32-Library/Src/UART_Utility.cyclo ./STM32-Library/Src/UART_Utility.d ./STM32-Library/Src/UART_Utility.o ./STM32-Library/Src/UART_Utility.su

.PHONY: clean-STM32-2d-Library-2f-Src

