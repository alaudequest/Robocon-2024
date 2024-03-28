################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IoTVisionLibrary/Src/74HC165.c \
../IoTVisionLibrary/Src/74HC595.c \
../IoTVisionLibrary/Src/AMS5915.c \
../IoTVisionLibrary/Src/DS3231.c \
../IoTVisionLibrary/Src/Flag.c \
../IoTVisionLibrary/Src/HX711.c \
../IoTVisionLibrary/Src/PCF8563.c \
../IoTVisionLibrary/Src/PID.c \
../IoTVisionLibrary/Src/RTC_Format.c \
../IoTVisionLibrary/Src/Servo.c \
../IoTVisionLibrary/Src/StringUtility.c \
../IoTVisionLibrary/Src/TB6600.c \
../IoTVisionLibrary/Src/UART_Utility.c 

OBJS += \
./IoTVisionLibrary/Src/74HC165.o \
./IoTVisionLibrary/Src/74HC595.o \
./IoTVisionLibrary/Src/AMS5915.o \
./IoTVisionLibrary/Src/DS3231.o \
./IoTVisionLibrary/Src/Flag.o \
./IoTVisionLibrary/Src/HX711.o \
./IoTVisionLibrary/Src/PCF8563.o \
./IoTVisionLibrary/Src/PID.o \
./IoTVisionLibrary/Src/RTC_Format.o \
./IoTVisionLibrary/Src/Servo.o \
./IoTVisionLibrary/Src/StringUtility.o \
./IoTVisionLibrary/Src/TB6600.o \
./IoTVisionLibrary/Src/UART_Utility.o 

C_DEPS += \
./IoTVisionLibrary/Src/74HC165.d \
./IoTVisionLibrary/Src/74HC595.d \
./IoTVisionLibrary/Src/AMS5915.d \
./IoTVisionLibrary/Src/DS3231.d \
./IoTVisionLibrary/Src/Flag.d \
./IoTVisionLibrary/Src/HX711.d \
./IoTVisionLibrary/Src/PCF8563.d \
./IoTVisionLibrary/Src/PID.d \
./IoTVisionLibrary/Src/RTC_Format.d \
./IoTVisionLibrary/Src/Servo.d \
./IoTVisionLibrary/Src/StringUtility.d \
./IoTVisionLibrary/Src/TB6600.d \
./IoTVisionLibrary/Src/UART_Utility.d 


# Each subdirectory must supply rules for building sources it contributes
IoTVisionLibrary/Src/%.o IoTVisionLibrary/Src/%.su IoTVisionLibrary/Src/%.cyclo: ../IoTVisionLibrary/Src/%.c IoTVisionLibrary/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/Admin/Desktop/Robocon2024/Main_F407/Actuator/Inc" -I"C:/Users/Admin/Desktop/Robocon2024/Main_F407/IoTVisionLibrary/Inc" -I"C:/Users/Admin/Desktop/Robocon2024/Main_F407/CAN/Inc" -I"C:/Users/Admin/Desktop/Robocon2024/Main_F407/IoTVisionLibrary/Inc" -I"C:/Users/Admin/Desktop/Robocon2024/Main_F407/AlgorimthControl/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-IoTVisionLibrary-2f-Src

clean-IoTVisionLibrary-2f-Src:
	-$(RM) ./IoTVisionLibrary/Src/74HC165.cyclo ./IoTVisionLibrary/Src/74HC165.d ./IoTVisionLibrary/Src/74HC165.o ./IoTVisionLibrary/Src/74HC165.su ./IoTVisionLibrary/Src/74HC595.cyclo ./IoTVisionLibrary/Src/74HC595.d ./IoTVisionLibrary/Src/74HC595.o ./IoTVisionLibrary/Src/74HC595.su ./IoTVisionLibrary/Src/AMS5915.cyclo ./IoTVisionLibrary/Src/AMS5915.d ./IoTVisionLibrary/Src/AMS5915.o ./IoTVisionLibrary/Src/AMS5915.su ./IoTVisionLibrary/Src/DS3231.cyclo ./IoTVisionLibrary/Src/DS3231.d ./IoTVisionLibrary/Src/DS3231.o ./IoTVisionLibrary/Src/DS3231.su ./IoTVisionLibrary/Src/Flag.cyclo ./IoTVisionLibrary/Src/Flag.d ./IoTVisionLibrary/Src/Flag.o ./IoTVisionLibrary/Src/Flag.su ./IoTVisionLibrary/Src/HX711.cyclo ./IoTVisionLibrary/Src/HX711.d ./IoTVisionLibrary/Src/HX711.o ./IoTVisionLibrary/Src/HX711.su ./IoTVisionLibrary/Src/PCF8563.cyclo ./IoTVisionLibrary/Src/PCF8563.d ./IoTVisionLibrary/Src/PCF8563.o ./IoTVisionLibrary/Src/PCF8563.su ./IoTVisionLibrary/Src/PID.cyclo ./IoTVisionLibrary/Src/PID.d ./IoTVisionLibrary/Src/PID.o ./IoTVisionLibrary/Src/PID.su ./IoTVisionLibrary/Src/RTC_Format.cyclo ./IoTVisionLibrary/Src/RTC_Format.d ./IoTVisionLibrary/Src/RTC_Format.o ./IoTVisionLibrary/Src/RTC_Format.su ./IoTVisionLibrary/Src/Servo.cyclo ./IoTVisionLibrary/Src/Servo.d ./IoTVisionLibrary/Src/Servo.o ./IoTVisionLibrary/Src/Servo.su ./IoTVisionLibrary/Src/StringUtility.cyclo ./IoTVisionLibrary/Src/StringUtility.d ./IoTVisionLibrary/Src/StringUtility.o ./IoTVisionLibrary/Src/StringUtility.su ./IoTVisionLibrary/Src/TB6600.cyclo ./IoTVisionLibrary/Src/TB6600.d ./IoTVisionLibrary/Src/TB6600.o ./IoTVisionLibrary/Src/TB6600.su ./IoTVisionLibrary/Src/UART_Utility.cyclo ./IoTVisionLibrary/Src/UART_Utility.d ./IoTVisionLibrary/Src/UART_Utility.o ./IoTVisionLibrary/Src/UART_Utility.su

.PHONY: clean-IoTVisionLibrary-2f-Src

