################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Actuator/Src/Accel.c \
../Actuator/Src/Actuator.c \
../Actuator/Src/Encoder.c \
../Actuator/Src/Motor.c \
../Actuator/Src/PutBall.c 

OBJS += \
./Actuator/Src/Accel.o \
./Actuator/Src/Actuator.o \
./Actuator/Src/Encoder.o \
./Actuator/Src/Motor.o \
./Actuator/Src/PutBall.o 

C_DEPS += \
./Actuator/Src/Accel.d \
./Actuator/Src/Actuator.d \
./Actuator/Src/Encoder.d \
./Actuator/Src/Motor.d \
./Actuator/Src/PutBall.d 


# Each subdirectory must supply rules for building sources it contributes
Actuator/Src/%.o Actuator/Src/%.su Actuator/Src/%.cyclo: ../Actuator/Src/%.c Actuator/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"E:/LAB/Robocon-2024/Main_F407/Actuator/Inc" -I"E:/LAB/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I"E:/LAB/Robocon-2024/Main_F407/CAN/Inc" -I"E:/LAB/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I"E:/LAB/Robocon-2024/Main_F407/AlgorimthControl/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Actuator-2f-Src

clean-Actuator-2f-Src:
	-$(RM) ./Actuator/Src/Accel.cyclo ./Actuator/Src/Accel.d ./Actuator/Src/Accel.o ./Actuator/Src/Accel.su ./Actuator/Src/Actuator.cyclo ./Actuator/Src/Actuator.d ./Actuator/Src/Actuator.o ./Actuator/Src/Actuator.su ./Actuator/Src/Encoder.cyclo ./Actuator/Src/Encoder.d ./Actuator/Src/Encoder.o ./Actuator/Src/Encoder.su ./Actuator/Src/Motor.cyclo ./Actuator/Src/Motor.d ./Actuator/Src/Motor.o ./Actuator/Src/Motor.su ./Actuator/Src/PutBall.cyclo ./Actuator/Src/PutBall.d ./Actuator/Src/PutBall.o ./Actuator/Src/PutBall.su

.PHONY: clean-Actuator-2f-Src

