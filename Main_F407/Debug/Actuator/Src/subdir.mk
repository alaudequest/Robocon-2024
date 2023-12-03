################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Actuator/Src/Actuator.c \
../Actuator/Src/Encoder.c 

OBJS += \
./Actuator/Src/Actuator.o \
./Actuator/Src/Encoder.o 

C_DEPS += \
./Actuator/Src/Actuator.d \
./Actuator/Src/Encoder.d 


# Each subdirectory must supply rules for building sources it contributes
Actuator/Src/%.o Actuator/Src/%.su Actuator/Src/%.cyclo: ../Actuator/Src/%.c Actuator/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"D:/Robocon-2024/Main_F407/Actuator/Inc" -I"D:/Robocon-2024/Main_F407/CAN/Inc" -I"D:/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I"D:/Robocon-2024/Main_F407/AlgorimthControl/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Actuator-2f-Src

clean-Actuator-2f-Src:
	-$(RM) ./Actuator/Src/Actuator.cyclo ./Actuator/Src/Actuator.d ./Actuator/Src/Actuator.o ./Actuator/Src/Actuator.su ./Actuator/Src/Encoder.cyclo ./Actuator/Src/Encoder.d ./Actuator/Src/Encoder.o ./Actuator/Src/Encoder.su

.PHONY: clean-Actuator-2f-Src

