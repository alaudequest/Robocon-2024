################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Communication/Src/AppInterface.c \
../Communication/Src/CRC16.c \
../Communication/Src/NodeSwerve_AppInterface.c 

OBJS += \
./Communication/Src/AppInterface.o \
./Communication/Src/CRC16.o \
./Communication/Src/NodeSwerve_AppInterface.o 

C_DEPS += \
./Communication/Src/AppInterface.d \
./Communication/Src/CRC16.d \
./Communication/Src/NodeSwerve_AppInterface.d 


# Each subdirectory must supply rules for building sources it contributes
Communication/Src/%.o Communication/Src/%.su Communication/Src/%.cyclo: ../Communication/Src/%.c Communication/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"E:/LAB/Robocon-2024/NodeSwerve_F103/CAN/Inc" -I"E:/LAB/Robocon-2024/NodeSwerve_F103/Communication/Inc" -I"E:/LAB/Robocon-2024/NodeSwerve_F103/STM32-Library/Inc" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Communication-2f-Src

clean-Communication-2f-Src:
	-$(RM) ./Communication/Src/AppInterface.cyclo ./Communication/Src/AppInterface.d ./Communication/Src/AppInterface.o ./Communication/Src/AppInterface.su ./Communication/Src/CRC16.cyclo ./Communication/Src/CRC16.d ./Communication/Src/CRC16.o ./Communication/Src/CRC16.su ./Communication/Src/NodeSwerve_AppInterface.cyclo ./Communication/Src/NodeSwerve_AppInterface.d ./Communication/Src/NodeSwerve_AppInterface.o ./Communication/Src/NodeSwerve_AppInterface.su

.PHONY: clean-Communication-2f-Src

