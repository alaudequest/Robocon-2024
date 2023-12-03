################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Robocon-2024/Main_F407/CAN/Src/CAN_Control.c \
D:/Robocon-2024/Main_F407/CAN/Src/CAN_FuncHandle.c 

OBJS += \
./HuyCAN/Src/CAN_Control.o \
./HuyCAN/Src/CAN_FuncHandle.o 

C_DEPS += \
./HuyCAN/Src/CAN_Control.d \
./HuyCAN/Src/CAN_FuncHandle.d 


# Each subdirectory must supply rules for building sources it contributes
HuyCAN/Src/CAN_Control.o: D:/Robocon-2024/Main_F407/CAN/Src/CAN_Control.c HuyCAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/Robocon-2024/Main_F407/CAN/Inc" -I"D:/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I../Core/Inc -I"D:/Robocon-2024/Main_F407/CAN/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
HuyCAN/Src/CAN_FuncHandle.o: D:/Robocon-2024/Main_F407/CAN/Src/CAN_FuncHandle.c HuyCAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"D:/Robocon-2024/Main_F407/CAN/Inc" -I"D:/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I../Core/Inc -I"D:/Robocon-2024/Main_F407/CAN/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HuyCAN-2f-Src

clean-HuyCAN-2f-Src:
	-$(RM) ./HuyCAN/Src/CAN_Control.cyclo ./HuyCAN/Src/CAN_Control.d ./HuyCAN/Src/CAN_Control.o ./HuyCAN/Src/CAN_Control.su ./HuyCAN/Src/CAN_FuncHandle.cyclo ./HuyCAN/Src/CAN_FuncHandle.d ./HuyCAN/Src/CAN_FuncHandle.o ./HuyCAN/Src/CAN_FuncHandle.su

.PHONY: clean-HuyCAN-2f-Src

