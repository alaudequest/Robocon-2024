################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CAN/Src/CAN_Control.c \
../CAN/Src/CAN_FuncHandle.c 

OBJS += \
./CAN/Src/CAN_Control.o \
./CAN/Src/CAN_FuncHandle.o 

C_DEPS += \
./CAN/Src/CAN_Control.d \
./CAN/Src/CAN_FuncHandle.d 


# Each subdirectory must supply rules for building sources it contributes
CAN/Src/%.o CAN/Src/%.su CAN/Src/%.cyclo: ../CAN/Src/%.c CAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/Admin/Desktop/Robocon2024/Actuator_GetShootBall/CAN/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-CAN-2f-Src

clean-CAN-2f-Src:
	-$(RM) ./CAN/Src/CAN_Control.cyclo ./CAN/Src/CAN_Control.d ./CAN/Src/CAN_Control.o ./CAN/Src/CAN_Control.su ./CAN/Src/CAN_FuncHandle.cyclo ./CAN/Src/CAN_FuncHandle.d ./CAN/Src/CAN_FuncHandle.o ./CAN/Src/CAN_FuncHandle.su

.PHONY: clean-CAN-2f-Src

