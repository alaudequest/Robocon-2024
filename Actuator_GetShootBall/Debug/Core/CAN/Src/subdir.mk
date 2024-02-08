################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/CAN/Src/CAN_Control.c \
../Core/CAN/Src/CAN_FuncHandle.c 

OBJS += \
./Core/CAN/Src/CAN_Control.o \
./Core/CAN/Src/CAN_FuncHandle.o 

C_DEPS += \
./Core/CAN/Src/CAN_Control.d \
./Core/CAN/Src/CAN_FuncHandle.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CAN/Src/%.o Core/CAN/Src/%.su Core/CAN/Src/%.cyclo: ../Core/CAN/Src/%.c Core/CAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-CAN-2f-Src

clean-Core-2f-CAN-2f-Src:
	-$(RM) ./Core/CAN/Src/CAN_Control.cyclo ./Core/CAN/Src/CAN_Control.d ./Core/CAN/Src/CAN_Control.o ./Core/CAN/Src/CAN_Control.su ./Core/CAN/Src/CAN_FuncHandle.cyclo ./Core/CAN/Src/CAN_FuncHandle.d ./Core/CAN/Src/CAN_FuncHandle.o ./Core/CAN/Src/CAN_FuncHandle.su

.PHONY: clean-Core-2f-CAN-2f-Src

