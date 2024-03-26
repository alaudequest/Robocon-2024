################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AlgorimthControl/Src/AngleOptimizer.c \
../AlgorimthControl/Src/InverseKinematic.c \
../AlgorimthControl/Src/LowPass.c \
../AlgorimthControl/Src/Odometry.c \
../AlgorimthControl/Src/PositionControl.c \
../AlgorimthControl/Src/ProcessControl.c \
../AlgorimthControl/Src/SwerveModule.c 

OBJS += \
./AlgorimthControl/Src/AngleOptimizer.o \
./AlgorimthControl/Src/InverseKinematic.o \
./AlgorimthControl/Src/LowPass.o \
./AlgorimthControl/Src/Odometry.o \
./AlgorimthControl/Src/PositionControl.o \
./AlgorimthControl/Src/ProcessControl.o \
./AlgorimthControl/Src/SwerveModule.o 

C_DEPS += \
./AlgorimthControl/Src/AngleOptimizer.d \
./AlgorimthControl/Src/InverseKinematic.d \
./AlgorimthControl/Src/LowPass.d \
./AlgorimthControl/Src/Odometry.d \
./AlgorimthControl/Src/PositionControl.d \
./AlgorimthControl/Src/ProcessControl.d \
./AlgorimthControl/Src/SwerveModule.d 


# Each subdirectory must supply rules for building sources it contributes
AlgorimthControl/Src/%.o AlgorimthControl/Src/%.su AlgorimthControl/Src/%.cyclo: ../AlgorimthControl/Src/%.c AlgorimthControl/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"E:/LAB/Robocon-2024/Main_F407/Actuator/Inc" -I"E:/LAB/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I"E:/LAB/Robocon-2024/Main_F407/CAN/Inc" -I"E:/LAB/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I"E:/LAB/Robocon-2024/Main_F407/AlgorimthControl/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-AlgorimthControl-2f-Src

clean-AlgorimthControl-2f-Src:
	-$(RM) ./AlgorimthControl/Src/AngleOptimizer.cyclo ./AlgorimthControl/Src/AngleOptimizer.d ./AlgorimthControl/Src/AngleOptimizer.o ./AlgorimthControl/Src/AngleOptimizer.su ./AlgorimthControl/Src/InverseKinematic.cyclo ./AlgorimthControl/Src/InverseKinematic.d ./AlgorimthControl/Src/InverseKinematic.o ./AlgorimthControl/Src/InverseKinematic.su ./AlgorimthControl/Src/LowPass.cyclo ./AlgorimthControl/Src/LowPass.d ./AlgorimthControl/Src/LowPass.o ./AlgorimthControl/Src/LowPass.su ./AlgorimthControl/Src/Odometry.cyclo ./AlgorimthControl/Src/Odometry.d ./AlgorimthControl/Src/Odometry.o ./AlgorimthControl/Src/Odometry.su ./AlgorimthControl/Src/PositionControl.cyclo ./AlgorimthControl/Src/PositionControl.d ./AlgorimthControl/Src/PositionControl.o ./AlgorimthControl/Src/PositionControl.su ./AlgorimthControl/Src/ProcessControl.cyclo ./AlgorimthControl/Src/ProcessControl.d ./AlgorimthControl/Src/ProcessControl.o ./AlgorimthControl/Src/ProcessControl.su ./AlgorimthControl/Src/SwerveModule.cyclo ./AlgorimthControl/Src/SwerveModule.d ./AlgorimthControl/Src/SwerveModule.o ./AlgorimthControl/Src/SwerveModule.su

.PHONY: clean-AlgorimthControl-2f-Src

