################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AlgorimthControl/Src/AngleOptimizer.c \
../AlgorimthControl/Src/InverseKinematic.c \
../AlgorimthControl/Src/LowPass.c \
../AlgorimthControl/Src/OdometerHandle.c \
../AlgorimthControl/Src/PIDPosition.c \
../AlgorimthControl/Src/Pose2d.c \
../AlgorimthControl/Src/Rotation2d.c \
../AlgorimthControl/Src/SwerveModule.c \
../AlgorimthControl/Src/Translation2d.c 

OBJS += \
./AlgorimthControl/Src/AngleOptimizer.o \
./AlgorimthControl/Src/InverseKinematic.o \
./AlgorimthControl/Src/LowPass.o \
./AlgorimthControl/Src/OdometerHandle.o \
./AlgorimthControl/Src/PIDPosition.o \
./AlgorimthControl/Src/Pose2d.o \
./AlgorimthControl/Src/Rotation2d.o \
./AlgorimthControl/Src/SwerveModule.o \
./AlgorimthControl/Src/Translation2d.o 

C_DEPS += \
./AlgorimthControl/Src/AngleOptimizer.d \
./AlgorimthControl/Src/InverseKinematic.d \
./AlgorimthControl/Src/LowPass.d \
./AlgorimthControl/Src/OdometerHandle.d \
./AlgorimthControl/Src/PIDPosition.d \
./AlgorimthControl/Src/Pose2d.d \
./AlgorimthControl/Src/Rotation2d.d \
./AlgorimthControl/Src/SwerveModule.d \
./AlgorimthControl/Src/Translation2d.d 


# Each subdirectory must supply rules for building sources it contributes
AlgorimthControl/Src/%.o AlgorimthControl/Src/%.su AlgorimthControl/Src/%.cyclo: ../AlgorimthControl/Src/%.c AlgorimthControl/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"D:/Robocon-2024/Main_F407/Actuator/Inc" -I"D:/Robocon-2024/Main_F407/CAN/Inc" -I"D:/Robocon-2024/Main_F407/IoTVisionLibrary/Inc" -I"D:/Robocon-2024/Main_F407/AlgorimthControl/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-AlgorimthControl-2f-Src

clean-AlgorimthControl-2f-Src:
	-$(RM) ./AlgorimthControl/Src/AngleOptimizer.cyclo ./AlgorimthControl/Src/AngleOptimizer.d ./AlgorimthControl/Src/AngleOptimizer.o ./AlgorimthControl/Src/AngleOptimizer.su ./AlgorimthControl/Src/InverseKinematic.cyclo ./AlgorimthControl/Src/InverseKinematic.d ./AlgorimthControl/Src/InverseKinematic.o ./AlgorimthControl/Src/InverseKinematic.su ./AlgorimthControl/Src/LowPass.cyclo ./AlgorimthControl/Src/LowPass.d ./AlgorimthControl/Src/LowPass.o ./AlgorimthControl/Src/LowPass.su ./AlgorimthControl/Src/OdometerHandle.cyclo ./AlgorimthControl/Src/OdometerHandle.d ./AlgorimthControl/Src/OdometerHandle.o ./AlgorimthControl/Src/OdometerHandle.su ./AlgorimthControl/Src/PIDPosition.cyclo ./AlgorimthControl/Src/PIDPosition.d ./AlgorimthControl/Src/PIDPosition.o ./AlgorimthControl/Src/PIDPosition.su ./AlgorimthControl/Src/Pose2d.cyclo ./AlgorimthControl/Src/Pose2d.d ./AlgorimthControl/Src/Pose2d.o ./AlgorimthControl/Src/Pose2d.su ./AlgorimthControl/Src/Rotation2d.cyclo ./AlgorimthControl/Src/Rotation2d.d ./AlgorimthControl/Src/Rotation2d.o ./AlgorimthControl/Src/Rotation2d.su ./AlgorimthControl/Src/SwerveModule.cyclo ./AlgorimthControl/Src/SwerveModule.d ./AlgorimthControl/Src/SwerveModule.o ./AlgorimthControl/Src/SwerveModule.su ./AlgorimthControl/Src/Translation2d.cyclo ./AlgorimthControl/Src/Translation2d.d ./AlgorimthControl/Src/Translation2d.o ./AlgorimthControl/Src/Translation2d.su

.PHONY: clean-AlgorimthControl-2f-Src

