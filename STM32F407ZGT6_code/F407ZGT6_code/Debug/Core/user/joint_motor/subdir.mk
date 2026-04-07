################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/user/joint_motor/comm_can.c \
../Core/user/joint_motor/joint_motor.c 

C_DEPS += \
./Core/user/joint_motor/comm_can.d \
./Core/user/joint_motor/joint_motor.d 

OBJS += \
./Core/user/joint_motor/comm_can.o \
./Core/user/joint_motor/joint_motor.o 


# Each subdirectory must supply rules for building sources it contributes
Core/user/joint_motor/%.o Core/user/joint_motor/%.su Core/user/joint_motor/%.cyclo: ../Core/user/joint_motor/%.c Core/user/joint_motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-user-2f-joint_motor

clean-Core-2f-user-2f-joint_motor:
	-$(RM) ./Core/user/joint_motor/comm_can.cyclo ./Core/user/joint_motor/comm_can.d ./Core/user/joint_motor/comm_can.o ./Core/user/joint_motor/comm_can.su ./Core/user/joint_motor/joint_motor.cyclo ./Core/user/joint_motor/joint_motor.d ./Core/user/joint_motor/joint_motor.o ./Core/user/joint_motor/joint_motor.su

.PHONY: clean-Core-2f-user-2f-joint_motor

