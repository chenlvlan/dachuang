################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/user/sys/app.c \
../Core/user/sys/cli.c \
../Core/user/sys/compute.c 

C_DEPS += \
./Core/user/sys/app.d \
./Core/user/sys/cli.d \
./Core/user/sys/compute.d 

OBJS += \
./Core/user/sys/app.o \
./Core/user/sys/cli.o \
./Core/user/sys/compute.o 


# Each subdirectory must supply rules for building sources it contributes
Core/user/sys/%.o Core/user/sys/%.su Core/user/sys/%.cyclo: ../Core/user/sys/%.c Core/user/sys/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-user-2f-sys

clean-Core-2f-user-2f-sys:
	-$(RM) ./Core/user/sys/app.cyclo ./Core/user/sys/app.d ./Core/user/sys/app.o ./Core/user/sys/app.su ./Core/user/sys/cli.cyclo ./Core/user/sys/cli.d ./Core/user/sys/cli.o ./Core/user/sys/cli.su ./Core/user/sys/compute.cyclo ./Core/user/sys/compute.d ./Core/user/sys/compute.o ./Core/user/sys/compute.su

.PHONY: clean-Core-2f-user-2f-sys

