################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/user/mpu6500/inv_mpu.c \
../Core/user/mpu6500/inv_mpu_dmp_motion_driver.c \
../Core/user/mpu6500/mpu6500.c 

C_DEPS += \
./Core/user/mpu6500/inv_mpu.d \
./Core/user/mpu6500/inv_mpu_dmp_motion_driver.d \
./Core/user/mpu6500/mpu6500.d 

OBJS += \
./Core/user/mpu6500/inv_mpu.o \
./Core/user/mpu6500/inv_mpu_dmp_motion_driver.o \
./Core/user/mpu6500/mpu6500.o 


# Each subdirectory must supply rules for building sources it contributes
Core/user/mpu6500/%.o Core/user/mpu6500/%.su Core/user/mpu6500/%.cyclo: ../Core/user/mpu6500/%.c Core/user/mpu6500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-user-2f-mpu6500

clean-Core-2f-user-2f-mpu6500:
	-$(RM) ./Core/user/mpu6500/inv_mpu.cyclo ./Core/user/mpu6500/inv_mpu.d ./Core/user/mpu6500/inv_mpu.o ./Core/user/mpu6500/inv_mpu.su ./Core/user/mpu6500/inv_mpu_dmp_motion_driver.cyclo ./Core/user/mpu6500/inv_mpu_dmp_motion_driver.d ./Core/user/mpu6500/inv_mpu_dmp_motion_driver.o ./Core/user/mpu6500/inv_mpu_dmp_motion_driver.su ./Core/user/mpu6500/mpu6500.cyclo ./Core/user/mpu6500/mpu6500.d ./Core/user/mpu6500/mpu6500.o ./Core/user/mpu6500/mpu6500.su

.PHONY: clean-Core-2f-user-2f-mpu6500

