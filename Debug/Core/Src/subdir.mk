################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/control.c \
../Core/Src/control_receive.c \
../Core/Src/define_printf.c \
../Core/Src/dynamics.c \
../Core/Src/esc.c \
../Core/Src/imu.c \
../Core/Src/kinematics.c \
../Core/Src/main.c \
../Core/Src/math_utils.c \
../Core/Src/servo.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/timer.c \
../Core/Src/tuning_receive.c \
../Core/Src/uart_utils.c 

OBJS += \
./Core/Src/control.o \
./Core/Src/control_receive.o \
./Core/Src/define_printf.o \
./Core/Src/dynamics.o \
./Core/Src/esc.o \
./Core/Src/imu.o \
./Core/Src/kinematics.o \
./Core/Src/main.o \
./Core/Src/math_utils.o \
./Core/Src/servo.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/timer.o \
./Core/Src/tuning_receive.o \
./Core/Src/uart_utils.o 

C_DEPS += \
./Core/Src/control.d \
./Core/Src/control_receive.d \
./Core/Src/define_printf.d \
./Core/Src/dynamics.d \
./Core/Src/esc.d \
./Core/Src/imu.d \
./Core/Src/kinematics.d \
./Core/Src/main.d \
./Core/Src/math_utils.d \
./Core/Src/servo.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/timer.d \
./Core/Src/tuning_receive.d \
./Core/Src/uart_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/control_receive.cyclo ./Core/Src/control_receive.d ./Core/Src/control_receive.o ./Core/Src/control_receive.su ./Core/Src/define_printf.cyclo ./Core/Src/define_printf.d ./Core/Src/define_printf.o ./Core/Src/define_printf.su ./Core/Src/dynamics.cyclo ./Core/Src/dynamics.d ./Core/Src/dynamics.o ./Core/Src/dynamics.su ./Core/Src/esc.cyclo ./Core/Src/esc.d ./Core/Src/esc.o ./Core/Src/esc.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/kinematics.cyclo ./Core/Src/kinematics.d ./Core/Src/kinematics.o ./Core/Src/kinematics.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/math_utils.cyclo ./Core/Src/math_utils.d ./Core/Src/math_utils.o ./Core/Src/math_utils.su ./Core/Src/servo.cyclo ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/timer.cyclo ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/timer.su ./Core/Src/tuning_receive.cyclo ./Core/Src/tuning_receive.d ./Core/Src/tuning_receive.o ./Core/Src/tuning_receive.su ./Core/Src/uart_utils.cyclo ./Core/Src/uart_utils.d ./Core/Src/uart_utils.o ./Core/Src/uart_utils.su

.PHONY: clean-Core-2f-Src

