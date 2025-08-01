################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/IR_driver.c \
../Core/Src/Motor_Encoder.c \
../Core/Src/lpf.c \
../Core/Src/lsm6dsox.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/queues.c \
../Core/Src/solver.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/IR_driver.o \
./Core/Src/Motor_Encoder.o \
./Core/Src/lpf.o \
./Core/Src/lsm6dsox.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/queues.o \
./Core/Src/solver.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/IR_driver.d \
./Core/Src/Motor_Encoder.d \
./Core/Src/lpf.d \
./Core/Src/lsm6dsox.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/queues.d \
./Core/Src/solver.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/IR_driver.cyclo ./Core/Src/IR_driver.d ./Core/Src/IR_driver.o ./Core/Src/IR_driver.su ./Core/Src/Motor_Encoder.cyclo ./Core/Src/Motor_Encoder.d ./Core/Src/Motor_Encoder.o ./Core/Src/Motor_Encoder.su ./Core/Src/lpf.cyclo ./Core/Src/lpf.d ./Core/Src/lpf.o ./Core/Src/lpf.su ./Core/Src/lsm6dsox.cyclo ./Core/Src/lsm6dsox.d ./Core/Src/lsm6dsox.o ./Core/Src/lsm6dsox.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/queues.cyclo ./Core/Src/queues.d ./Core/Src/queues.o ./Core/Src/queues.su ./Core/Src/solver.cyclo ./Core/Src/solver.d ./Core/Src/solver.o ./Core/Src/solver.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

