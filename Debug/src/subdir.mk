################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/adc.c \
../src/extFetCtrl.c \
../src/ledPwm.c \
../src/main.c \
../src/mpu6050.c \
../src/onboardLedCtrl.c \
../src/sw.c \
../src/uart.c 

OBJS += \
./src/adc.o \
./src/extFetCtrl.o \
./src/ledPwm.o \
./src/main.o \
./src/mpu6050.o \
./src/onboardLedCtrl.o \
./src/sw.o \
./src/uart.o 

C_DEPS += \
./src/adc.d \
./src/extFetCtrl.d \
./src/ledPwm.d \
./src/main.d \
./src/mpu6050.d \
./src/onboardLedCtrl.d \
./src/sw.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I../ledSegment/include -I../ledSegment/stm32utils/include -I../ledSegment/stm32utils/system/include -I../ledSegment/stm32utils/system/include/cmsis -I../ledSegment/stm32utils/system/include/stm32f1-stdperiph -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


