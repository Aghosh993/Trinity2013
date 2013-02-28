################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Robot_SRC/encoder.c 

OBJS += \
./src/Robot_SRC/encoder.o 

C_DEPS += \
./src/Robot_SRC/encoder.d 


# Each subdirectory must supply rules for building sources it contributes
src/Robot_SRC/%.o: ../src/Robot_SRC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -I/home/aghosh01/Embedded/stm32_dev_wkspace/STM32F3x/inc -I/home/aghosh01/Embedded/stm32_dev_wkspace/STM32F3x/src/Robot_SRC -I/home/aghosh01/Embedded/stm32_dev_wkspace/STM32F3x/inc/Robot -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


