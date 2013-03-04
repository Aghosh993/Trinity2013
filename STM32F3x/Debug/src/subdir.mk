################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/debug.cpp \
../src/encoder.cpp \
../src/interrupt_defs.cpp \
../src/main.cpp \
../src/pwm.cpp 

C_SRCS += \
../src/newlib_stubs.c \
../src/stm32f30x_adc.c \
../src/stm32f30x_can.c \
../src/stm32f30x_comp.c \
../src/stm32f30x_crc.c \
../src/stm32f30x_dac.c \
../src/stm32f30x_dbgmcu.c \
../src/stm32f30x_dma.c \
../src/stm32f30x_exti.c \
../src/stm32f30x_flash.c \
../src/stm32f30x_gpio.c \
../src/stm32f30x_i2c.c \
../src/stm32f30x_iwdg.c \
../src/stm32f30x_misc.c \
../src/stm32f30x_opamp.c \
../src/stm32f30x_pwr.c \
../src/stm32f30x_rcc.c \
../src/stm32f30x_rtc.c \
../src/stm32f30x_spi.c \
../src/stm32f30x_syscfg.c \
../src/stm32f30x_tim.c \
../src/stm32f30x_usart.c \
../src/stm32f30x_wwdg.c \
../src/stm32f3_discovery_l3gd20.c \
../src/system_stm32f30x.c 

S_UPPER_SRCS += \
../src/startup_stm32f30x.S 

OBJS += \
./src/debug.o \
./src/encoder.o \
./src/interrupt_defs.o \
./src/main.o \
./src/newlib_stubs.o \
./src/pwm.o \
./src/startup_stm32f30x.o \
./src/stm32f30x_adc.o \
./src/stm32f30x_can.o \
./src/stm32f30x_comp.o \
./src/stm32f30x_crc.o \
./src/stm32f30x_dac.o \
./src/stm32f30x_dbgmcu.o \
./src/stm32f30x_dma.o \
./src/stm32f30x_exti.o \
./src/stm32f30x_flash.o \
./src/stm32f30x_gpio.o \
./src/stm32f30x_i2c.o \
./src/stm32f30x_iwdg.o \
./src/stm32f30x_misc.o \
./src/stm32f30x_opamp.o \
./src/stm32f30x_pwr.o \
./src/stm32f30x_rcc.o \
./src/stm32f30x_rtc.o \
./src/stm32f30x_spi.o \
./src/stm32f30x_syscfg.o \
./src/stm32f30x_tim.o \
./src/stm32f30x_usart.o \
./src/stm32f30x_wwdg.o \
./src/stm32f3_discovery_l3gd20.o \
./src/system_stm32f30x.o 

C_DEPS += \
./src/newlib_stubs.d \
./src/stm32f30x_adc.d \
./src/stm32f30x_can.d \
./src/stm32f30x_comp.d \
./src/stm32f30x_crc.d \
./src/stm32f30x_dac.d \
./src/stm32f30x_dbgmcu.d \
./src/stm32f30x_dma.d \
./src/stm32f30x_exti.d \
./src/stm32f30x_flash.d \
./src/stm32f30x_gpio.d \
./src/stm32f30x_i2c.d \
./src/stm32f30x_iwdg.d \
./src/stm32f30x_misc.d \
./src/stm32f30x_opamp.d \
./src/stm32f30x_pwr.d \
./src/stm32f30x_rcc.d \
./src/stm32f30x_rtc.d \
./src/stm32f30x_spi.d \
./src/stm32f30x_syscfg.d \
./src/stm32f30x_tim.d \
./src/stm32f30x_usart.d \
./src/stm32f30x_wwdg.d \
./src/stm32f3_discovery_l3gd20.d \
./src/system_stm32f30x.d 

S_UPPER_DEPS += \
./src/startup_stm32f30x.d 

CPP_DEPS += \
./src/debug.d \
./src/encoder.d \
./src/interrupt_defs.d \
./src/main.d \
./src/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C++ Compiler'
	arm-none-eabi-g++ -I/home/aghosh01/git/Trinity2013/STM32F3x/inc -I/home/aghosh01/git/Trinity2013/STM32F3x/inc/Robot -O0 -Wall -Wa,-adhlns="$@.lst" -fno-exceptions -fno-rtti -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -I/home/aghosh01/git/Trinity2013/STM32F3x/inc -I/home/aghosh01/git/Trinity2013/STM32F3x/inc/Robot -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


