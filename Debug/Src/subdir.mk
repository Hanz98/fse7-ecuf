################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ECUF_functions.c \
../Src/ECUF_main.c \
../Src/can.c \
../Src/canFun.c \
../Src/can_ECUF.c \
../Src/can_eforce_init.c \
../Src/main.c \
../Src/ringbuf.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/ECUF_functions.o \
./Src/ECUF_main.o \
./Src/can.o \
./Src/canFun.o \
./Src/can_ECUF.o \
./Src/can_eforce_init.o \
./Src/main.o \
./Src/ringbuf.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/ECUF_functions.d \
./Src/ECUF_main.d \
./Src/can.d \
./Src/canFun.d \
./Src/can_ECUF.d \
./Src/can_eforce_init.d \
./Src/main.d \
./Src/ringbuf.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-DTX_WITH_CANDB=1' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F105xC -I"/home/Hanz/Dokumenty/front/front/Inc" -I"/home/Hanz/Dokumenty/front/front/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/Hanz/Dokumenty/front/front/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/Hanz/Dokumenty/front/front/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/Hanz/Dokumenty/front/front/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


