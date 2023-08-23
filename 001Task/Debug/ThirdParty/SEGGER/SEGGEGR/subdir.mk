################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT.c \
../ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_printf.c \
../ThirdParty/SEGGER/SEGGEGR/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT.o \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_ASM_ARMv7M.o \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_printf.o \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT.d \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_printf.d \
./ThirdParty/SEGGER/SEGGEGR/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/SEGGEGR/%.o ThirdParty/SEGGER/SEGGEGR/%.su: ../ThirdParty/SEGGER/SEGGEGR/%.c ThirdParty/SEGGER/SEGGEGR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/OS" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/SEGGEGR" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/SEGGER/SEGGEGR/%.o: ../ThirdParty/SEGGER/SEGGEGR/%.S ThirdParty/SEGGER/SEGGEGR/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-ThirdParty-2f-SEGGER-2f-SEGGEGR

clean-ThirdParty-2f-SEGGER-2f-SEGGEGR:
	-$(RM) ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT.d ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT.o ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT.su ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_ASM_ARMv7M.d ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_ASM_ARMv7M.o ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_printf.d ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_printf.o ./ThirdParty/SEGGER/SEGGEGR/SEGGER_RTT_printf.su ./ThirdParty/SEGGER/SEGGEGR/SEGGER_SYSVIEW.d ./ThirdParty/SEGGER/SEGGEGR/SEGGER_SYSVIEW.o ./ThirdParty/SEGGER/SEGGEGR/SEGGER_SYSVIEW.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-SEGGEGR

