################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.c 

OBJS += \
./ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.o 

C_DEPS += \
./ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/SEGGEGR/Syscalls/%.o ThirdParty/SEGGER/SEGGEGR/Syscalls/%.su ThirdParty/SEGGER/SEGGEGR/Syscalls/%.cyclo: ../ThirdParty/SEGGER/SEGGEGR/Syscalls/%.c ThirdParty/SEGGER/SEGGEGR/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/REC" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/Config" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/OS" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/SEGGEGR" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/FreeRTOS" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-SEGGEGR-2f-Syscalls

clean-ThirdParty-2f-SEGGER-2f-SEGGEGR-2f-Syscalls:
	-$(RM) ./ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.cyclo ./ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./ThirdParty/SEGGER/SEGGEGR/Syscalls/SEGGER_RTT_Syscalls_GCC.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-SEGGEGR-2f-Syscalls

