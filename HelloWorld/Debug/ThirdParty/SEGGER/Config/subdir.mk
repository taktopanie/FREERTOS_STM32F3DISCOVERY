################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c 

OBJS += \
./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o 

C_DEPS += \
./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/Config/%.o ThirdParty/SEGGER/Config/%.su ThirdParty/SEGGER/Config/%.cyclo: ../ThirdParty/SEGGER/Config/%.c ThirdParty/SEGGER/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/REC" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/Config" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/OS" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/SEGGER/SEGGER" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/FreeRTOS" -I"/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/HelloWorld/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-Config

clean-ThirdParty-2f-SEGGER-2f-Config:
	-$(RM) ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.cyclo ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-Config

