################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c 

OBJS += \
./common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o 

C_DEPS += \
./common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o: /home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c common/ThirdParty/SEGGER/OS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/FreeRTOS/include -I/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F -I/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/SEGGER/Config -I/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/SEGGER/OS -I/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/SEGGER/SEGGER -I/home/taktopanie/Desktop/moje/programming/STM32/RTOS_workspace/common/ThirdParty/FreeRTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-common-2f-ThirdParty-2f-SEGGER-2f-OS

clean-common-2f-ThirdParty-2f-SEGGER-2f-OS:
	-$(RM) ./common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.cyclo ./common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d ./common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o ./common/ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.su

.PHONY: clean-common-2f-ThirdParty-2f-SEGGER-2f-OS

