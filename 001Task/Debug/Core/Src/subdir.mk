################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_hal_timebase_tim.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f3xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_hal_timebase_tim.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_hal_timebase_tim.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -DconfigSUPPORT_DYNAMIC_ALLOCATION -c -I../Core/Inc -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/OS" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/SEGGEGR" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/OS" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/SEGGEGR" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS" -I"/home/maciej/Desktop/moje/projekty/STM32/proj/FREERTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_hal_timebase_tim.d ./Core/Src/stm32f3xx_hal_timebase_tim.o ./Core/Src/stm32f3xx_hal_timebase_tim.su ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su

.PHONY: clean-Core-2f-Src

