################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Core/Src/main.c \
../Drivers/Core/Src/stm32f3xx_hal_msp.c \
../Drivers/Core/Src/stm32f3xx_hal_timebase_tim.c \
../Drivers/Core/Src/stm32f3xx_it.c \
../Drivers/Core/Src/syscalls.c \
../Drivers/Core/Src/system_stm32f3xx.c 

OBJS += \
./Drivers/Core/Src/main.o \
./Drivers/Core/Src/stm32f3xx_hal_msp.o \
./Drivers/Core/Src/stm32f3xx_hal_timebase_tim.o \
./Drivers/Core/Src/stm32f3xx_it.o \
./Drivers/Core/Src/syscalls.o \
./Drivers/Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Drivers/Core/Src/main.d \
./Drivers/Core/Src/stm32f3xx_hal_msp.d \
./Drivers/Core/Src/stm32f3xx_hal_timebase_tim.d \
./Drivers/Core/Src/stm32f3xx_it.d \
./Drivers/Core/Src/syscalls.d \
./Drivers/Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Core/Src/main.o: ../Drivers/Core/Src/main.c Drivers/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -DconfigSUPPORT_DYNAMIC_ALLOCATION -c -I../Core/Inc -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/OS" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/SEGGEGR" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Core/Src/%.o Drivers/Core/Src/%.su: ../Drivers/Core/Src/%.c Drivers/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/OS" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/SEGGEGR" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS" -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Core-2f-Src

clean-Drivers-2f-Core-2f-Src:
	-$(RM) ./Drivers/Core/Src/main.d ./Drivers/Core/Src/main.o ./Drivers/Core/Src/main.su ./Drivers/Core/Src/stm32f3xx_hal_msp.d ./Drivers/Core/Src/stm32f3xx_hal_msp.o ./Drivers/Core/Src/stm32f3xx_hal_msp.su ./Drivers/Core/Src/stm32f3xx_hal_timebase_tim.d ./Drivers/Core/Src/stm32f3xx_hal_timebase_tim.o ./Drivers/Core/Src/stm32f3xx_hal_timebase_tim.su ./Drivers/Core/Src/stm32f3xx_it.d ./Drivers/Core/Src/stm32f3xx_it.o ./Drivers/Core/Src/stm32f3xx_it.su ./Drivers/Core/Src/syscalls.d ./Drivers/Core/Src/syscalls.o ./Drivers/Core/Src/syscalls.su ./Drivers/Core/Src/system_stm32f3xx.d ./Drivers/Core/Src/system_stm32f3xx.o ./Drivers/Core/Src/system_stm32f3xx.su

.PHONY: clean-Drivers-2f-Core-2f-Src

