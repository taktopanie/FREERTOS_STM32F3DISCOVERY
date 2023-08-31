################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/Core/Startup/startup_stm32f303vctx.s 

OBJS += \
./Drivers/Core/Startup/startup_stm32f303vctx.o 

S_DEPS += \
./Drivers/Core/Startup/startup_stm32f303vctx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Core/Startup/%.o: ../Drivers/Core/Startup/%.s Drivers/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/home/maciej/Desktop/moje/projekty/STM32/Projekty/RTOS/MasteringRTOS/RTOS_workspace/001Task/ThirdParty/SEGGER/Config" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Drivers-2f-Core-2f-Startup

clean-Drivers-2f-Core-2f-Startup:
	-$(RM) ./Drivers/Core/Startup/startup_stm32f303vctx.d ./Drivers/Core/Startup/startup_stm32f303vctx.o

.PHONY: clean-Drivers-2f-Core-2f-Startup

