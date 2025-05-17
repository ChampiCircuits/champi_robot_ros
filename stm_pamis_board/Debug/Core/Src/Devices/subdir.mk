################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Devices/SCServos.cpp 

OBJS += \
./Core/Src/Devices/SCServos.o 

CPP_DEPS += \
./Core/Src/Devices/SCServos.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Devices/%.o Core/Src/Devices/%.su Core/Src/Devices/%.cyclo: ../Core/Src/Devices/%.cpp Core/Src/Devices/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Devices

clean-Core-2f-Src-2f-Devices:
	-$(RM) ./Core/Src/Devices/SCServos.cyclo ./Core/Src/Devices/SCServos.d ./Core/Src/Devices/SCServos.o ./Core/Src/Devices/SCServos.su

.PHONY: clean-Core-2f-Src-2f-Devices

