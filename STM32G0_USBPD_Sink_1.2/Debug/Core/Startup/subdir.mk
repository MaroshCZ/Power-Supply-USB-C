################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g0b1retx.s 

OBJS += \
./Core/Startup/startup_stm32g0b1retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g0b1retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -I"C:/ST/STM32_CubeIDE/workspace1.14/UCPD/STM32G0_USBPD_Sink_1.2/USB_Device" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g0b1retx.d ./Core/Startup/startup_stm32g0b1retx.o

.PHONY: clean-Core-2f-Startup

