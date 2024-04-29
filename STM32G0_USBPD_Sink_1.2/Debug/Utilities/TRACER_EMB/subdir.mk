################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/TRACER_EMB/tracer_emb.c \
../Utilities/TRACER_EMB/tracer_emb_hw.c 

OBJS += \
./Utilities/TRACER_EMB/tracer_emb.o \
./Utilities/TRACER_EMB/tracer_emb_hw.o 

C_DEPS += \
./Utilities/TRACER_EMB/tracer_emb.d \
./Utilities/TRACER_EMB/tracer_emb_hw.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/TRACER_EMB/%.o Utilities/TRACER_EMB/%.su Utilities/TRACER_EMB/%.cyclo: ../Utilities/TRACER_EMB/%.c Utilities/TRACER_EMB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx '-DCMSIS_device_header=<stm32g0xx.h>' -c -I../Core/Inc -I../USBPD -I../USBPD/App -I../USBPD/Target -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Utilities/GUI_INTERFACE -I../Utilities/TRACER_EMB -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utilities-2f-TRACER_EMB

clean-Utilities-2f-TRACER_EMB:
	-$(RM) ./Utilities/TRACER_EMB/tracer_emb.cyclo ./Utilities/TRACER_EMB/tracer_emb.d ./Utilities/TRACER_EMB/tracer_emb.o ./Utilities/TRACER_EMB/tracer_emb.su ./Utilities/TRACER_EMB/tracer_emb_hw.cyclo ./Utilities/TRACER_EMB/tracer_emb_hw.d ./Utilities/TRACER_EMB/tracer_emb_hw.o ./Utilities/TRACER_EMB/tracer_emb_hw.su

.PHONY: clean-Utilities-2f-TRACER_EMB

