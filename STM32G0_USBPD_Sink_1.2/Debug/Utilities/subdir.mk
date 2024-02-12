################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE/bsp_gui.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE/data_struct_tlv.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE/gui_api.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB/tracer_emb.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB/tracer_emb_hw.c 

OBJS += \
./Utilities/bsp_gui.o \
./Utilities/data_struct_tlv.o \
./Utilities/gui_api.o \
./Utilities/tracer_emb.o \
./Utilities/tracer_emb_hw.o 

C_DEPS += \
./Utilities/bsp_gui.d \
./Utilities/data_struct_tlv.d \
./Utilities/gui_api.d \
./Utilities/tracer_emb.d \
./Utilities/tracer_emb_hw.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/bsp_gui.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE/bsp_gui.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Utilities/data_struct_tlv.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE/data_struct_tlv.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Utilities/gui_api.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE/gui_api.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Utilities/tracer_emb.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB/tracer_emb.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Utilities/tracer_emb_hw.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB/tracer_emb_hw.c Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utilities

clean-Utilities:
	-$(RM) ./Utilities/bsp_gui.cyclo ./Utilities/bsp_gui.d ./Utilities/bsp_gui.o ./Utilities/bsp_gui.su ./Utilities/data_struct_tlv.cyclo ./Utilities/data_struct_tlv.d ./Utilities/data_struct_tlv.o ./Utilities/data_struct_tlv.su ./Utilities/gui_api.cyclo ./Utilities/gui_api.d ./Utilities/gui_api.o ./Utilities/gui_api.su ./Utilities/tracer_emb.cyclo ./Utilities/tracer_emb.d ./Utilities/tracer_emb.o ./Utilities/tracer_emb.su ./Utilities/tracer_emb_hw.cyclo ./Utilities/tracer_emb_hw.d ./Utilities/tracer_emb_hw.o ./Utilities/tracer_emb_hw.su

.PHONY: clean-Utilities
