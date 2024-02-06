################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_cad_hw_if.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_hw.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_hw_if_it.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_phy.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_phy_hw_if.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_pwr_hw_if.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_timersserver.c \
C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.c 

OBJS += \
./Middlewares/USBPD/usbpd_cad_hw_if.o \
./Middlewares/USBPD/usbpd_hw.o \
./Middlewares/USBPD/usbpd_hw_if_it.o \
./Middlewares/USBPD/usbpd_phy.o \
./Middlewares/USBPD/usbpd_phy_hw_if.o \
./Middlewares/USBPD/usbpd_pwr_hw_if.o \
./Middlewares/USBPD/usbpd_timersserver.o \
./Middlewares/USBPD/usbpd_trace.o 

C_DEPS += \
./Middlewares/USBPD/usbpd_cad_hw_if.d \
./Middlewares/USBPD/usbpd_hw.d \
./Middlewares/USBPD/usbpd_hw_if_it.d \
./Middlewares/USBPD/usbpd_phy.d \
./Middlewares/USBPD/usbpd_phy_hw_if.d \
./Middlewares/USBPD/usbpd_pwr_hw_if.d \
./Middlewares/USBPD/usbpd_timersserver.d \
./Middlewares/USBPD/usbpd_trace.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USBPD/usbpd_cad_hw_if.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_cad_hw_if.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_hw.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_hw.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_hw_if_it.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_hw_if_it.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_phy.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_phy.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_phy_hw_if.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_phy_hw_if.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_pwr_hw_if.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_pwr_hw_if.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_timersserver.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/src/usbpd_timersserver.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/USBPD/usbpd_trace.o: C:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.c Middlewares/USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/GUI_INTERFACE -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Utilities/TRACER_EMB -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Core/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/ST/STM32CubeMX/Repository/STM32Cube_FW_G0_V1.6.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-USBPD

clean-Middlewares-2f-USBPD:
	-$(RM) ./Middlewares/USBPD/usbpd_cad_hw_if.cyclo ./Middlewares/USBPD/usbpd_cad_hw_if.d ./Middlewares/USBPD/usbpd_cad_hw_if.o ./Middlewares/USBPD/usbpd_cad_hw_if.su ./Middlewares/USBPD/usbpd_hw.cyclo ./Middlewares/USBPD/usbpd_hw.d ./Middlewares/USBPD/usbpd_hw.o ./Middlewares/USBPD/usbpd_hw.su ./Middlewares/USBPD/usbpd_hw_if_it.cyclo ./Middlewares/USBPD/usbpd_hw_if_it.d ./Middlewares/USBPD/usbpd_hw_if_it.o ./Middlewares/USBPD/usbpd_hw_if_it.su ./Middlewares/USBPD/usbpd_phy.cyclo ./Middlewares/USBPD/usbpd_phy.d ./Middlewares/USBPD/usbpd_phy.o ./Middlewares/USBPD/usbpd_phy.su ./Middlewares/USBPD/usbpd_phy_hw_if.cyclo ./Middlewares/USBPD/usbpd_phy_hw_if.d ./Middlewares/USBPD/usbpd_phy_hw_if.o ./Middlewares/USBPD/usbpd_phy_hw_if.su ./Middlewares/USBPD/usbpd_pwr_hw_if.cyclo ./Middlewares/USBPD/usbpd_pwr_hw_if.d ./Middlewares/USBPD/usbpd_pwr_hw_if.o ./Middlewares/USBPD/usbpd_pwr_hw_if.su ./Middlewares/USBPD/usbpd_timersserver.cyclo ./Middlewares/USBPD/usbpd_timersserver.d ./Middlewares/USBPD/usbpd_timersserver.o ./Middlewares/USBPD/usbpd_timersserver.su ./Middlewares/USBPD/usbpd_trace.cyclo ./Middlewares/USBPD/usbpd_trace.d ./Middlewares/USBPD/usbpd_trace.o ./Middlewares/USBPD/usbpd_trace.su

.PHONY: clean-Middlewares-2f-USBPD

