################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USBPD/Target/usbpd_dpm_user.c \
../USBPD/Target/usbpd_pwr_user.c \
../USBPD/Target/usbpd_vdm_user.c 

OBJS += \
./USBPD/Target/usbpd_dpm_user.o \
./USBPD/Target/usbpd_pwr_user.o \
./USBPD/Target/usbpd_vdm_user.o 

C_DEPS += \
./USBPD/Target/usbpd_dpm_user.d \
./USBPD/Target/usbpd_pwr_user.d \
./USBPD/Target/usbpd_vdm_user.d 


# Each subdirectory must supply rules for building sources it contributes
USBPD/Target/%.o USBPD/Target/%.su USBPD/Target/%.cyclo: ../USBPD/Target/%.c USBPD/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -D_TRACE -D_GUI_INTERFACE -DUSBPDCORE_LIB_PD3_FULL -DUSE_HAL_DRIVER -DSTM32G0B1xx '-DCMSIS_device_header=<stm32g0xx.h>' -c -I../Core/Inc -I../USB_Device/Target -I../USB_Device/App -I../USBPD -I../USBPD/App -I../USBPD/Target -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Utilities/GUI_INTERFACE -I../Utilities/TRACER_EMB -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Application -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-USBPD-2f-Target

clean-USBPD-2f-Target:
	-$(RM) ./USBPD/Target/usbpd_dpm_user.cyclo ./USBPD/Target/usbpd_dpm_user.d ./USBPD/Target/usbpd_dpm_user.o ./USBPD/Target/usbpd_dpm_user.su ./USBPD/Target/usbpd_pwr_user.cyclo ./USBPD/Target/usbpd_pwr_user.d ./USBPD/Target/usbpd_pwr_user.o ./USBPD/Target/usbpd_pwr_user.su ./USBPD/Target/usbpd_vdm_user.cyclo ./USBPD/Target/usbpd_vdm_user.d ./USBPD/Target/usbpd_vdm_user.o ./USBPD/Target/usbpd_vdm_user.su

.PHONY: clean-USBPD-2f-Target

