################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libs/UGUI/ugui.c \
../libs/UGUI/ugui_button.c \
../libs/UGUI/ugui_checkbox.c \
../libs/UGUI/ugui_image.c \
../libs/UGUI/ugui_progress.c \
../libs/UGUI/ugui_sim.c \
../libs/UGUI/ugui_sim_x11.c \
../libs/UGUI/ugui_textbox.c 

OBJS += \
./libs/UGUI/ugui.o \
./libs/UGUI/ugui_button.o \
./libs/UGUI/ugui_checkbox.o \
./libs/UGUI/ugui_image.o \
./libs/UGUI/ugui_progress.o \
./libs/UGUI/ugui_sim.o \
./libs/UGUI/ugui_sim_x11.o \
./libs/UGUI/ugui_textbox.o 

C_DEPS += \
./libs/UGUI/ugui.d \
./libs/UGUI/ugui_button.d \
./libs/UGUI/ugui_checkbox.d \
./libs/UGUI/ugui_image.d \
./libs/UGUI/ugui_progress.d \
./libs/UGUI/ugui_sim.d \
./libs/UGUI/ugui_sim_x11.d \
./libs/UGUI/ugui_textbox.d 


# Each subdirectory must supply rules for building sources it contributes
libs/UGUI/%.o libs/UGUI/%.su libs/UGUI/%.cyclo: ../libs/UGUI/%.c libs/UGUI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_DIRECT_SMPS_SUPPLY -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/exysta/Desktop/work/oulu/Embeded_System_Project/SmartWatchProject/Code/Testing/libs/UGUI" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-libs-2f-UGUI

clean-libs-2f-UGUI:
	-$(RM) ./libs/UGUI/ugui.cyclo ./libs/UGUI/ugui.d ./libs/UGUI/ugui.o ./libs/UGUI/ugui.su ./libs/UGUI/ugui_button.cyclo ./libs/UGUI/ugui_button.d ./libs/UGUI/ugui_button.o ./libs/UGUI/ugui_button.su ./libs/UGUI/ugui_checkbox.cyclo ./libs/UGUI/ugui_checkbox.d ./libs/UGUI/ugui_checkbox.o ./libs/UGUI/ugui_checkbox.su ./libs/UGUI/ugui_image.cyclo ./libs/UGUI/ugui_image.d ./libs/UGUI/ugui_image.o ./libs/UGUI/ugui_image.su ./libs/UGUI/ugui_progress.cyclo ./libs/UGUI/ugui_progress.d ./libs/UGUI/ugui_progress.o ./libs/UGUI/ugui_progress.su ./libs/UGUI/ugui_sim.cyclo ./libs/UGUI/ugui_sim.d ./libs/UGUI/ugui_sim.o ./libs/UGUI/ugui_sim.su ./libs/UGUI/ugui_sim_x11.cyclo ./libs/UGUI/ugui_sim_x11.d ./libs/UGUI/ugui_sim_x11.o ./libs/UGUI/ugui_sim_x11.su ./libs/UGUI/ugui_textbox.cyclo ./libs/UGUI/ugui_textbox.d ./libs/UGUI/ugui_textbox.o ./libs/UGUI/ugui_textbox.su

.PHONY: clean-libs-2f-UGUI

