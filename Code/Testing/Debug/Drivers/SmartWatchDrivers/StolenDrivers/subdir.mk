################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SmartWatchDrivers/StolenDrivers/GNSS.c \
../Drivers/SmartWatchDrivers/StolenDrivers/bmp280.c \
../Drivers/SmartWatchDrivers/StolenDrivers/fonts.c \
../Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.c \
../Drivers/SmartWatchDrivers/StolenDrivers/st7789.c 

OBJS += \
./Drivers/SmartWatchDrivers/StolenDrivers/GNSS.o \
./Drivers/SmartWatchDrivers/StolenDrivers/bmp280.o \
./Drivers/SmartWatchDrivers/StolenDrivers/fonts.o \
./Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.o \
./Drivers/SmartWatchDrivers/StolenDrivers/st7789.o 

C_DEPS += \
./Drivers/SmartWatchDrivers/StolenDrivers/GNSS.d \
./Drivers/SmartWatchDrivers/StolenDrivers/bmp280.d \
./Drivers/SmartWatchDrivers/StolenDrivers/fonts.d \
./Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.d \
./Drivers/SmartWatchDrivers/StolenDrivers/st7789.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SmartWatchDrivers/StolenDrivers/%.o Drivers/SmartWatchDrivers/StolenDrivers/%.su Drivers/SmartWatchDrivers/StolenDrivers/%.cyclo: ../Drivers/SmartWatchDrivers/StolenDrivers/%.c Drivers/SmartWatchDrivers/StolenDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_DIRECT_SMPS_SUPPLY -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/exysta/Desktop/work/oulu/Embeded_System_Project/SmartWatchProject/Code/Testing/Drivers/SmartWatchDrivers/CustomDrivers" -I"C:/Users/exysta/Desktop/work/oulu/Embeded_System_Project/SmartWatchProject/Code/Testing/Drivers/SmartWatchDrivers/StolenDrivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SmartWatchDrivers-2f-StolenDrivers

clean-Drivers-2f-SmartWatchDrivers-2f-StolenDrivers:
	-$(RM) ./Drivers/SmartWatchDrivers/StolenDrivers/GNSS.cyclo ./Drivers/SmartWatchDrivers/StolenDrivers/GNSS.d ./Drivers/SmartWatchDrivers/StolenDrivers/GNSS.o ./Drivers/SmartWatchDrivers/StolenDrivers/GNSS.su ./Drivers/SmartWatchDrivers/StolenDrivers/bmp280.cyclo ./Drivers/SmartWatchDrivers/StolenDrivers/bmp280.d ./Drivers/SmartWatchDrivers/StolenDrivers/bmp280.o ./Drivers/SmartWatchDrivers/StolenDrivers/bmp280.su ./Drivers/SmartWatchDrivers/StolenDrivers/fonts.cyclo ./Drivers/SmartWatchDrivers/StolenDrivers/fonts.d ./Drivers/SmartWatchDrivers/StolenDrivers/fonts.o ./Drivers/SmartWatchDrivers/StolenDrivers/fonts.su ./Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.cyclo ./Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.d ./Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.o ./Drivers/SmartWatchDrivers/StolenDrivers/max30102_for_stm32_hal.su ./Drivers/SmartWatchDrivers/StolenDrivers/st7789.cyclo ./Drivers/SmartWatchDrivers/StolenDrivers/st7789.d ./Drivers/SmartWatchDrivers/StolenDrivers/st7789.o ./Drivers/SmartWatchDrivers/StolenDrivers/st7789.su

.PHONY: clean-Drivers-2f-SmartWatchDrivers-2f-StolenDrivers

