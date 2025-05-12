################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SmartWatchDrivers/src/GNSS.c \
../Drivers/SmartWatchDrivers/src/MPU6500_driver.c \
../Drivers/SmartWatchDrivers/src/ble_comms.c \
../Drivers/SmartWatchDrivers/src/bmp280.c \
../Drivers/SmartWatchDrivers/src/display.c \
../Drivers/SmartWatchDrivers/src/fonts.c \
../Drivers/SmartWatchDrivers/src/input.c \
../Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.c \
../Drivers/SmartWatchDrivers/src/sensors.c \
../Drivers/SmartWatchDrivers/src/st7789.c 

OBJS += \
./Drivers/SmartWatchDrivers/src/GNSS.o \
./Drivers/SmartWatchDrivers/src/MPU6500_driver.o \
./Drivers/SmartWatchDrivers/src/ble_comms.o \
./Drivers/SmartWatchDrivers/src/bmp280.o \
./Drivers/SmartWatchDrivers/src/display.o \
./Drivers/SmartWatchDrivers/src/fonts.o \
./Drivers/SmartWatchDrivers/src/input.o \
./Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.o \
./Drivers/SmartWatchDrivers/src/sensors.o \
./Drivers/SmartWatchDrivers/src/st7789.o 

C_DEPS += \
./Drivers/SmartWatchDrivers/src/GNSS.d \
./Drivers/SmartWatchDrivers/src/MPU6500_driver.d \
./Drivers/SmartWatchDrivers/src/ble_comms.d \
./Drivers/SmartWatchDrivers/src/bmp280.d \
./Drivers/SmartWatchDrivers/src/display.d \
./Drivers/SmartWatchDrivers/src/fonts.d \
./Drivers/SmartWatchDrivers/src/input.d \
./Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.d \
./Drivers/SmartWatchDrivers/src/sensors.d \
./Drivers/SmartWatchDrivers/src/st7789.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SmartWatchDrivers/src/%.o Drivers/SmartWatchDrivers/src/%.su Drivers/SmartWatchDrivers/src/%.cyclo: ../Drivers/SmartWatchDrivers/src/%.c Drivers/SmartWatchDrivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/exysta/Desktop/work/oulu/Embeded_System_Project/SmartWatchProject/Code/Main/Drivers/SmartWatchDrivers/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SmartWatchDrivers-2f-src

clean-Drivers-2f-SmartWatchDrivers-2f-src:
	-$(RM) ./Drivers/SmartWatchDrivers/src/GNSS.cyclo ./Drivers/SmartWatchDrivers/src/GNSS.d ./Drivers/SmartWatchDrivers/src/GNSS.o ./Drivers/SmartWatchDrivers/src/GNSS.su ./Drivers/SmartWatchDrivers/src/MPU6500_driver.cyclo ./Drivers/SmartWatchDrivers/src/MPU6500_driver.d ./Drivers/SmartWatchDrivers/src/MPU6500_driver.o ./Drivers/SmartWatchDrivers/src/MPU6500_driver.su ./Drivers/SmartWatchDrivers/src/ble_comms.cyclo ./Drivers/SmartWatchDrivers/src/ble_comms.d ./Drivers/SmartWatchDrivers/src/ble_comms.o ./Drivers/SmartWatchDrivers/src/ble_comms.su ./Drivers/SmartWatchDrivers/src/bmp280.cyclo ./Drivers/SmartWatchDrivers/src/bmp280.d ./Drivers/SmartWatchDrivers/src/bmp280.o ./Drivers/SmartWatchDrivers/src/bmp280.su ./Drivers/SmartWatchDrivers/src/display.cyclo ./Drivers/SmartWatchDrivers/src/display.d ./Drivers/SmartWatchDrivers/src/display.o ./Drivers/SmartWatchDrivers/src/display.su ./Drivers/SmartWatchDrivers/src/fonts.cyclo ./Drivers/SmartWatchDrivers/src/fonts.d ./Drivers/SmartWatchDrivers/src/fonts.o ./Drivers/SmartWatchDrivers/src/fonts.su ./Drivers/SmartWatchDrivers/src/input.cyclo ./Drivers/SmartWatchDrivers/src/input.d ./Drivers/SmartWatchDrivers/src/input.o ./Drivers/SmartWatchDrivers/src/input.su ./Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.cyclo ./Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.d ./Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.o ./Drivers/SmartWatchDrivers/src/max30102_for_stm32_hal.su ./Drivers/SmartWatchDrivers/src/sensors.cyclo ./Drivers/SmartWatchDrivers/src/sensors.d ./Drivers/SmartWatchDrivers/src/sensors.o ./Drivers/SmartWatchDrivers/src/sensors.su ./Drivers/SmartWatchDrivers/src/st7789.cyclo ./Drivers/SmartWatchDrivers/src/st7789.d ./Drivers/SmartWatchDrivers/src/st7789.o ./Drivers/SmartWatchDrivers/src/st7789.su

.PHONY: clean-Drivers-2f-SmartWatchDrivers-2f-src

