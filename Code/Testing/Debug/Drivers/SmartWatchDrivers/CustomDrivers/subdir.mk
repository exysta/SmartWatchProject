################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.c \
../Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.c \
../Drivers/SmartWatchDrivers/CustomDrivers/display.c \
../Drivers/SmartWatchDrivers/CustomDrivers/input.c \
../Drivers/SmartWatchDrivers/CustomDrivers/sensors.c \
../Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.c 

OBJS += \
./Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.o \
./Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.o \
./Drivers/SmartWatchDrivers/CustomDrivers/display.o \
./Drivers/SmartWatchDrivers/CustomDrivers/input.o \
./Drivers/SmartWatchDrivers/CustomDrivers/sensors.o \
./Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.o 

C_DEPS += \
./Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.d \
./Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.d \
./Drivers/SmartWatchDrivers/CustomDrivers/display.d \
./Drivers/SmartWatchDrivers/CustomDrivers/input.d \
./Drivers/SmartWatchDrivers/CustomDrivers/sensors.d \
./Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SmartWatchDrivers/CustomDrivers/%.o Drivers/SmartWatchDrivers/CustomDrivers/%.su Drivers/SmartWatchDrivers/CustomDrivers/%.cyclo: ../Drivers/SmartWatchDrivers/CustomDrivers/%.c Drivers/SmartWatchDrivers/CustomDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_DIRECT_SMPS_SUPPLY -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/exysta/Desktop/work/oulu/Embeded_System_Project/SmartWatchProject/Code/Testing/Drivers/SmartWatchDrivers/CustomDrivers" -I"C:/Users/exysta/Desktop/work/oulu/Embeded_System_Project/SmartWatchProject/Code/Testing/Drivers/SmartWatchDrivers/StolenDrivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SmartWatchDrivers-2f-CustomDrivers

clean-Drivers-2f-SmartWatchDrivers-2f-CustomDrivers:
	-$(RM) ./Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.cyclo ./Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.d ./Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.o ./Drivers/SmartWatchDrivers/CustomDrivers/MPU6500_driver.su ./Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.cyclo ./Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.d ./Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.o ./Drivers/SmartWatchDrivers/CustomDrivers/ble_comms.su ./Drivers/SmartWatchDrivers/CustomDrivers/display.cyclo ./Drivers/SmartWatchDrivers/CustomDrivers/display.d ./Drivers/SmartWatchDrivers/CustomDrivers/display.o ./Drivers/SmartWatchDrivers/CustomDrivers/display.su ./Drivers/SmartWatchDrivers/CustomDrivers/input.cyclo ./Drivers/SmartWatchDrivers/CustomDrivers/input.d ./Drivers/SmartWatchDrivers/CustomDrivers/input.o ./Drivers/SmartWatchDrivers/CustomDrivers/input.su ./Drivers/SmartWatchDrivers/CustomDrivers/sensors.cyclo ./Drivers/SmartWatchDrivers/CustomDrivers/sensors.d ./Drivers/SmartWatchDrivers/CustomDrivers/sensors.o ./Drivers/SmartWatchDrivers/CustomDrivers/sensors.su ./Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.cyclo ./Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.d ./Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.o ./Drivers/SmartWatchDrivers/CustomDrivers/uart_comms.su

.PHONY: clean-Drivers-2f-SmartWatchDrivers-2f-CustomDrivers

