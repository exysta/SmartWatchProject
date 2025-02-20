# SmartWatchProject

## Description
This project made by involves the design and implementation of a prototype smartwatch for an embedded systems course. The smartwatch focuses on health monitoring, environmental sensing, and real-time data visualization. It includes features such as heart rate monitoring, SpO2 measurement, motion tracking, GPS, Bluetooth connectivity, and a user interface via an LCD screen. The device is designed to be low-power and cost-effective.

## Key Features
- **Health Monitoring**: Heart rate, SpO2 (blood oxygen saturation), and motion tracking.
- **Environmental Sensors**: Temperature, humidity, and atmospheric pressure.
- **Connectivity**: Bluetooth Low Energy (BLE) for smartphone integration and GPS for location tracking.
- **Display**: 1.9" TFT LCD screen for real-time data visualization.
- **Power Management**: Rechargeable 3.7V LiPo battery with USB-C charging.
- **User Interaction**: Control buttons for navigation and a reset button.

## Hardware Components
- **Microcontroller**: STM32L476RCT6 (ARM Cortex-M4, 80 MHz, 1MB Flash).
- **Sensors**:
  - MAX30102 (Heart rate/SpO2)
  - MPU-6500 (3-axis accelerometer/gyroscope)
  - BMP280 (Pressure, temperature, humidity)
  - NEO-7M GPS module
- **Connectivity**: AT-09 BLE module (CC2541 chip).
- **Display**: ST7789 SPI-driven TFT LCD.
- **Power**: LM317 voltage regulators, 3.7V 1500mAh LiPo battery.
- **Miscellaneous**: Status LEDs, push buttons, SWD debug connector.

## Software Architecture
- **APIs**: LED, Bluetooth, Button, Sensors, SPI/I2C drivers.
- **Main Loop**:
  - Periodic sensor data acquisition.
  - LCD refresh and BLE data transmission.
  - GPS data parsing.
- **Firmware**: Developed in C using STM32CubeIDE.

## Installation & Setup
1. **Hardware Assembly**:
   - Solder components onto the 4-layer PCB (top layer for components, GND/power planes).
   - Connect sensors, display, and modules as per the block diagram (Page 9).
2. **Programming**:
   - Use ST-LINK/V2 programmer via SWD connector (PA13/SWDIO, PA14/SWDCK).
   - Flash firmware using STM32CubeIDE.
3. **Power Up**:
   - Charge the LiPo battery via USB-C.
   - Toggle the A11JP power switch to start the device.

## Usage
1. **Navigation**:
   - Use **PA0** (Back) and **PA1** (Next) buttons to cycle through LCD screens.
2. **Data Transmission**:
   - Pair the device via BLE (AT-09 module) to view sensor data on a companion app.
3. **Reset**:
   - Press the reset button (NRST pin) to restart the system.

## Testing Summary
- **Hardware**: Voltage stability, sensor calibration, PCB integrity.
- **Software**: Peripheral communication (I2C, SPI, UART), interrupt handling.
- **Integration**: End-to-end data flow, BLE/GPS coexistence, stress testing.

## Cost
- **Total Prototype Cost**: ~€90.
- **Major Components**:
  - STM32 MCU (€6)
  - Sensors (€3–€6 each)
  - LCD (€6)
  - Battery (€6)

## Contributors
- **Mathéo Morin**: Project management, documentation.
- **Lucas Chapart**: Hardware design, PCB layout.
- **Mayane Guillon**: Firmware development, testing.

## License
Open-source (MIT License).

## References
- [STM32L476RCT6 Datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32/476rc.html)
- Course: Embedded System Project (University of Oulu).
