# SmartWatch Project

## Overview

This repository hosts the source code and technical documentation for the SmartWatch project, developed as part of an embedded systems course. The prototype is a wearable device designed for health monitoring, motion tracking, and real-time data visualization, while seamlessly communicating with a smartphone via Bluetooth Low Energy (BLE).

## Key Features

- **Health Monitoring:** Measures heart rate and blood oxygen saturation (SpO₂) using the MAX30102 sensor.
- **Motion Tracking:** Detects 3D acceleration and orientation with the MPU-6500 sensor.
- **Environmental Sensing:** Captures temperature, humidity, and atmospheric pressure using the BMP280 sensor.
- **GPS Navigation:** Provides accurate location tracking with the NEO-7M module.
- **User Interface:** Displays real-time data on a high-performance ST7789 TFT screen and allows navigation via dedicated buttons.
- **Wireless Communication:** Transmits sensor data using the AT-09 Bluetooth module.
- **Efficient Power Management:** Powered by a rechargeable LiPo battery (USB-C charging) with voltage regulation via LM317.

## Repository Structure

```
smartwatch/
├── hardware/         # Schematics, PCB design files, and hardware documentation
├── software/         # Source code, drivers, and firmware modules
├── docs/             # Detailed technical documentation, testing procedures, cost calculations, etc.
└── README.md         # This file
```

## Hardware Overview

The design centers around the STM32L476RCT6 microcontroller, ensuring low power consumption and high performance. The main hardware components include:

- **MCU:** STM32L476RCT6 (ARM Cortex-M4, 80 MHz, low-power design)
- **Bluetooth Module:** AT-09 (based on TI CC2541) for BLE connectivity
- **Sensors:**
  - MPU-6500: IMU for acceleration and gyroscopic data
  - MAX30102: For heart rate and SpO₂ measurements
  - BMP280: For temperature, humidity, and pressure readings
- **Display:** ST7789 TFT screen for real-time information display
- **GPS Module:** NEO-7M for precise location tracking
- **Power Management:** Voltage regulators (LM317) and a power switch optimize energy efficiency


## Software Architecture

The firmware is organized into modular APIs that manage:

- **LED Indicators:** Status and error signals.
- **Bluetooth Communication:** Data exchange over BLE.
- **User Input:** Button handling for reset and display navigation.
- **Sensor Data Acquisition:** Interfaces for reading data via I2C and SPI.
- **Peripheral Drivers:** SPI and I2C drivers for smooth communication.

Upon startup, the system performs a handshake with each module. Once all components are verified, the main loop continuously:

- Acquires sensor data on a periodic basis.
- Refreshes the TFT display.
- Monitors user inputs.
- Transmits sensor data via Bluetooth every second.
- Continuously updates GPS data.

## Testing Procedures

Testing covers multiple aspects of the system to ensure reliability:

- **Visual & Hardware Inspection:** Check for soldering issues, short circuits, and proper PCB assembly.
- **Power Supply Testing:** Verify voltage levels and current draw under various loads.
- **Sensor Validation:** Calibrate and compare readings for health, motion, and environmental sensors.
- **Communication Tests:** Ensure robust Bluetooth pairing and accurate GPS data reception.
- **System Integration:** Confirm end-to-end functionality, including data flow from sensors to display and wireless transmission.


## Getting Started

1. **Clone the Repository:**
   ```sh
   git clone https://github.com/your-username/smartwatch.git
   cd smartwatch
   ```
2. **Hardware Setup:**
   - Refer to the `/hardware` folder for PCB schematics and design files.
3. **Build the Firmware:**
   - Open the project in STM32CubeIDE or your preferred environment.
   - Compile the project using:
     ```sh
     make
     ```
4. **Flash the Firmware:**
   - Use an ST-LINK/V2 programmer to flash the firmware onto the STM32 MCU.
   - Follow the instructions in `/hardware/programming_guide.pdf` for detailed steps.

## Contributing

Contributions are welcome! Please review `CONTRIBUTING.md` for guidelines on how to contribute to this project.

## License

This project is licensed under the MIT License. See `LICENSE` for more information.

## Acknowledgments

- **Project Team:** Mathéo Morin, Lucas Chapart, Mayane Guillon
- **Supervisor:** Marko Kauppinen
- **Course:** Embedded Systems Project
