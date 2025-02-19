# light_weight_telemetry_system
A lightweight, real-time telemetry system for drones using an STM32 microcontroller and LoRa modules. Transmits GPS coordinates, altitude, and temperature over **500m** with **&lt;5% packet loss**. Ideal for UAV monitoring and data logging.
# Drone Telemetry System with STM32 and LoRa


[Drone Battery (5V)] → [AMS1117 3.3V LDO]  
                        │  
                        ├── [STM32F411]  
                        │      ├── [LoRa (SX1276)] via SPI  
                        │      ├── [GPS (NEO-M8N)] via UART  
                        │      └── [BMP280] via I2C  
                        ├── [LoRa Antenna]  
                        └── [Sensors]

                        
## Features
- **Real-Time Data**: GPS (latitude/longitude), barometric altitude, and temperature.
- **Long Range**: 500m+ line-of-sight communication using LoRa.
- **Low Power**: STM32 sleep modes and LoRa duty cycling.
- **Lightweight**: Total system weight <50g.
- **<5% Packet Loss**: Optimized LoRa settings with CRC checksums.

---

## Hardware Components
| Component               | Model/Specification                     |
|-------------------------|-----------------------------------------|
| Microcontroller         | STM32F411CEU6                           |
| LoRa Module             | SX1276 (HopeRF RFM95W)                  |
| GPS Module              | u-blox NEO-M8N                          |
| Altitude/Temp Sensor    | BMP280                                  |
| Voltage Regulator       | AMS1117 3.3V LDO                        |
| Antenna                 | 868/915 MHz Helical Antenna             |

---

## Wiring Diagram
| Component | STM32 Pin | Connection Details       |
|-----------|-----------|--------------------------|
| LoRa MOSI | PA7       | SPI1_MOSI                |
| LoRa MISO | PA6       | SPI1_MISO                |
| LoRa SCK  | PA5       | SPI1_SCK                 |
| LoRa NSS  | PA4       | GPIO (Chip Select)       |
| LoRa RESET| PB0       | GPIO (Reset)             |
| LoRa DIO0 | PB1       | GPIO (Interrupt)         |
| GPS TX    | PA3       | USART2_RX                |
| GPS RX    | PA2       | USART2_TX (Optional)     |
| BMP280 SDA| PB7       | I2C1_SDA                 |
| BMP280 SCL| PB6       | I2C1_SCL                 |

---

## Software Setup
1. **IDE**: Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. **Libraries**:
   - [RadioLib](https://github.com/jgromes/RadioLib) for LoRa communication.
   - [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) for GPS parsing.
   - STM32 HAL drivers (auto-generated by STM32CubeMX).
3. **Code Upload**:
   - Clone this repository.
   - Open the project in STM32CubeIDE.
   - Build and flash to the STM32.

---

## Hardware Assembly
1. Solder components to a perfboard or custom PCB.
2. Connect sensors and LoRa module to the STM32 as per the [wiring table](#wiring-diagram).
3. Power the system from the drone’s battery via the 3.3V LDO regulator.
4. Mount the antenna away from carbon fiber parts and motors.

---

## Usage
1. **Transmitter (Drone)**:
   - The STM32 reads sensor data and transmits packets via LoRa at 1 Hz.
   - Packet format: `[latitude, longitude, altitude (m), temperature (°C)]`.
2. **Receiver (Ground Station)**:
   - Use another LoRa module connected to an STM32
   - Decode packets and display data using a serial monitor.

 ---
