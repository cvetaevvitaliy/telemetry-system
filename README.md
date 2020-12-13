# Telemetry System
This is a project for transmitting telemetry for a balloon, rocket and other things that can fly high.

------

The project consists of:

- Ground station (RX)
-  Transmitting module (TX)

------



## Ground station (RX)

#### Block diagram

![](https://github.com/cvetaevvitaliy/telemetry-system/blob/main/doc/pic/Ground-station.png)

This is a module that includes one PCB:

- LoRa transceiver (E19-868M30S)  - 1W power output
- MCU STM32F405
- OLED display
- Lipo charging chip
- Save logging information to MicroSD card

The display is only needed to display service information, all telemetry is transmitted to a PC via USB to display telemetry in a GUI application, transmission of control commands for the "air device"

------



## Transmitting module (TX)

#### Block diagram

![](https://github.com/cvetaevvitaliy/telemetry-system/blob/main/doc/pic/Telemetry-module-RX.png)

This is a module that includes separate PCB modules for scalability:

- TX module LoRa transceiver (E19-868M30S)  - 1W power output and MCU STM32F405, logging data to MicroSD card
- GPS module and connectors 
- IMU module ( acc, gyro, height (pressure) sensor )
- "Servo module" - to control flight stabilization (MCU STM32F405)
- Power module - for power supply of all modules

------

*the document will be updated as the project develops