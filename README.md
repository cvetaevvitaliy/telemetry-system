# Telemetry System
This is a project for transmitting telemetry for a balloon, rocket and other things that can fly high.

------

The project consists of:

- Ground station (RX)
-  Transmitting module (TX)

------



## Ground station (RX)

It is a module that includes one PCB:

- LoRa transceiver (E19-868M30S)  - 1W power output
- MCU STM32F405
- OLED display
- Lipo charging chip
- Save logging information to MicroSD card

The display is only needed to display service information, all telemetry is transmitted to a PC via USB to display telemetry in a GUI application, transmission of control commands for the "air device"

------



## Transmitting module (TX)

This is a module that includes separate PCB modules for scalability:

- TX module LoRa transceiver (E19-868M30S)  - 1W power output ) and MCU, logging data to MicroSD card
- GPS module and connectors 
- IMU module ( acc, gyro, height (pressure) sensor )
- "Servo module" - to control flight stabilization
- Power module - for power supply of all modules

------

*the document will be updated as the project develops