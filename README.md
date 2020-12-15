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
- Data loger - logging of received information to MicroSD

The display is only needed to display service information, all telemetry is transmitted to a PC via USB to display telemetry in a GUI application, transmission of control commands for the rocket

For details see [WiKi - Ground station (RX)](https://github.com/cvetaevvitaliy/telemetry-system/wiki/Transmitting-module-(TX))


------



## Transmitting module (TX)

#### Block diagram

![](https://github.com/cvetaevvitaliy/telemetry-system/blob/main/doc/pic/Telemetry-module-RX.png)

This is a module that includes separate PCB modules for scalability:

- GPS module (Matek M8Q) and SMA connectors for antennas
- MCU module: LoRa transceiver (E19-868M30S)  - 1W power output, MCU STM32F405, logging data to MicroSD card (black box)
- IMU module ( acc, gyro, pressure, sensor )
- Servo module - to control flight stabilization (MCU STM32F405)
- Power module - for power supply of all modules

For details see [WiKi - Transmitting module (RX)](https://github.com/cvetaevvitaliy/telemetry-system/wiki/Transmitting-module-(TX))

------

*the document will be updated as the project develops