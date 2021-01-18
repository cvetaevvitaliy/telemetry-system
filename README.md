# Telemetry System  <div align="right"> ![Cloud-build](https://github.com/cvetaevvitaliy/telemetry-system/workflows/Cloud-build/badge.svg) ![Local-build](https://github.com/cvetaevvitaliy/telemetry-system/workflows/Local-build/badge.svg) </div>
This is a project for transmitting telemetry for a balloon, rocket and other things that can fly high. <br>

------

The project consists of:

- Ground station (RX)
-  Transmitting module (TX)

------



## Ground station (RX)

#### Block diagram

![](https://github.com/cvetaevvitaliy/telemetry-system/blob/master/doc/pic/Ground-station.png)

This is a module that includes one PCB:

- LoRa transceiver (E19-868M30S)  - 1W power output
- MCU STM32F405
- OLED display
- Lipo charging chip
- Data loger - logging of received information to MicroSD

The display is only needed to display service information, all telemetry is transmitted to a PC via USB to display telemetry in a GUI application, transmission of control commands for the rocket

For details see [WiKi - Ground station (RX)](https://github.com/cvetaevvitaliy/telemetry-system/wiki/Ground-station-(RX))


------



## Transmitting module (TX)

#### Block diagram

![](https://github.com/cvetaevvitaliy/telemetry-system/blob/master/doc/pic/Telemetry-module-RX.png)

This is a module that includes separate PCB modules for scalability:

- GPS module (Matek M8Q) and SMA connectors for antennas
- MCU module: LoRa transceiver (E19-868M30S)  - 1W power output, MCU STM32F405, logging data to MicroSD card (black box)
- IMU module ( acc, gyro, pressure, sensor )
- Servo module - to control flight stabilization (MCU STM32F405)
- Power module - for power supply of all modules

For details see [WiKi - Transmitting module (RX)](https://github.com/cvetaevvitaliy/telemetry-system/wiki/Transmitting-module-(TX))

------

## Releases

[Open releases versions ](https://github.com/cvetaevvitaliy/telemetry-system/releases)

------

## Open Source / Contributors

This software that is **open source** and is available free of charge without warranty to all users.

Big thanks to current contributors:

------

## How to build

TODO

```
mkdir build
cd build
cmake -DBUILD=GROUND_STATION ..
make -j
make clean
cmake -DBUILD=MCU_MODULE ..
make -j
make clean
cmake -DBUILD=SERVO_MODULE ..
make -j
```


### How to flash

TODO

example:

```dfu-util -a 0 -s 0x08000000:leave -D ../../rocket/telemetry-system/build/MCU_MODULE-V0.0.1-f78256d4.bin```

------

*the document will be updated as the project develops
