# Introduction

CircuitPython firmware for the Capable Robot Programmable USB Hub.

## Dependencies

This firmware depends on:

* [Adafruit CircuitPython](https://github.com/adafruit/circuitpython)
* [Bus Device](https://github.com/adafruit/Adafruit_CircuitPython_BusDevice)

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
[the Adafruit library and driver bundle](https://github.com/adafruit/Adafruit_CircuitPython_Bundle). The USB Hub ships with dependencies installed.


## I2C Devices

**I2C Bus 1** is accessible from the host side via the I2C bridge and from the MCU.

| Address | Device | Description | 
|  --- | --- | --- | 
| 0x20 | MCP23008| GPIO expander for downstream USB data switches 
| 0x50 | 24AA025E48 | EEPROM with Product SKU and unique EUI
| 0x56 | UCS2113 |USB Dual-Port Power Switch and Current Monitor, ports 3 & 4
| 0x57 | UCS2113 |USB Dual-Port Power Switch and Current Monitor, ports 1 & 2
| 0x61 | TLC59116 | LED driver for power LEDs
| 0x62 | TLC59116 | LED driver for data LEDs

**I2C Bus 2** is only accessible from from the MCU.

| Address | Device | Description | 
|  --- | --- | --- | 
| 0x2C | TUSB212 | USB High Speed Signal Conditioner on the upstream link
| 0x2D | USB4715 | USB Hub chip configuration interface

## Contributing

Contributions are welcome! Please read our [Code of Conduct](https://github.com/capablerobot/CapableRobot_CircuitPython_USBHub_Firmware/blob/master/CODE_OF_CONDUCT.md) before contributing to help this project stay welcoming.