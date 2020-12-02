# Introduction

This repository contains the open-source & CircuitPython-based firmware for the Capable Robot Programmable USB Hub.

## Upgrading Firmware on your USB Hub

There are two steps to upgrading the firmware running on your USB Hub.

- Upgrading the Circuitpython release
- Upgrading the firmware (e.g. the code in this in repository)

A command-line-tool (CLI) is included in this repository to assist in these steps.

Note that during the firmware update process your USB Hub will reboot which will cause connected devices to disconnected and re-enumerate.  It is recommended that downstream devices like USB thumb drives, hard drives, etc are disconnected prior to starting this procedure.

The step-by-step instructions:

1. Power on your Programmable USB Hub.
2. Connect the MCU USB Port to a host computer (the Host USB port does not need to be connected).
3. A USB mass-storage device name "CIRCUITPY" will mount on your host computer.
4. Update the Circuitpython release: `./usbhub_firmware.py circuitpython PATH_TO_CIRCUITPY_DRIVE`.  This will cause the Hub to reboot into bootloader mode, the latest supported Circuitpython release will be installed, and then the USB Hub will then reboot into normal mode.
5. Update the firmware: `./usbhub_firmware.py firmware PATH_TO_CIRCUITPY_DRIVE`
6. Unmount the "CIRCUITPY" drive and then power-cycle your Programmable USB Hub.

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