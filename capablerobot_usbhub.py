# The MIT License (MIT)
#
# Copyright (c) 2019 Chris Osterwood for Capable Robot Components
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import board
import digitalio
import time

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

_REVISION       = const(0x0000)
_VENDOR_ID      = const(0x3000)
_PRODUCT_ID     = const(0x3002)
_DEVICE_ID      = const(0x3004)
_HUB_CONFIG_1   = const(0x3006)
_HUB_CONFIG_2   = const(0x3007)
_HUB_CONFIG_3   = const(0x3008)
_PORT_SWAP      = const(0x30FA)
_HUB_CTRL       = const(0x3104)
_SUSPEND        = const(0x3197)
_POWER_STATUS   = const(0x30E5)
_REMAP_12       = const(0x30FB)
_REMAP_34       = const(0x30FC)
_POWER_STATE    = const(0x3100)
_CONNECTION     = const(0x3194)
_DEVICE_SPEED   = const(0x3195)
_POWER_SELECT_1 = const(0x3C00)
_POWER_SELECT_2 = const(0x3C04)
_POWER_SELECT_3 = const(0x3C08)
_POWER_SELECT_4 = const(0x3C0C)


_CFG_REG_CMD    = bytearray([0x99, 0x37, 0x00])

def _register_length(addr):
    if addr in [_REVISION]:
        return 4

    if addr in [_VENDOR_ID, _PRODUCT_ID, _DEVICE_ID, _POWER_STATE]:
        return 2

    return 1

def bytearry_to_int(b, lsb_first=True):
    if lsb_first:
        x = 0
        shift = 0
        for c in b:
            x |= (c << shift*8)
            shift += 1
    else:
        x = 0
        for c in b:
            x <<= 8
            x |= c
    return x

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)


class USBHub:

    def __init__(self, i2c_bus, addr=0x2D):
        self.pin_rst = digitalio.DigitalInOut(board.USBRST)
        self.pin_rst.switch_to_output(value=False)

        self.pin_hen = digitalio.DigitalInOut(board.USBHEN)
        self.pin_hen.switch_to_output(value=True)

        self.remap = [1,2,3,4]

        self.reset()
        self.i2c_device = I2CDevice(i2c_bus, addr)

        self.reset()
        self.configure()

    def _write_register(self, address, xbytes):

        if len(xbytes) != _register_length(address):
            raise ValueError("Incorrect payload length for register %d" % address)

        ## 2 bytes for 'write' and count
        ## 4 bytes for address (base + offset)
        ## num bytes actually get written
        length = 2+4+len(xbytes)

        ## Prepare the pre-amble
        out = [
            0x00, 
            0x00, 
            length, # Length of rest of packet
            0x00,   # Write configuration register
            len(xbytes) & 0xFF, # Will be writing N bytes (later)
            0xBF, 0x80,
            (address >> 8) & 0xFF, address & 0xFF
        ]

        with self.i2c_device as i2c:
            ## Write the pre-amble and then the payload
            i2c.write(bytearray(out+xbytes))
            
            ## Execute the Configuration Register Access command
            i2c.write(_CFG_REG_CMD)

    def _read_register(self, address):
        length = _register_length(address)

        ## Prepare the pre-amble
        out = [
            0x00, 
            0x00, 
            0x06, # Length of rest of packet
            0x01, # Read configuration register
            length & 0xFF, # Will be reading N bytes (later)
            0xBF, 0x80,
            (address >> 8) & 0xFF, address & 0xFF
        ]

        inbuf  = bytearray(length+1)

        with self.i2c_device as i2c:
            ## Write the pre-amble 
            i2c.write(bytearray(out))
        
            ## Execute the Configuration Register Access command
            i2c.write(_CFG_REG_CMD)
        
            ## Access the part of memory where our data is
            i2c.write_then_readinto(bytearray([0x00, 0x06]), inbuf, stop=False)

        ## First byte is the length of the rest of the message.  
        ## We don't want to return that to the caller
        return inbuf[1:length+1]


    @property
    def id(self):
        buf = self._read_register(_REVISION)
        device_id   = (buf[3] << 8) + buf[2]
        revision_id = buf[0]

        return device_id, revision_id

    @property
    def vendor_id(self):
        return bytearry_to_int(self._read_register(_VENDOR_ID))

    @property
    def product_id(self):
        return bytearry_to_int(self._read_register(_PRODUCT_ID))

    @property
    def speeds(self):
        conn  = bytearry_to_int(self._read_register(_CONNECTION))
        speed = bytearry_to_int(self._read_register(_DEVICE_SPEED))
        
        out = [0]*5

        ## Have to follow logical to physical remapping
        for idx, port in enumerate(self.remap):
            out[port] = (speed >> (idx*2)) & 0b11

        ## Upstream port is not in the speed register, so take data from
        ## the connection register.  Unfortunately, no way to know speed.
        out[0] = (conn & 0b1)*3

        return out

    def attach(self):
        ## 0xAA 0x55 : Exit SOC_CONFIG and Enter HUB_CONFIG stage
        ## 0xAA 0x56 : Exit SOC_CONFIG and Enter HUB_CONFIG stage with SMBus slave enabled
        out  = [0xAA, 0x56, 0x00]

        with self.i2c_device as i2c:
            i2c.write(bytearray(out))

    def reset(self):
        # Put in reset for 10 ms
        self.pin_rst.value = False
        time.sleep(0.01)

        # Must wait at least 1 ms after RESET_N deassertion for straps to be read
        self.pin_rst.value = True
        time.sleep(0.02)

    def configure(self):
        ## Reverse DP/DM pints of  upstream port and ports 3 & 4 
        self.set_port_swap(values=[True, False, False, True, True])
        self.set_hub_control(lpm_disable=True)
        self.set_hub_config_3(port_map_enable=True)

        ## Remap ports so that case physcial markings match the USB 
        self.set_port_remap(ports=[2,4,1,3])

        self.attach()
        self.upstream(True)

        time.sleep(0.02)

    
    def upstream(self, state):
        self.pin_hen.value = not state

    def set_port_swap(self, values=[False, False, False, False, False]):
        value = 0

        for idx, bit in enumerate(values):
            if bit:
                value = set_bit(value, idx)

        self._write_register(_PORT_SWAP, [value])

    def set_hub_control(self, lpm_disable=False, reset=False):
        value = lpm_disable << 1 | \
                reset

        self._write_register(_HUB_CTRL, [value])

    def set_hub_config_3(self, port_map_enable=True, string_descriptor_enable=False):
        value = port_map_enable << 3 | \
                string_descriptor_enable

        self._write_register(_HUB_CONFIG_3, [value])

    def set_port_remap(self, ports=[1,2,3,4]):
        self.remap = ports

        port12 = ((ports[1] << 4) & 0xFF) | (ports[0] & 0xFF)
        port34 = ((ports[3] << 4) & 0xFF) | (ports[2] & 0xFF)

        self._write_register(_REMAP_12, [port12])
        self._write_register(_REMAP_34, [port34])
