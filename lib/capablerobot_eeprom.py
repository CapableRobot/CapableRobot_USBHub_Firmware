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

from adafruit_bus_device.i2c_device import I2CDevice

## Array of default addr, pages bytes per page, location of EUI
EEPROMS = {}
EEPROMS['24AA02E48']  = [0x50,  8, 32,  8, (0xFA, 0xFF)]
EEPROMS['24AA025E48'] = [0x50,  8, 16, 16, (0xFA, 0xFF)]
EEPROMS['24AA02E64']  = [0x50,  8, 32,  8, (0xF8, 0xFF)]
EEPROMS['24AA025E64'] = [0x50,  8, 16, 16, (0xF8, 0xFF)]

def bytearry_to_ints(b):
    return [char for char in b]

def _memaddr_to_buf(address):
    if address > 255:
        raise ValueError("Library does not support memory addresses greater than one byte")

    return bytearray([address])

class EEPROM:

    def __init__(self, i2c_bus, name, addr=None):
        
        if addr is None:
            self.addr = EEPROMS[name][0]
        else:
            self.addr = addr

        self.addr_size = EEPROMS[name][1]
        self.pages = EEPROMS[name][2]
        self.bpp = EEPROMS[name][3]
        self.eui_addr = EEPROMS[name][4]

        self.i2c_device = I2CDevice(i2c_bus, self.addr)
        self.name = name

    def capacity(self):
        """Storage capacity in bytes"""
        return self.pages * self.bpp

    def read(self, mem_addr, nbytes):
        """Read one or more bytes from the EEPROM starting from a specific address"""
        buf = bytearray(nbytes)

        with self.i2c_device as i2c:
            i2c.write_then_readinto(_memaddr_to_buf(mem_addr), buf)

        return buf

    def write(self, mem_addr, buf):
        """Write one or more bytes to the EEPROM starting from a specific address"""
        with self.i2c_device as i2c:
            i2c.write(_memaddr_to_buf(mem_addr)+bytearray(buf))
        
    @property
    def eui(self, string=True):
        addr = list(self.eui_addr)
        data = bytearry_to_ints(self.read(addr[0], addr[1]-addr[0]+1))
        
        if len(data) == 6:
            data = data[0:3] + [0xFF, 0xFE] + data[3:6]

        if string:
            return ''.join(["%0.2X" % v for v in data])

        return data

    @property
    def serial(self):
        return self.eui

    @property
    def sku(self):
        data = bytearry_to_ints(self.read(0x00, 0x06))

        ## Prototype units didn't have the PCB SKU programmed into the EEPROM
        ## If EEPROM location is empty, we assume we're interacting with that hardware
        if data[0] == 0 or data[0] == 255:
            return 'CRR3C4'

        return ''.join([chr(v) for v in data])
