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

import adafruit_bus_device.i2c_device as i2c_device

_REG_PORT1_CURRENT = const(0x00)
_REG_PORT2_CURRENT = const(0x01)
_REG_PORT_STATUS   = const(0x02)

_MA_PER_BIT = 13.3

class UCS2113:

    def __init__(self, i2c, address=0x57):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._buffer = bytearray(1)

    def _read_register_bytes(self, register, result, length=None):
        # Read the specified register address and fill the specified result byte
        # array with result bytes.  Make sure result buffer is the desired size
        # of data to read.
        if length is None:
            length = len(result)

        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytes([register]), result, in_end=length, stop=False)

    def currents(self, one=True, two=True, raw=False):
        out = []

        if one:
            self._read_register_bytes(_REG_PORT1_CURRENT, self._buffer)
            out.append(self._buffer[0])

        if two:
            self._read_register_bytes(_REG_PORT2_CURRENT, self._buffer)
            out.append(self._buffer[0])

        if raw:
            return out

        return [float(v) * _MA_PER_BIT for v in out]


class Ports:

    def __init__(self, i2c):
        self.ch12 = UCS2113(i2c, address=0x57)
        self.ch34 = UCS2113(i2c, address=0x56)

    def currents(self, ports=[1,2,3,4], total=True, raw=False, rescale=0):
        out = []

        one = 1 in ports
        two = 2 in ports
        three = 3 in ports
        four = 4 in ports

        if one or two:
            out.extend(self.ch12.currents(one=one, two=two, raw=raw))

        if three or four:
            out.extend(self.ch34.currents(one=three, two=four, raw=raw))

        if total:
            out = [sum(out)] + out

        ## PCB has limit of 2.9A max (2.7A nominal) per port, which is a reading of 218
        ## So, values here are scaled such taht 218 becomes 255
        if rescale != 0:
            scale = float(255) / float(218) * rescale
            out = [int(float(v) * scale) for v in out]

        return out