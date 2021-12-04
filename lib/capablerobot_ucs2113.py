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

import time
import adafruit_bus_device.i2c_device as i2c_device

_REG_PORT1_CURRENT = const(0x00)
_REG_PORT2_CURRENT = const(0x01)
_REG_PORT_STATUS   = const(0x02)
_REG_INTERRUPT1    = const(0x03)
_REG_INTERRUPT2    = const(0x04)
_REG_AUTO_RECOVERY = const(0x15)
_REG_OC_BEHAVIOR1  = const(0x23)
_REG_OC_BEHAVIOR2  = const(0x24)


_MA_PER_BIT = 13.3

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)

def get_bit(value, bit):
    return (value & (1<<bit)) > 0

class UCS2113:

    def __init__(self, i2c, address=0x57, filter_threshold=5, filter_length=8):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._buffer = bytearray(3)

        self._filter_threshold = filter_threshold
        self._filter_length = filter_length

        self._ch1_history = [0] * self._filter_length
        self._ch2_history = [0] * self._filter_length

    def _read_register_bytes(self, register, result, length=None, max_attempts=5):
        # Read the specified register address and fill the specified result byte
        # array with result bytes.  Make sure result buffer is the desired size
        # of data to read.
        if length is None:
            length = len(result)

        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            try:
                with self.i2c_device as i2c:
                    i2c.write_then_readinto(bytes([register]), result, in_end=length)
                break
            except OSError:
                time.sleep(0.01)
                if attempts >= max_attempts:
                    return False

        return True

    def currents(self, one=True, two=True, raw=False, filtered=True):
        out = []
        value = None

        if one:
            success = self._read_register_bytes(_REG_PORT1_CURRENT, self._buffer)
            if not success:
                return []

            value = int(self._buffer[0])
            
            self._ch1_history.pop(0)
            self._ch1_history.append(value)
            
            if filtered and value < self._filter_threshold:
                filter_mean = sum(self._ch1_history) / self._filter_length
                out.append(filter_mean)
            else:
                out.append(value)

        if two:
            success = self._read_register_bytes(_REG_PORT2_CURRENT, self._buffer)
            if not success:
                return []

            value = int(self._buffer[0])

            self._ch2_history.pop(0)
            self._ch2_history.append(value)

            if filtered and value < self._filter_threshold:
                filter_mean = sum(self._ch2_history) / self._filter_length
                out.append(filter_mean)
            else:
                out.append(value)

        if raw:
            return [round(v) for v in out]

        return [float(v) * _MA_PER_BIT for v in out]

    def port_status(self):
        success = self._read_register_bytes(_REG_PORT_STATUS, self._buffer, length=3)

        if not success:
            return None

        ## CC1, CC2, ALERT1, ALERT2 in bits 4 thru 7
        value = self._buffer[0]

        out = [
            self.decode_interrupt_register(self._buffer[1]),
            self.decode_interrupt_register(self._buffer[2])
        ]

        if get_bit(value, 4):
            out[0].append("CC")

        if get_bit(value, 5):
            out[1].append("CC")

        if get_bit(value, 6):
            out[1].append("ALERT")

        if get_bit(value, 7):
            out[1].append("ALERT")

        return out

    def interrupts(self, channel):
        if channel == 1:
            addr = _REG_INTERRUPT1
        else:
            addr = _REG_INTERRUPT2

        success = self._read_register_bytes(addr, self._buffer)

        if not success:
            return None

        return self.decode_interrupt_register(self._buffer[0])

    def decode_interrupt_register(self, value):
        out = []

        if get_bit(value, 7):
            out.append("ERR")

        if get_bit(value, 6):
            out.append("DISCH")

        if get_bit(value, 5):
            out.append("RESET")

        if get_bit(value, 4):
            out.append("KEEP_OUT")

        if get_bit(value, 3):
            out.append("DIE_TEMP_HIGH")

        if get_bit(value, 2):
            out.append("OVER_VOLT")

        if get_bit(value, 1):
            out.append("BACK_BIAS")

        if get_bit(value, 0):
            out.append("OVER_LIMIT")

        return out



class Ports:

    def __init__(self, i2c):
        self.ch12 = UCS2113(i2c, address=0x57)
        self.ch34 = UCS2113(i2c, address=0x56)

    def status(self):

        ch12 = self.ch12.port_status()
        ch34 = self.ch34.port_status()

        if ch12 is None or ch34 is None:
            return [[], [], [], []]

        out = [ch12[0], ch12[1], ch34[0], ch34[1]]

        return out

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

        if len(out) != len(ports):
            return []

        if total:
            out = [sum(out)] + out

        ## PCB has limit of 2.9A max (2.7A nominal) per port, which is a reading of 218
        ## So, values here are scaled such that 218 becomes 255
        if rescale != 0:
            scale = float(255) / float(218) * rescale
            out = [int(float(v) * scale) for v in out]

        return out