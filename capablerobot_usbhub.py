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

import board
import digitalio
import analogio
from micropython import const

from adafruit_bus_device.i2c_device import I2CDevice

# pylint: disable=bad-whitespace
_I2C_ADDR_USB   = const(0x2D)
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
_CHARGE_CONFIG  = const(0x343C)
  
_CFG_REG_CMD      = bytearray([0x99, 0x37, 0x00])
_DEFAULT_PORT_MAP = [1, 2, 3, 4]
# pylint: enable=bad-whitespace

_I2C_ADDR_MCP   = const(0x20)
_GPIO           = const(0x09)

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
        for char in b:
            x |= (char << shift*8)
            shift += 1
    else:
        x = 0
        for char in b:
            x <<= 8
            x |= char
    return x

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)

def get_bit(value, bit):
    return (value & (1<<bit)) > 0 

class USBHub:

    def __init__(self, i2c1_bus, i2c2_bus, force=False):

        ## Setup pins so that statue upon switchign to output
        ## is identical to the board electrical default.  This
        ## allows object to be created an no state change occur.
        self.pin_rst = digitalio.DigitalInOut(board.USBRESET)
        self.pin_rst.switch_to_output(value=True)

        self.pin_hen = digitalio.DigitalInOut(board.USBHOSTEN)
        self.pin_hen.switch_to_output(value=False)

        self.vlim = analogio.AnalogIn(board.ANVLIM)
        self.vlogic = analogio.AnalogIn(board.AN5V)

        self.i2c_device = I2CDevice(i2c2_bus, _I2C_ADDR_USB, probe=False)
        self.mcp_device = I2CDevice(i2c1_bus, _I2C_ADDR_MCP, probe=False)

        ## Here we are using the port remapping to determine if the hub
        ## has been previously configured.  If so, we don't need to reset
        ## it or configure it and can just control it as-is.
        ##
        ## If the hub has not been configured (e.g. when the board is first 
        ## powered on), this call will raise an OSError.  That will then trigger
        ## the normal reset & configure process.
        try:
            self.remap = self.get_port_remap()
        except OSError:
            self.remap = _DEFAULT_PORT_MAP

        if self.remap == _DEFAULT_PORT_MAP or force:
            self.reset()
            self.configure()
            self.set_mcp_config()

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

        inbuf = bytearray(length+1)

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

    # pylint: disable=invalid-name
    @property
    def id(self):
        buf = self._read_register(_REVISION)
        device_id = (buf[3] << 8) + buf[2]
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
        conn = bytearry_to_int(self._read_register(_CONNECTION))
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
        out = [0xAA, 0x56, 0x00]

        with self.i2c_device as i2c:
            i2c.write(bytearray(out))

    def reset(self):       
        time.sleep(0.05)
        
        # Put in reset for at least 10 ms
        self.pin_rst.value = False
        time.sleep(0.05)

        # Must wait at least 1 ms after RESET_N deassertion for straps to be read
        # Testing has found this delay must be MUCH longer than 1 ms for subsequent
        # I2C calls to suceed.
        self.pin_rst.value = True
        time.sleep(0.05)

    def configure(self):
        ## Reverse DP/DM pints of  upstream port and ports 3 & 4
        self.set_port_swap(values=[True, False, False, True, True])
        self.set_hub_control(lpm_disable=True)
        self.set_hub_config_3(port_map_enable=True)

        ## Remap ports so that case physcial markings match the USB
        self.set_port_remap(ports=[2, 4, 1, 3])

        self.set_charging_config()

        self.attach()

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

    def set_port_remap(self, ports=[1, 2, 3, 4]):
        self.remap = ports

        port12 = ((ports[1] << 4) & 0xFF) | (ports[0] & 0xFF)
        port34 = ((ports[3] << 4) & 0xFF) | (ports[2] & 0xFF)

        self._write_register(_REMAP_12, [port12])
        self._write_register(_REMAP_34, [port34])

    def set_charging_config(self, ports=[1,2,3,4], ucs_lim=0b11, enable=True, dcp=True, se1=0b00, china_mode=False):

        ## ucs_lim : When controlling UCS through I2C, this sets the current limit.
        ## 0b00 : 500 mA
        ## 0b01 : 1000 mA
        ## 0b10 : 1500 mA
        ## 0b11 : 2000 mA

        ## 'dcp' is Dedicated Charging Mode.  Ignored if china_mode is enabled.
        ## This mode only active when a USB Host is not present.  When a host is 
        ## present, CDP mode is used.

        ## Bit 1 & 2 are SE1. Enables SE1 charging mode for certain devices. 
        ## This mode is only activated when a USB host is not present. When a 
        ## host is present, the mode of operation is CDP. When SE1 mode and DCP 
        ## mode are both enabled, the hub toggles between the two modes of 
        ## operation as necessary to ensure the device can charge.
        ##
        ## 0b00 : Mode Disabled
        ## 0b01 : 1A mode (D-: 2.7V, D+: 2.0V)
        ## 0b10 : 2A mode (D-: 2.0V, D+: 2.7V)
        ## 0b11 : 2.5A mode enabled (D-: 2.7V, D+: 2.7V)

        ## Bit 0 is Battery Charging Support Enable. This bit enables CDP and 
        ## must be set for any battery charging functions to be enabled. Other 
        ## functions in addi- tion to CDP are enabled by setting their 
        ## respective bits in addition to this bit.

        value = (ucs_lim & 0b11) << 6 | \
                dcp << 5 | \
                china_mode << 4 | \
                (se1 & 0b11) << 1 | \
                enable 
        print("charging {}".format(value))

        for port in ports:
            ## Register address is based on the port number
            self._write_register(_CHARGE_CONFIG+port-1, [value])

    def set_mcp_config(self, inputs=[False, False, False, False]):
        """Set direction on MCP IO pins.  'inputs' list will set GP0 thru GP4 to inputs, if respective position is true"""

        ## Bits 7 thru 4 control USB data enables on downstream ports 1 thru 4, respectively.
        ## They must be set to 0 to make them outputs.
        value = 0b00000000 | \
                inputs[3] << 3 | \
                inputs[2] << 2 | \
                inputs[1] << 1 | \
                inputs[0]

        with self.mcp_device as i2c:
            ## Write to IODIR register and defaults to other registers.
            ## 0x09 (GPIO) register has to be 0b0000_0000 so that downstream ports default to enabled
            i2c.write(bytearray([0x00, value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

    def _read_mcp_register(self, addr):
        inbuf = bytearray(1)

        with self.mcp_device as i2c:
            i2c.write_then_readinto(bytearray([addr]), inbuf)
        
        return inbuf[0]

    def data_state(self):
        value = self._read_mcp_register(_GPIO)
        return [not get_bit(value, 7), not get_bit(value, 6), not get_bit(value, 5), not get_bit(value, 4)]

    def data_enable(self, ports=[]):
        inbuf = bytearray(1)

        with self.mcp_device as i2c:
            i2c.write_then_readinto(bytearray([_GPIO]), inbuf)

            for port in ports:
                inbuf[0] = clear_bit(inbuf[0], 8-port)

            i2c.write(bytearray([_GPIO])+inbuf)

    def data_disable(self, ports=[]):
        inbuf = bytearray(1)

        with self.mcp_device as i2c:
            i2c.write_then_readinto(bytearray([_GPIO]), inbuf)

            for port in ports:
                inbuf[0] = set_bit(inbuf[0], 8-port)

            i2c.write(bytearray([_GPIO])+inbuf)


    def get_port_remap(self):
        port12 = bytearry_to_int(self._read_register(_REMAP_12))
        port34 = bytearry_to_int(self._read_register(_REMAP_34))

        return [port12 & 0x0F, (port12 >> 4) & 0x0F, port34 & 0x0F, (port34 >> 4) & 0x0F]

    def power_state(self, ports=[1,2,3,4]):
        out = []

        for port in ports:
            data = self._read_register(_POWER_SELECT_1+(port-1)*4)
            out.append((data[0] & 1) == 1)

        return out

    def power_disable(self, ports=[]):
        for port in ports:
            self._write_register(_POWER_SELECT_1+(port-1)*4, [0x80])

    def power_enable(self, ports=[]):
        for port in ports:
            self._write_register(_POWER_SELECT_1+(port-1)*4, [0x81])

    @property
    def rails(self):
        vlim, vlogic = None, None

        if self.vlim is not None:
            voltage = float(self.vlim.value) / 65535.0 * self.vlim.reference_voltage 
            vlim = voltage * (1870 + 20000) / 1870
        
        if self.vlogic is not None:
            voltage = float(self.vlogic.value) / 65535.0 * self.vlogic.reference_voltage 
            vlogic = voltage * 2

        return vlim, vlogic
        
