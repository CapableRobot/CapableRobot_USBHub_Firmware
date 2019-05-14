
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice


_CONTROL    = const(0x80)
_AUTOINCR   = const(0xA2)

_NUM_LEDS   = const(16)

class TLC59116:

    def __init__(self, i2c_bus, addr, **kwargs):
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.state = [0]*_NUM_LEDS
        self.configure(**kwargs)

    def _write_register(self, address, xbytes):
        with self.i2c_device as i2c:
            i2c.write(bytearray([address]+xbytes))

    def configure(self, 
        auto_increment=True, 
        allcall=True, 
        osc_off=False, 
        pwm=0xA0, freq=0, 
        sub1=False, sub2=False, sub3=False,
        change_on_ack=False, clear_errors=False, group_blink=False):

        data =  []

        ## Configure MODE1 register
        data += [auto_increment << 7 | \
                 osc_off << 4 | \
                 sub1 << 3 | \
                 sub2 << 2 | \
                 sub3 << 1 | \
                 allcall]

        ## Configure MODE2 register
        data += [clear_errors << 7 | \
                 group_blink << 5 | \
                 change_on_ack << 3]


        data += self.state
        data += [pwm, freq]

        ## Set all LED configs to 0b11, which enables individual PWM control and global dimming
        data += [0xFF, 0xFF, 0xFF, 0xFF]

        ## Defaults for sub-address and all-call address
        data += [0x92, 0x94, 0x98, 0xD0]

        self._write_register(_CONTROL, data)

    def off(self):
        self.state = [0]*_NUM_LEDS
        self.update()

    def aux(self, value, update=True):
        self.state[15] = value

        if update:
            self.update()

    def rgb(self, channel, rgb, update=True):
        if channel == 0:
            idx = 0
        else:
            idx = 15 - channel*3

        rgb = list(rgb)

        self.state[idx+0] = rgb[0]
        self.state[idx+1] = rgb[1]
        self.state[idx+2] = rgb[2]

        if update:
            self.update()

    def update(self):
        self._write_register(_AUTOINCR, self.state)
