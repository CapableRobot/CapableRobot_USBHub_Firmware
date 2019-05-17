import time

import digitalio
import analogio
import board
import busio
import supervisor

import capablerobot_usbhub
import capablerobot_tlc59116
import capablerobot_eeprom

boot_time = time.monotonic()

led1 = digitalio.DigitalInOut(board.LED1)
led1.switch_to_output(value=False)

led2 = digitalio.DigitalInOut(board.LED2)
led2.switch_to_output(value=True)

led3 = digitalio.DigitalInOut(board.LED3)
led3.switch_to_output(value=True)

BEAT = 0.05

def stdout(*args):
    if supervisor.runtime.serial_connected:
        print(*args)

stdout("... booted ...")

i2c1 = busio.I2C(board.SCL,  board.SDA)
i2c2 = busio.I2C(board.SCL2, board.SDA2)

stdout("... configuring hub ...")
usb = capablerobot_usbhub.USBHub(i2c2)

stdout("... configuring leds ...")
BRIGHT   = 20
led_pwr  = capablerobot_tlc59116.TLC59116(i2c1, 0x61, pwm=BRIGHT)
led_data = capablerobot_tlc59116.TLC59116(i2c1, 0x62, pwm=BRIGHT)

eeprom = capablerobot_eeprom.EEPROM(i2c1, '24AA025E48')
stdout("serial number: %s" % eeprom.eui)

while True:
    time.sleep(0.1)

    led3.value = not led3.value

    # stdout(time.monotonic() - boot_time, usb.speeds)
    # stdout(time.monotonic() - boot_time, usb.rails)

    ## Set the data leds based on the detected per-port speeds
    for idx, speed in enumerate(usb.speeds):
        color = (0,0,0)

        if speed == 0b01:
            color = (0,0,255)
        if speed == 0b10:
            color = (0,255,0)
        if speed == 0b11:
            color = (255,255,255)

        ## If the upstream port is disconnected, light the LED reg
        if speed == 0b00 and idx == 0:
            color = (255,0,0)

        led_data.rgb(idx, color, update=False)

    led_data.update()