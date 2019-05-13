import digitalio
import board
import busio
import supervisor
import time

import capablerobot_usbhub

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

stdout("... booted     ...")

i2c2 = busio.I2C(board.SCL2, board.SDA2)
usb = capablerobot_usbhub.USBHub(i2c2)

stdout("... configured ...")

stdout("ID ", usb.id)
stdout("VENDOR_ID ", usb.vendor_id)
stdout("PRODUCT_ID ", usb.product_id)

while True:
    
    led3.value = False
    time.sleep(BEAT)

    led3.value = True
    time.sleep(1-BEAT)
