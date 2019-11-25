import time

import digitalio
import analogio
import board
import busio
import supervisor

import capablerobot_usbhub
import capablerobot_tlc59116
import capablerobot_eeprom
import capablerobot_ucs2113

boot_time = time.monotonic()

led1 = digitalio.DigitalInOut(board.LED1)
led1.switch_to_output(value=False)

led2 = digitalio.DigitalInOut(board.LED2)
led2.switch_to_output(value=True)

led3 = digitalio.DigitalInOut(board.LED3)
led3.switch_to_output(value=True)

BEAT = 0.05
LED_BRIGHT = 80

def stdout(*args):
    if supervisor.runtime.serial_connected:
        print(*args)

stdout("... booted ...")

i2c1 = busio.I2C(board.SCL,  board.SDA)
i2c2 = busio.I2C(board.SCL2, board.SDA2)

stdout("... configuring hub ...")
usb = capablerobot_usbhub.USBHub(i2c1, i2c2)
ucs = capablerobot_ucs2113.Ports(i2c1)

stdout("... configuring leds ...")
BRIGHT   = 20
led_pwr  = capablerobot_tlc59116.TLC59116(i2c1, 0x61, pwm=BRIGHT)
led_data = capablerobot_tlc59116.TLC59116(i2c1, 0x62, pwm=BRIGHT)

eeprom = capablerobot_eeprom.EEPROM(i2c1, '24AA025E48')
stdout()
stdout("Unit SKU : %s" % eeprom.sku)
stdout("  Serial : %s" % eeprom.serial)
stdout()

while True:
    time.sleep(0.1)

    ## Internal heartbeat LED
    led3.value = not led3.value

    data_state = usb.data_state()

    ## Set the data LEDs based on the detected per-port speeds
    for idx, speed in enumerate(usb.speeds):
        color = (0,0,0)

        if idx > 0 and data_state[idx-1] == False:
            ## If port data is disabled, light the LED orange
            color = (LED_BRIGHT,int(LED_BRIGHT/2),0)
        elif speed == 0b01:
            color = (0,0,LED_BRIGHT)
        elif speed == 0b10:
            color = (0,LED_BRIGHT,0)
        elif speed == 0b11:
            color = (LED_BRIGHT,LED_BRIGHT,LED_BRIGHT)

        ## If the upstream port is disconnected, light the LED red
        if speed == 0b00 and idx == 0:
            color = (LED_BRIGHT,0,0)

        led_data.rgb(idx, color, update=False)

    led_data.update()


    power_state = usb.power_state()

    ## Set the power LEDs based on the measured per-port current
    for idx, current in enumerate(ucs.currents(raw=True, rescale=2)):

        ## With rescaling, raw reading may be above 255 (max value for LED), so cap it
        if current > 255:
            current == 255

        if idx == 0:
            color = (0,0,int(current/4))
        else:
            if power_state[idx-1] == False:
                ## If port power is disabled, light the LED orange
                color = (LED_BRIGHT,int(LED_BRIGHT/2),0)
            else:
                ## Otherwise, light blue with intensity based on measured power draw
                color = (0,0,current)

        led_pwr.rgb(idx, color, update=False)

    led_pwr.update()





