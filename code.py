import time

import digitalio
import analogio
import board
import busio
import supervisor

import capablerobot_usbhub
import capablerobot_tlc59116

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

stdout("... configuring leds ...")
BRIGHT   = 20
led_pwr  = capablerobot_tlc59116.TLC59116(i2c1, 0x61, pwm=BRIGHT)
led_data = capablerobot_tlc59116.TLC59116(i2c1, 0x62, pwm=BRIGHT)

stdout()
stdout("Unit SKU : %s" % usb.unit_sku)
stdout("Revision : %s" % usb.unit_revision)
stdout("  Serial : %s" % usb.unit_serial)
stdout()

upstream_state = 'reset'
upstream_last_time = boot_time


def reset():
    global upstream_state, upstream_last_time
    
    usb.reset()
    usb.configure()
    usb.set_mcp_config()

    ## Light the host data LED orange to show the reset is occuring
    led_data.rgb(0, (LED_BRIGHT,int(LED_BRIGHT/2),0), update=True)
    time.sleep(0.5)

    ## Reset the upstream timeout to ensure that the next
    ## reset can only occurs after the specified timeout
    upstream_state = 'reset'
    upstream_last_time = time.monotonic()

    usb.set_last_poll_time(time.monotonic())

while True:
    time.sleep(usb.config["loop_delay"])
    
    ## Look for data from the Host computer via special USB4715 registers
    try:
        usb.poll_for_host_comms()
    except RuntimeError:
        stdout(time.monotonic(), "--- RESET due to loop delay ---")
        reset()
        continue
        
    ## Internal heartbeat LED
    led3.value = not led3.value

    if usb.config["external_heartbeat"]:
        if led3.value:
            led_data.aux(0, update=False)
        else:
            led_data.aux(250, update=False)
    elif led3.value:
        ## If the configuration was changed while the LED is on, 
        ## we still need to turn it off when the next update happens.
        led_data.aux(0, update=False)


    data_state = usb.data_state()

    if data_state is None:
        continue

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

        if idx == 0:
            if speed == 0b00:
                ## If the upstream port is disconnected, light the 
                ## LED red and record that the link is down
                color = (LED_BRIGHT,0,0)
                upstream_state = 'down'
            else:
                upstream_last_time = time.monotonic()
                upstream_state = 'up'

        led_data.rgb(idx, color, update=False)

    led_data.update()

    power_state = usb.power_state()

    ## Set the power LEDs based on the measured per-port current
    ucs_currents = usb.power_measure(raw=True, rescale=2)
    ucs_status = usb.power_errors()

    for idx, current in enumerate(ucs_currents):

        ## With rescaling, raw reading may be above 255 (max value for LED), so cap it
        if current > 255:
            current == 255

        if idx == 0:
            color = (0,0,int(current/4))
        else:
            if power_state[idx-1] == False:
                ## If port power is disabled, light the LED orange
                color = (LED_BRIGHT,int(LED_BRIGHT/2),0)
            elif "ERR" in ucs_status[idx-1] or "ALERT" in ucs_status[idx-1]:
                ## UCS is reporting an alert, light the LED red
                color = (LED_BRIGHT,0,0)
            elif "CC" in ucs_status[idx-1]:
                ## UCS is reporting constant current mode, light the LED green
                color = (0,LED_BRIGHT,0)
            else:
                ## Otherwise, light blue with intensity based on measured power draw
                color = (0,0,current)

        led_pwr.rgb(idx, color, update=False)

    led_pwr.update()

    if usb.config['reset_on_link_loss'] and upstream_state == 'down':
        if time.monotonic() - upstream_last_time > usb.config['link_loss_delay']:
            stdout("--- RESET DUE TO LINK LOSS ---")
            reset()



