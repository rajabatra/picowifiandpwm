# SPDX-FileCopyrightText: Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
#PWM for servo, LED and LED strip added: Raja Batra

import time
from microcontroller import cpu
import board
##
import time
import board
import pwmio
import servo
import neopixel
##

import busio
from digitalio import DigitalInOut
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT

## servo setup
# create a PWMOut object on Pin A2.
pwm = pwmio.PWMOut(board.GP0, frequency=100)

# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)

# ##
# LED setup 

yellowled = pwmio.PWMOut(board.GP28, frequency=200, duty_cycle=0)
dc=1

## led strip setup
# Update this to match the number of NeoPixel LEDs connected to your board.
num_pixels = 8

pixels = neopixel.NeoPixel(board.GP1, num_pixels)
pixels.brightness = 0.5






### WiFi ###

# Get wifi details and more from a secrets.py file
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

# Raspberry Pi RP2040
esp32_cs = DigitalInOut(board.GP13)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP15)


spi = busio.SPI(board.GP10, board.GP11, board.GP12)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets)

# Configure the RP2040 Pico LED Pin as an output
led_pin = DigitalInOut(board.LED)
led_pin.switch_to_output()

# Define callback functions which will be called when certain events happen.
# pylint: disable=unused-argument
def connected(client):
    # Connected function will be called when the client is connected to MQTT Broker.
    print("Connected to MQTT Broker! ")


def subscribe(client, userdata, topic, granted_qos):
    # This method is called when the client subscribes to a new feed.
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


# pylint: disable=unused-argument
def disconnected(client):
    # Disconnected function will be called when the client disconnects.
    print("Disconnected from MQTT Broker!")

##function to change PWM of light
def setPWM(client, topic, message):
    # Method called whenever user/feeds/yellowled has a new value
    print("New message on topic {0}: {1} ".format(topic, message))
    global dc
    dc = int(message)
   
    

##function to turn on LED
def on_led_msg(client, topic, message):
    # Method called whenever user/feeds/led has a new value
    print("New message on topic {0}: {1} ".format(topic, message))
    if message == "ON":
        led_pin.value = True
    elif message == "OFF":
        led_pin.value = False
    else:
        print("Unexpected message on LED feed.")



# Connect to WiFi
print("Connecting to WiFi...")
wifi.connect()
print("Connected!")

# Initialize MQTT interface with the esp interface
MQTT.set_socket(socket, esp)

# Initialize a new MQTT Client object
mqtt_client = MQTT.MQTT(
    broker="[broker address]",
    port=8883,
    username=secrets["mqtt_username"],
    password=secrets["mqtt_password"],
)

# Initialize an MQTT Client
io = IO_MQTT(mqtt_client)

# Connect the callback methods defined above to MQTT Broker
io.on_connect = connected
io.on_disconnect = disconnected
io.on_subscribe = subscribe

# Set up a callback for the board led feed
io.add_feed_callback("raja/led", on_led_msg)

# Set up a callback for the yellow led feed
io.add_feed_callback("raja/setpwm", setPWM)

# Connect to Adafruit IO
print("Connecting to MQTT Broker...")
io.connect()

# Subscribe to all messages on the board led feed
io.subscribe("raja/led")

# Subscribe to all messages on the yellow led feed
io.subscribe("raja/setpwm")

prv_refresh_time = 0.0


while True:



    # Poll for incoming messages
    try:
        io.loop()
   

    except (ValueError, RuntimeError) as e:
        print("Failed to get data, retrying\n", e)
        wifi.reset()
        wifi.connect()
        io.reconnect()
        continue
  
        
    
    for i in range(100):
        # PWM up and down
        
        if i < dc:
            
            #led up
            yellowled.duty_cycle = int(i * 65535 / 100)  # Up
            a = int(i * 65535 / 100)
            print("on")

            #led strip first color
            pixels.fill((255, 0, 0))

            #servo rotation
            my_servo.throttle = 1.0
        elif((a - int((i-dc) * 65535) / 100)>0):
            #led down
            yellowled.duty_cycle = a - int((i-dc) * 65535 / 100)# Down
            #led second color
            pixels.fill((0, 255, 255))
            #servo rotation
            my_servo.throttle = -1.0
      
        

  #  Send a new temperature reading to IO every 30 seconds
    if (time.monotonic() - prv_refresh_time) > 30:
        # take the cpu's temperature
        cpu_temp = cpu.temperature
        # truncate to two decimal points
        cpu_temp = str(cpu_temp)[:5]
        print("CPU temperature is %s degrees C" % cpu_temp)
        # publish it to io
        print("Publishing %s to temperature feed..." % cpu_temp)
        io.publish("raja/temperature", cpu_temp)
        print("Published!")
        prv_refresh_time = time.monotonic()
        prv_refresh_time = time.monotonic()
