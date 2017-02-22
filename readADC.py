import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import time
import os
import math

#Setup
upPin = "P9_11"
downPin = "P9_12"
locationPin = "P9_33"

a = 12 #distance from actuator attachment and the pivot point
b = 19 #distance from pivot point to acvtuator pivot on turn table

keepAiming = False; #boolean on whether to keep aiming or not

ADC.setup() #setting up the ADC reading

# location = ADC.read(locationPin)
# Adafruit says to read the value twice due to a bug
# location =    ADC.read(locationPin)

#Housekeeping
GPIO.setup(upPin, GPIO.OUT)
GPIO.setup(downPin, GPIO.OUT)
GPIO.output(upPin, GPIO.HIGH)
GPIO.output(downPin, GPIO.HIGH)

while True:
    test = ADC.read_raw(locationPin)
    print ADC.read_raw(locationPin)
    time.sleep(0.2)