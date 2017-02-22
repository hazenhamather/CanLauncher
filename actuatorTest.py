import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import time
import os

upPin = "P9_11"
downPin = "P9_12"
locationPin = "P9_33"

ADC.setup()

location = ADC.read(locationPin)
# Adafruit says to read the value twice due to a bug
location = ADC.read(locationPin)

GPIO.setup(upPin, GPIO.OUT)
GPIO.setup(downPin, GPIO.OUT)
GPIO.output(upPin, GPIO.HIGH)
GPIO.output(downPin, GPIO.HIGH)

GPIO.output(upPin, GPIO.LOW)
time.sleep(3)
location = ADC.read(locationPin)
print location
GPIO.output(upPin, GPIO.HIGH)
time.sleep(1)
location = ADC.read(locationPin)
print location
GPIO.output(downPin, GPIO.LOW)
time.sleep(3)
location = ADC.read(locationPin)
print location
GPIO.output(downPin, GPIO.HIGH)
location = ADC.read(locationPin)
print location
print "finished"