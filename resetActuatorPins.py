import Adafruit_BBIO.GPIO as GPIO

#Setup
upPin = "P9_11"
downPin = "P9_12"

#Housekeeping
GPIO.setup(upPin, GPIO.OUT)
GPIO.setup(downPin, GPIO.OUT)
GPIO.output(upPin, GPIO.HIGH)
GPIO.output(downPin, GPIO.HIGH)