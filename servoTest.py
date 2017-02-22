import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
import os

servoPin = "P9_14"
PWM.start(servoPin, 9, 20)