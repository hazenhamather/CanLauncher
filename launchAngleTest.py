import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import time
import os
import math

#Setup
upPin = "P9_11"
downPin = "P9_12"
locationPin = "P9_33"
topAngle = 51
bottomAngle = 0

a = 12 #distance from actuator attachment and the pivot point
b = 19 #distance from pivot point to acvtuator pivot on turn table

ADC.setup() #setting up the ADC reading

#Housekeeping
GPIO.setup(upPin, GPIO.OUT)
GPIO.setup(downPin, GPIO.OUT)
GPIO.output(upPin, GPIO.HIGH)
GPIO.output(downPin, GPIO.HIGH)

def main():
    angleD = 15 #degrees
    c = getActuatorLength()
    pos = getXY(c)
    currentAngle = getAngle(pos)
    print currentAngle

    if currentAngle < angleD:
        aimUp(angleD)
    if currentAngle > angleD:
        aimDown(angleD)

def aimUp(desiredAngle):
    keepAiming = True
    GPIO.output(upPin, GPIO.LOW)
    while keepAiming:
        time.sleep(0.1)
        c = getActuatorLength()
        pos = getXY(c)
        currentAngle = getAngle(pos)
        print currentAngle
        if currentAngle >= topAngle:
            keepAiming = False
            GPIO.output(upPin, GPIO.HIGH)
            break
        if currentAngle >= desiredAngle:
            GPIO.output(upPin, GPIO.HIGH)
            keepAiming = False
            continue

def aimDown(desiredAngle):
    keepAiming = True
    GPIO.output(downPin, GPIO.LOW)
    while keepAiming:
        time.sleep(0.1)
        c = getActuatorLength()
        pos = getXY(c)
        currentAngle = getAngle(pos)
        print currentAngle
        if currentAngle <= bottomAngle:
            keepAiming = False
            GPIO.output(downPin, GPIO.HIGH)
            break
        if currentAngle <= desiredAngle:
            GPIO.output(downPin, GPIO.HIGH)
            keepAiming = False
            continue

def getActuatorLength():
    location = ADC.read_raw(locationPin)
    location = ADC.read_raw(locationPin)
    return (location - 9141.71)/(-303.294)  # inches

def getXY(c):
    x = (-50 * math.pow(c,2) - 29 * math.sqrt(-4 * math.pow(c,4) + 4084 * math.pow(c,2) - 198025) + 25525) / 2932
    y = (58 * math.pow(c,2)- 25 * math.sqrt(-4 * math.pow(c,4)+ 4084 * math.pow(c,2)- 198025) - 29609) / 2932
    return [x,y]

def getAngle(pos):
    return math.atan(pos[1]/pos[0])*180/math.pi

if __name__=="__main__":
    main()