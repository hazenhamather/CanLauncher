import cv2
import time
import numpy as np
import math
import os
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

#PWM Pins
motorPin = "P9_14"
loaderPin = "P8_19"
targetFoundPin = "P9_21"
launchReadyPin = "P9_22"

#Button Pins
startButton = "P9_13"
confirmButton = "P9_15"
# launchButton = "P9_16"

upPin = "P9_11"
downPin = "P9_12"
locationPin = "P9_33"
topAngle = 51
bottomAngle = 0

#Looking down the barrel
rightDirection1 = "P9_24"
rightDirection2 = "P9_26"

leftDirection1 = "P9_28"
leftDirection2 = "P9_30"

GPIO.setup(rightDirection1, GPIO.OUT)
GPIO.setup(rightDirection2, GPIO.OUT)
GPIO.setup(leftDirection1, GPIO.OUT)
GPIO.setup(leftDirection2, GPIO.OUT)

GPIO.output(rightDirection1, GPIO.HIGH)
GPIO.output(rightDirection2, GPIO.LOW)
GPIO.output(leftDirection1, GPIO.LOW)
GPIO.output(leftDirection2, GPIO.HIGH)

a = 12 #distance from actuator attachment and the pivot point
b = 19 #distance from pivot point to acvtuator pivot on turn table

ADC.setup() #setting up the ADC reading

#Housekeeping
GPIO.setup(upPin, GPIO.OUT)
GPIO.setup(downPin, GPIO.OUT)
GPIO.output(upPin, GPIO.HIGH)
GPIO.output(downPin, GPIO.HIGH)

def main():
    # cap = cv2.VideoCapture(0)
    os.system("cd /dev")
    os.system("v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=1")
    os.system("cd ~/CanLauncher")

    os.system("config-pin -a P9_14 pwm")
    os.system("config-pin -a P9_21 pwm")
    os.system("config-pin -a P9_22 pwm")

    GPIO.setup(startButton, GPIO.IN)
    GPIO.setup(confirmButton, GPIO.IN)
    # GPIO.setup(launchButton, GPIO.IN)

    time.sleep(0.5)

    boom()

def boom():
    while 1:
        cap = cv2.VideoCapture(0)
        print "Awaiting start signal"
        GPIO.wait_for_edge(startButton, GPIO.RISING)
        GPIO.cleanup()
        distanceToTarget = scanForFace(cap)
        # distanceToTarget = 50
        # distanceToTarget = getDistanceToFace(faces)
        rightTarget = confirmTarget()
        if rightTarget:
            angleD = getLaunchAngle(distanceToTarget)
            aim(angleD)
            clearedForLaunch = confirmLaunch()
            launchBaby(distanceToTarget)
            resetAngle()
    # while 1:
    #         # get a frame from RGB camera
    #     ret, frame = cap.read()
    #     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #     faces = faceCascade.detectMultiScale(
    #         gray,
    #         scaleFactor = 1.5,
    #         minNeighbors = 5
    #     )
    #     print "done"
    #     for (x,y,w,h) in faces:
    #         cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    #         distance = 146.645*math.exp(-7.207e-3*w)
    #         print distance
    #     #     if x < (halfScreen - w/2):
    #     #     	print "Pan Right"
    #     #     elif x > (halfScreen + w/2):
    #     #     	print "Pan Left"
    #     #     else:
    #     #         print "Correct Target?"
    #     # cv2.imshow("Faces",frame)
    #     k = cv2.waitKey(5) & 0xFF
    #     if k == 27:
    #         break
    # cv2.destroyAllWindows()

def scanForFace(cap):
    while 1:
        faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
        time.sleep(1)
        ret, frame = cap.read()
        # cap.release()
        # time.sleep(0.01)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.3,
            minNeighbors=5
        )
        print faces
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                cv2.destroyAllWindows()
                cap.release()
                inches = 146.645*math.exp(-7.207e-3*w)
                print "The distance to your face is"
                print inches*0.0254
                return inches * 0.0254
            break

def getDistanceToFace(faces):
    # focalLength = 780
    # faceWidth = 6.8
    # camWidth = 640
    # camHeight = 480
    # halfScreen = camWidth / 2
    for (x,y,w,h) in faces:
        inches = 146.645*math.exp(-7.207e-3*w)
        return inches * 0.0254

def confirmTarget():
    # PWM.start(targetFoundPin,50,1)
    print "Entered confirm target method"
    # GPIO.add_event_detect(confirmButton, GPIO.RISING)
    # GPIO.add_event_detect(launchButton, GPIO.RISING)
    while 1:
        return True
        # if not GPIO.input(confirmButton):
        # if GPIO.event_detected(confirmButton):
            # PWM.stop(targetFoundPin)
            # return True
        # elif GPIO.input(launchButton):
            # PWM.stop(targetFoundPin)
            # return False
        # if GPIO.input(confirmButton):
        #     PWM.stop(targetFoundPin)
        #     return TrueGPIO.event_detected
        # elif GPIO.input(launchButton):
        #     PWM.stop(targetFoundPin)
        #     return False

def confirmLaunch():
    # PWM.start(targetFoundPin, 50, 1)
    # PWM.start(launchReadyPin, 50, 1)
    print "Awaiting Launch confirmation"
    while 1:
        # if not GPIO.input(launchButton):
            # PWM.stop(targetFoundPin)
            # PWM.stop(launchReadyPin)
            return True
        # elif GPIO.input(confirmButton):
            # PWM.stop(targetFoundPin)
            # PWM.stop(launchReadyPin)
            # return False

def launchBaby(distance):
    # GPIO.wait_for_edge(startButton, GPIO.RISING) #no turning back
    # GPIO.cleanup()
    RPM = 586.5670268*math.sqrt(distance)
    print RPM
    dutyCycle = (0.0002114464417*RPM+0.03844561492)*100
    dutyCycle = dutyCycle*3.5*2
    dutyCycle = int(round(dutyCycle))
    if dutyCycle > 100:
        dutyCycle = 99
    print dutyCycle
    if dutyCycle <= 60:
        PWM.start(motorPin, dutyCycle, 20000)
    else:
        # thing = dutyCycle-60
        remainder = dutyCycle%10
        tens = dutyCycle//10
        # x = []
        # for i in range(0,tens):
        #     x.append(10)

        for i in range(0,dutyCycle-60,10):
            PWM.start(motorPin,60+i,20000)
            time.sleep(1)
            finalDutyCycle = 60+i
        PWM.start(motorPin, finalDutyCycle+remainder, 20000)
    # PWM.start(motorPin, dutyCycle, 20000)
    time.sleep(5)
    PWM.start(loaderPin, 4, 60)
    time.sleep(3.5)
    PWM.stop(motorPin)
    PWM.stop(loaderPin)
    PWM.cleanup()


def getLaunchAngle(distance):
    #Kuza will give me the equation
    return 45

def resetAngle():
    aim(30)

def aim(targetAngle):
    c = getActuatorLength()
    pos = getXY(c)
    currentAngle = getAngle(pos)
    print currentAngle

    if currentAngle < targetAngle:
        aimUp(targetAngle)
    if currentAngle > targetAngle:
        aimDown(targetAngle)

def aimUp(desiredAngle):
    keepAiming = True
    GPIO.output(upPin, GPIO.LOW)
    while keepAiming:
        c = getActuatorLength()
        pos = getXY(c)
        currentAngle = getAngle(pos)
        time.sleep(.5)
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
        # time.sleep(.100)
        c = getActuatorLength()
        pos = getXY(c)
        currentAngle = getAngle(pos)
        time.sleep(.5)
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

if __name__ == "__main__":
    main()