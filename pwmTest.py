import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
import os

os.system("config-pin -a P9_14 pwm")

#Looking down the barrel
motorPins = "P9_14"
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


PWM.start(motorPins, 30, 20000)
time.sleep(5)

# for i in range(0,31,10):
#     PWM.start(motorPins, 70+i, 20000)
#     time.sleep(2)
# time.sleep(5)


#PWM.start(channel, duty, freq=2000, polarity=0)

# for i in range (1,50):
#     PWM.start(motorPins, 100-i, 20000)
#     print (100-i)
#     time.sleep(1)
# PWM.set_duty_cycle(turretPin,10.5)
# time.sleep(5)
# PWM.set_duty_cycle(turretPin,4.5)
# time.sleep(5)
# PWM.set_duty_cycle(turretPin,7.5)
# time.sleep(5)
# for i in range(0,300):
#     PWM.set_duty_cycle(turretPin,7.5+(i/100))
#     time.sleep(0.02)
# for i in range(0,600):
#     PWM.set_duty_cycle(turretPin,10.5-i/100)
# for i in range(0,300):
#     PWM.set_duty_cycle(turretPin,4.5+i/100)

PWM.stop(motorPins)
PWM.cleanup()