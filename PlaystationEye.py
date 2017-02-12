# import the necessary modules
# import freenect
import numpy as np
import cv2
import time
import math
import Adafruit_BBIO.GPIO as GPIO

#warmup
time.sleep(0.5)

# time.sleep(5)
cap = cv2.VideoCapture(1)
faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
focalLength = 780
faceWidth = 6.8
camWidth = 640
camHeight = 480
halfScreen = camWidth/2;

foundFace = False;
targetConfirmed = False;
launchConfirmed = False;

#Set up confirm button
yesButton = "P9_11"
noButton = "P9_12"
GPIO.setup(yesButton, GPIO.IN)
GPIO.setup(noButton, GPIO.IN)

def scanForFace():
	while 1:
		#This is where we will scan back and forth hunting for a face. We will
		#begin turning the turret and scanning at the same time.

		#Turn Servo one click right and start back other way when we max out

		#Do a cap.read() command
		ret, frame = cap.read();
		gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    	faces = faceCascade.detectMultiScale(
        	gray,
        	scaleFactor = 1.3,
        	minNeighbors = 5
    	)
    	for (x,y,w,h) in faces:
    		foundFace = True
    		aimToFace()

def aimToFace():
	while 1:
	    ret, frame = cap.read()
    	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    	faces = faceCascade.detectMultiScale(
        	gray,
        	scaleFactor = 1.3,
        	minNeighbors = 5
    	)
    	for (x,y,w,h) in faces:
        	cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        	distance = 146.645*math.exp(-7.207e-3*w);
        	# print distance
        	if x < (halfScreen - 1.5*w):
        		#click servo right
        		print "Pan Right"
        	elif x > (halfScreen + 1.5*w):
        		#Click servo left
        		print "Pan Left"
        	else:
        		isConfirmed = confirmLaunch()
        		if isConfirmed:
        			prepareToLaunch(distance)
        		else:
        			break;

def confirmLaunch():
	GPIO.wait_for_edge(yesButton, GPIO.FALLING)
	GPIO.wait_for_edge(noButton, GPIO.FALLING)
	while 1:
		#Confirm with button press that this is the intended target (use a toggle switch possibly)
		if GPIO.event_detected(yesButton):
			return True
		elif GPIO.event_detected(noButton):
			return False
		
def prepareToLaunch(distance):
	#Calculate the require launch angle and initial velocity

	#Aim the Barrel

	#Load the can

	#Spin up the wheels

	#

# while 1:
#         # get a frame from RGB camera
#     ret, frame = cap.read()
#     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#     faces = faceCascade.detectMultiScale(
#         gray,
#         scaleFactor = 1.3,
#         minNeighbors = 5
#     )
#     for (x,y,w,h) in faces:
#         cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
#         distance = 146.645*math.exp(-7.207e-3*w);
#         # print distance
#         if x < (halfScreen - w/2):
#         	print "Pan Right"
#         elif x > (halfScreen + w/2):
#         	print "Pan Left"
#         elif 
#     # cv2.imshow("Faces",frame)
#     k = cv2.waitKey(5) & 0xFF
#     if k == 27:
#         break
# cv2.destroyAllWindows()