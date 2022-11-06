#!usr/bin/env python3

#---------------------------------------------------------------------------------------------
# SETUP
#---------------------------------------------------------------------------------------------

# camera modules
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
# opencv modules
import cv2 as cv
import numpy
import sys

sys.path.append('/usr/local/lib/python3.9/dist-packages')
# GPIO modules & initialization
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17,GPIO.OUT) #Enemy Flag to send to Propeller
GPIO.setup(18,GPIO.OUT) #Friend Flag to send to Propeller
GPIO.setup(3,GPIO.OUT) #Enemy Flag
GPIO.setup(2,GPIO.OUT) #Friendly Flag

# load camera
PiCamera._set_exposure_mode = 7
camera = PiCamera()


#=============================================================================================
# getTagNumbers(image,drawMarker = False):
# This Function to takes an image as input and outputs the tag ids of the aruco tags detected 
# and the pixel coordinates of the corners of the aruco tag in the iamge.
#=============================================================================================
def getTagNumbers(image,drawMarker = False):
	#Convert image to grayscale
	#grayScaleIm = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
	#RIm = cv.rotate(grayScaleIm,cv.ROTATE_180)
    #cv.waitKey(0)
	#Get the arcuo tag dictionary for the type of tag we want
	arucoTagDict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
	#Create the aruco parameter var
	tagParams = cv.aruco.DetectorParameters_create()
	#Detect the tags
	(corners, tagIds, rejected) = cv.aruco.detectMarkers(image, arucoTagDict,parameters=tagParams)
	if (len(corners) > 0) and (drawMarker):
		cv.aruco.drawDetectedMarkers(image,corners)

	return corners,tagIds


#=============================================================================================
# determineType(tagId)
# This Function takes a Tag Id as input and returns a 1 for enemny or 0 for friend. 
# A tag ID above 9 indicates an enemy
#=============================================================================================
def determineType(tagId):
	if tagId>9:
		enemy = True
	else:
		enemy = False
	return enemy

 
#---------------------------------------------------------------------------------------------
# MAIN
#---------------------------------------------------------------------------------------------

#Setting up camera
camera.resolution = (512,400)
rawCap = PiRGBArray(camera)

#Sleep so camera has time to setup
sleep(0.1)

#Setting all LED's and comunication pins to LOW to begin with
GPIO.output(3,GPIO.LOW)
GPIO.output(2,GPIO.LOW)
GPIO.output(17,GPIO.LOW)
GPIO.output(18,GPIO.LOW)

IdArray = [None] * (8) #Array to store Tag ID's of detected tags
IdCounter = 0

for frame in camera.capture_continuous(rawCap, format="bgr",use_video_port=True):
    
    #Saving the captured frame
    myImage = cv.cvtColor(frame.array,cv.COLOR_BGR2GRAY)      
    
    #Passing the frame through the getTagNumbers function
    (corners,detectedIds) = getTagNumbers(myImage)
    
    print(detectedIds)

    #Only if an actua tag was detected will the code proceed
    if detectedIds is not None:

        #If tag is a friendly HIGH 2
        if determineType(detectedIds) == False: 
            #GPIO.output(2,GPIO.HIGH) #LED for Friend
            GPIO.output(18,GPIO.LOW)
            GPIO.output(17,GPIO.LOW)
            if detectedIds not in IdArray:
                GPIO.output(18,GPIO.HIGH)
                sleep(0.5)

        #If tag is a enemey HIGH 3
        elif determineType(detectedIds) == True:    
            #GPIO.output(3,GPIO.HIGH) #LED for Enemy
            GPIO.output(18,GPIO.LOW)
            GPIO.output(17,GPIO.LOW)
            if detectedIds not in IdArray:
                GPIO.output(17,GPIO.HIGH)
                sleep(0.5)

        #Add the detected Id number to the array for only the first detection
        if detectedIds not in IdArray:      
            IdArray[IdCounter] = detectedIds
            IdCounter += 1

    else:
        #Else low both
        GPIO.output(18,GPIO.LOW)
        GPIO.output(17,GPIO.LOW)
        #GPIO.output(3,GPIO.LOW)
        #GPIO.output(2,GPIO.LOW)
        
    #Cleaning up the capture   
    cv.destroyAllWindows()        
    rawCap.truncate(0)