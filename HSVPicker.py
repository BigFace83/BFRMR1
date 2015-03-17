#!/usr/bin/python

########################################################################################
#
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|
#
# HSV Colour selector for object detection using OpenCV
#
#
# Author : Peter Neal
#
# Date : 17 March 2015
# Last Update : 17 March 2015
#
########################################################################################

import cv2
import numpy as np
from colorama import init,Fore
init(autoreset=True)

def nothing(x):
    pass

print(Fore.GREEN + "Starting OpenCV")
capture = cv2.VideoCapture(0)

capture.set(3,640)
capture.set(4,480)

cv2.namedWindow("camera", 0)
print (Fore.GREEN + "Creating OpenCV windows")
cv2.resizeWindow("camera", 640,480) 
cv2.moveWindow("camera", 400,30)
print (Fore.GREEN + "Moving OpenCV window")
cv2.waitKey(50)

# create trackbars for HSV Selection
cv2.createTrackbar('HLow','camera',0,179,nothing)
cv2.createTrackbar('SLow','camera',0,255,nothing)
cv2.createTrackbar('VLow','camera',0,255,nothing)

cv2.createTrackbar('HHigh','camera',0,179,nothing)
cv2.createTrackbar('SHigh','camera',0,255,nothing)
cv2.createTrackbar('VHigh','camera',0,255,nothing)



while True:

    HLow = cv2.getTrackbarPos('HLow','camera')
    SLow = cv2.getTrackbarPos('SLow','camera')
    VLow = cv2.getTrackbarPos('VLow','camera')
    HHigh = cv2.getTrackbarPos('HHigh','camera')
    SHigh = cv2.getTrackbarPos('SHigh','camera')
    VHigh = cv2.getTrackbarPos('VHigh','camera')

    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #convert img to HSV and store result in imgHSVyellow
    lower = np.array([HLow, SLow, VLow]) #np arrays for upper and lower thresholds
    upper = np.array([HHigh, SHigh, VHigh])

    imgthreshed = cv2.inRange(imgHSV, lower, upper) #threshold imgHSV
    imgthreshed = cv2.blur(imgthreshed,(3,3))

    cv2.imshow("camera", imgthreshed)
    cv2.waitKey(10)

cv2.destroyAllWindows()








 
