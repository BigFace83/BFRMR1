
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|

import cv
import cv2
import numpy
import sys

#filecounter = 0 #for saving files when required
DisplayImage = True

print "Starting OpenCV"
capture = cv2.VideoCapture(0)

capture.set(3,640) #1024 640
capture.set(4,480) #600 480

if DisplayImage is True:
    cv2.namedWindow("camera", 0) 
    print "Creating OpenCV windows"
    cv2.waitKey(200)
    cv2.resizeWindow("camera", 300,300) 
    print "Resizing OpenCV windows"
    cv2.waitKey(200)
    cv2.moveWindow("camera", 500,30)
    print "Moving OpenCV window"
    cv2.waitKey(200)


##################################################################################################
#
# Display image - Capture a frame and display it on the screen
#
##################################################################################################
def DisplayFrame():

    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent

    cv2.imshow("camera", img)
    cv2.waitKey(120)

##################################################################################################
#
# Detect Edges - Capture a frame and edge detect before displaying
#
##################################################################################################
def DetectEdges():

    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #convert img to grayscale and store result in imgGray
    imgEdge = cv2.Canny(imgGray, 30, 200) #edge detection





    cv2.imshow("camera", imgEdge)
    cv2.waitKey(120)


##################################################################################################
#
# Find objects - Find objects of colour determined by threshold values passed to function
# Maybe return object location information in the future for use in main program
#
##################################################################################################
def FindObjects(ThresholdArray, MinSize, DistanceAtCentre):

    objects = [] #tuple to hold object information
    

    #print "OpenCV capturing frames"
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent


    #print "OpenCV processing"
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #convert img to HSV and store result in imgHSVyellow
    lower = numpy.array([ThresholdArray[0],ThresholdArray[1],ThresholdArray[2]]) #numpy arrays for upper and lower thresholds
    upper = numpy.array([ThresholdArray[3], ThresholdArray[4], ThresholdArray[5]])
    frame_threshed = cv2.inRange(imgHSV, lower, upper) #threshold imgHSV
    frame_threshed = cv2.blur(frame_threshed,(5,5))


    #playing around with contours
    contours, hierarchy = cv2.findContours(frame_threshed,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    for x in range (len(contours)):
        
        contourarea = cv2.contourArea(contours[x])
        if contourarea > MinSize:
            #cnt = contours[x]
            #cv2.drawContours(img, [cnt], 0, (0,255,0), 3)

            rect = cv2.minAreaRect(contours[x])
            box = cv2.cv.BoxPoints(rect)
            box = numpy.int0(box)
            cv2.drawContours(img,[box],0,(0,255,0),1) 

            boxcentre = rect[0] #get centre coordinates of each object
            boxcentrex = int(boxcentre[0])
            boxcentrey = int(boxcentre[1])
            cv2.circle(img, (boxcentrex, boxcentrey), 5, (0,255,0),-1) #draw a circle at centre point of object

            #thisobject = (boxcentrex,boxcentrey,contourarea)
            #objects = objects + (thisobject,)
          
            objects.append(boxcentrex)
            objects.append(boxcentrey)
            objects.append(contourarea)
            objects.append(DistanceAtCentre)

    cv2.putText(img,str(DistanceAtCentre)+"cm", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(120)

    return objects
 

def destroy():
    
    cv2.destroyAllWindows()
   


