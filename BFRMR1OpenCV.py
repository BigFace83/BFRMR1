
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
    cv2.namedWindow("camera2", 0)
    print "Creating OpenCV windows"
    cv2.waitKey(200)
    cv2.resizeWindow("camera", 640,480) 
    cv2.resizeWindow("camera2", 640,480) 
    print "Resizing OpenCV windows"
    cv2.waitKey(200)
    cv2.moveWindow("camera", 400,30)
    cv2.moveWindow("camera2", 1100,30)
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
def DetectObjects():

    StepSize = 10
    SlopeChanges = []
    ObstacleEdges = []
    ObstacleArray = []
    SlopePositive = False
    EdgeFound = False

    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #convert img to grayscale and store result in imgGray
    imgGray = cv2.bilateralFilter(imgGray,9,80,80) #blur the image slightly to remove noise             
    imgEdge = cv2.Canny(imgGray, 10, 100)          #edge detection

    ObstacleArray = []
    imagewidth = imgEdge.shape[1] - 1
    imageheight = imgEdge.shape[0] - 1
    

    for j in range (0,imagewidth,StepSize):    #for the width of image array
        for i in range(imageheight,0,-1):      #step through every pixel in height of array from bottom to top
            if imgEdge.item(i,j) == 255:       #check to see if the pixel is white which indicates an edge has been found
                ObstacleArray.append((j,i))    #if it is, add x,y coordinates to ObstacleArray
                break                          #if white pixel is found, skip rest of pixels in column
        else:                                  #no white pixel found
            ObstacleArray.append((j,0))        #if nothing found, assume no obstacle. I may regret this decision!
            
    
    for x in range (len(ObstacleArray)-1):      #draw lines between points in ObstacleArray 
        cv2.line(img, ObstacleArray[x], ObstacleArray[x+1],(0,255,0),1) 
    for x in range (len(ObstacleArray)):        #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(img, (x*StepSize,imageheight), ObstacleArray[x],(0,255,0),1)  

    #look for slope changes in ObstacleArray to try and find obstacles on the floor
    for x in range (len(ObstacleArray)-1):
        CurrentCoord = ObstacleArray[x]
        CurrentY = CurrentCoord[1]
        NextCoord = ObstacleArray[x+1]
        NextY = NextCoord[1]
        Difference = CurrentY - NextY
        if Difference > 0:             #positive slope
            if SlopePositive is False:
                if x is not 0:         #ignore first point as this will always be logged
                    if CurrentY > 240: #change of slope in lower half of image
                        SlopeChanges.append(x)
            SlopePositive = True
        elif Difference < 0:           #negative slope
            if SlopePositive is True:
                if x is not 0:         #ignore first point as this will always be logged
                    if CurrentY > 240: #change of slope in lower half of image
                        SlopeChanges.append(x)
            SlopePositive = False
            
    

    for x in range (0,(len(SlopeChanges)),1):              
        cv2.circle(img, ObstacleArray[SlopeChanges[x]], 2, (0,0,255),-1) #draw a circle at centre point of slope changes 
    
    #to find egdes of each object
    for x in range (0,(len(SlopeChanges)),1):        #for each slope change found and stored in SlopeChanges
        FirstEdge = ()
        SlopeThreshold = 20
        CurrentSlopeCoord = ObstacleArray[SlopeChanges[x]]
        SlopeY = CurrentSlopeCoord[1]
        for y in range(SlopeChanges[x],1,-1):        #step through values in ObstacleArray
            CurrentCoord = ObstacleArray[y]          #separate out X and Y coordinates of point stored in SlopeChanges
            CurrentX = CurrentCoord[0]
            CurrentY = CurrentCoord[1]
            NextCoord = ObstacleArray[y-1]           #get Y coordinate of next point in ObstacleArray
            NextY = NextCoord[1]
            Difference = CurrentY- NextY
            if Difference > SlopeThreshold:          #until a difference of more than SlopeThreshold is found
                FirstEdge = (CurrentX,SlopeY)        #store coordinates of obstacle edge to be used later of other side of obstacle is found
                EdgeFound = True                     #only scan in other direction if an edge has been found
                break
            EdgeFound = False                        #if no edge found, ignore this change in slope, probably not an obstacle
        if EdgeFound is True:
            for y in range(SlopeChanges[x],(len(ObstacleArray))-1,1): #scan in other direction to find opposing edge of obstacle
               CurrentCoord = ObstacleArray[y]
               CurrentX = CurrentCoord[0]
               CurrentY = CurrentCoord[1]
               NextCoord = ObstacleArray[y+1]
               NextY = NextCoord[1]
               Difference = CurrentY - NextY         
               if Difference > SlopeThreshold:
                    ObstacleEdges.append(FirstEdge)   #only log edges if both sides of the obstacle have been found
                    ObstacleEdges.append((CurrentX,SlopeY))
                    break

    #print "Slope Changes", SlopeChanges
    print "Obstacle Edges", ObstacleEdges
    print "Number of Slope Changes" , len(SlopeChanges)
    print "Number of Obstacle Edges" ,len(ObstacleEdges)

    for x in range (0,(len(ObstacleEdges)),1):              
        cv2.circle(img, ObstacleEdges[x], 2, (255,0,0),-1) #draw a circle at centre point of edges

    for x in range (0,(len(ObstacleEdges))-1,2): 
        cv2.line(img, ObstacleEdges[x], ObstacleEdges[x+1],(0,0,255),10) 

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(150)
        #cv2.imshow("camera2", imgEdge)
        #cv2.waitKey(120)


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
   


