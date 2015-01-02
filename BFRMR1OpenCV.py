
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|

import cv
import cv2
import numpy
import sys
import math

#filecounter = 0 #for saving files when required
DisplayImage = False

print "Starting OpenCV"
capture = cv2.VideoCapture(0)

capture.set(3,640) #1024 640
capture.set(4,480) #600 480

if DisplayImage is True:
    cv2.namedWindow("camera", 0)
    cv2.namedWindow("map", 0)
    print "Creating OpenCV windows"
    cv2.waitKey(200)
    cv2.resizeWindow("camera", 640,480) 
    cv2.resizeWindow("map", 800,400) 
    print "Resizing OpenCV windows"
    cv2.waitKey(200)
    cv2.moveWindow("camera", 400,30)
    cv2.moveWindow("map", 1100,30)
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
# FindFirstEdge
#
# Captures an image, converts it to grayscale and blurs it before doing canny edge detection.
# Resulting edge detected image is then analysed to find the first edge in the image, scanning
# from the bottom to the top at set intervals in the x direction.
# Returns an array of screen coordinates corresponding to the first edge found.
##################################################################################################
def FindFirstEdge():

    StepSize = 10
    EdgeArray = []

    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #convert img to grayscale and store result in imgGray
    imgGray = cv2.bilateralFilter(imgGray,9,100,510) #blur the image slightly to remove noise             
    imgEdge = cv2.Canny(imgGray, 10, 60)           #edge detection

    imagewidth = imgEdge.shape[1] - 1
    imageheight = imgEdge.shape[0] - 1
    

    for j in range (0,imagewidth,StepSize):    #for the width of image array
        for i in range(imageheight,0,-1):      #step through every pixel in height of array from bottom to top
            if imgEdge.item(i,j) == 255:       #check to see if the pixel is white which indicates an edge has been found
                EdgeArray.append((j,i))    #if it is, add x,y coordinates to ObstacleArray
                break                          #if white pixel is found, skip rest of pixels in column
        else:                                  #no white pixel found
            EdgeArray.append((j,0))        #if nothing found, assume no obstacle. I may regret this decision!
            
    
    for x in range (len(EdgeArray)-1):      #draw lines between points in ObstacleArray 
        cv2.line(img, EdgeArray[x], EdgeArray[x+1],(0,255,0),1) 
    for x in range (len(EdgeArray)):        #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(img, (x*StepSize,imageheight), EdgeArray[x],(0,255,0),1)

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(120)

    return EdgeArray


##################################################################################################
#
# FindWorldCoords
#
# Converts all points in EdgeArray from screen coordinates to world coordinates and returns the
# results as an array of x and y coordinates. Needs head pan and tilt angles.
#
##################################################################################################
def FindWorldCoords(EdgeArray,HeadPanAngle,HeadTiltAngle):

    WorldArray = []

    for x in range(len(EdgeArray)): #For each point in edge array
        Point = EdgeArray[x]
        XScreen = Point[0]
        XDist = XScreen - 320
        XAngleRad = (XDist/632.0)
        XAngleDeg = math.degrees(XAngleRad)
        XAngleTotal = HeadPanAngle + XAngleDeg
        

        YScreen = Point[1]
        YDist = 240 - YScreen
        YAngleRad = (YDist/632.0)
        YAngleDeg = math.degrees(YAngleRad)
        YAngleTotal = HeadTiltAngle + YAngleDeg
        if YAngleTotal < 0: #YAngle must be less than zero other wise distance to object on the ground cannot be found
            YAngleTotalRad = math.radians(90+YAngleTotal)
            YCam = math.tan(YAngleTotalRad) * 23.0

            XAngleTotalRad = math.radians(XAngleTotal)
            XWorld = math.tan(XAngleTotalRad) * YCam
            YWorld = math.cos(XAngleTotalRad) * YCam

            WorldArray.append((int(XWorld),int(YWorld)))

            print YAngleTotal

    return WorldArray

##################################################################################################
#
# Detect Objects- Capture a frame and find obstacles in the image.
# Calculate distances to obstacles and display on the screen overlayed on the image
# Takes HeadPan and HeadTilt angles to calculate positions of obstacles in robot centric
# coordinates that are then returned in array form
#
##################################################################################################
def DetectObjects(HeadPanAngle,HeadTiltAngle,SonarValue):

    StepSize = 10
    SlopeChanges = []
    ObstacleEdges = []
    ObstacleArray = []
    DistanceToObstacle = []
    EdgeCoordinates = []
    SlopePositive = False
    EdgeFound = False

    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #convert img to grayscale and store result in imgGray
    imgGray = cv2.bilateralFilter(imgGray,9,50,50) #blur the image slightly to remove noise             
    imgEdge = cv2.Canny(imgGray, 10, 80)          #edge detection

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
                    if CurrentY > 200: #change of slope in lower half of image
                        SlopeChanges.append(x)
            SlopePositive = True
        elif Difference < 0:           #negative slope
            if SlopePositive is True:
                if x is not 0:         #ignore first point as this will always be logged
                    if CurrentY > 200: #change of slope in lower half of image
                        SlopeChanges.append(x)
            SlopePositive = False
            
    

    for x in range (0,(len(SlopeChanges)),1):              
        cv2.circle(img, ObstacleArray[SlopeChanges[x]], 2, (0,0,255),-1) #draw a circle at centre point of slope changes 
    
    #to find edges of each object
    for x in range (0,(len(SlopeChanges)),1):        #for each slope change found and stored in SlopeChanges
        FirstEdge = ()
        SlopeThreshold = 50
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

        #for each obstacle, find the distance to the object

        CurrentCoord = ObstacleEdges[x]
        CurrentY = CurrentCoord[1]
        CurrentX = CurrentCoord[0]
        print "Obstacle Y coord" ,CurrentY-240
        AngleDeg = 90 - ((CurrentY-240)/11.13) + HeadTiltAngle
        print "Angle Degrees", AngleDeg
        AngleRad = math.radians(AngleDeg)
        print "Angle Radians", AngleRad
        Dist = math.tan(AngleRad) * 23
        xanglerad = math.atan((340-CurrentX)/638)
        print "Xangle",xanglerad
        DistToObject = Dist/(math.cos(xanglerad))
        print "Distance to Object = ", DistToObject, "cm"
        DistanceToObstacle.append(DistToObject)
        cv2.putText(img,str(DistToObject)[:4]+"cm", ObstacleEdges[x], cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),1,cv2.CV_AA)
        
    #put sonar reading on to the screen
    cv2.putText(img,"Sonar = "+str(SonarValue)[:4]+"cm", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),1,cv2.CV_AA)

    #form an array of edge angles and distances to object of the form (Xangle1, Xangle2, Distance in cm, Xangle1...)
    for x in range (0,(len(ObstacleEdges)),2): 
        CurrentCoord = ObstacleEdges[x]
        CurrentX = CurrentCoord[0]
        CurrentXRobot = ((CurrentX-320)/11.13) + HeadPanAngle
        AngleRads = math.radians(CurrentXRobot) 
        XCoord = int(math.sin(AngleRads)*DistanceToObstacle[x/2])
        YCoord= int(math.cos(AngleRads)*DistanceToObstacle[x/2])
        EdgeCoordinates.append((XCoord,YCoord))

        NextCoord = ObstacleEdges[x+1]
        NextX = NextCoord[0]
        NextXRobot = ((NextX-320)/11.13) + HeadPanAngle
        AngleRads = math.radians(NextXRobot) 
        XCoord = int(math.sin(AngleRads)*DistanceToObstacle[x/2])
        YCoord= int(math.cos(AngleRads)*DistanceToObstacle[x/2])
        EdgeCoordinates.append((XCoord,YCoord))
 
    

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(150)
        
    print EdgeCoordinates
    return EdgeCoordinates

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
 
##################################################################################################
#
# ShowMap - Displays a map in an opencv window
# 
#
##################################################################################################
def ShowMap(MapArray):
    
    MapDisplay = cv2.cvtColor(MapArray, cv2.COLOR_GRAY2BGR)
    cv2.imshow("map", MapDisplay)
    cv2.waitKey(120)




def destroy():
    
    cv2.destroyAllWindows()
   


