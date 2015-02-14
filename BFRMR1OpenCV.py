
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|

#import cv
import cv2
import numpy
import sys
import math
import zbar

DisplayImage = False


print "Starting OpenCV"
capture = cv2.VideoCapture(0)

capture.set(3,640) #1024 640 1280 800 384
capture.set(4,480) #600 480 960 600 288

if DisplayImage is True:
    cv2.namedWindow("camera", 0)
    cv2.namedWindow("map", 0)
    print "Creating OpenCV windows"
    cv2.waitKey(50)
    cv2.resizeWindow("camera", 640,480) 
    cv2.resizeWindow("map", 600,600) 
    print "Resizing OpenCV windows"
    cv2.waitKey(50)
    cv2.moveWindow("camera", 400,30)
    cv2.moveWindow("map", 1100,30)
    print "Moving OpenCV window"
    cv2.waitKey(50)


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
    cv2.waitKey(100)

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

    StepSize = 5
    EdgeArray = []

    #camera calibration values
    K = numpy.array([[612.00, 0, 330.00], [0, 613.00, 254.00], [0, 0, 1]])
    d = numpy.array([0.05897, -0.4494, 0, 0, 0]) # just use first two terms (no translation)

    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    #undistort image
    h, w = img.shape[:2]
    newcamera, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0) 
    #img = cv2.undistort(img, K, d, None, newcamera)

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)   #convert img to grayscale and store result in imgGray
    imgGray = cv2.bilateralFilter(imgGray,9,150,150) #blur the image slightly to remove noise             
    imgEdge = cv2.Canny(imgGray, 50, 100)             #edge detection

    imagewidth = imgEdge.shape[1] - 1
    imageheight = imgEdge.shape[0] - 1
    

    for j in range (0,imagewidth,StepSize):    #for the width of image array
        for i in range(imageheight-5,0,-1):    #step through every pixel in height of array from bottom to top
                                               #Ignore first couple of pixels as may trigger due to undistort
            if imgEdge.item(i,j) == 255:       #check to see if the pixel is white which indicates an edge has been found
                EdgeArray.append((j,i))        #if it is, add x,y coordinates to ObstacleArray
                break                          #if white pixel is found, skip rest of pixels in column
        else:                                  #no white pixel found
            EdgeArray.append((j,0))            #if nothing found, assume no obstacle. Set pixel position way off the screen to indicate
                                               #no obstacle detected
            
    
    for x in range (len(EdgeArray)-1):      #draw lines between points in ObstacleArray 
        cv2.line(img, EdgeArray[x], EdgeArray[x+1],(0,255,0),1) 
    for x in range (len(EdgeArray)):        #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(img, (x*StepSize,imageheight), EdgeArray[x],(0,255,0),1)

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(100)

    return EdgeArray

##################################################################################################
#
# ReadQRCode
#
#
##################################################################################################
def ReadQRCode():

    Data = 0
    Location = 0

    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)   #convert img to grayscale and store result in imgGray
    
    height, width = img.shape[:2] #get height and width of img
 
    # wrap image data
    image = zbar.Image(width, height, 'Y800', img.tostring())
    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')
    # scan the image for barcodes
    scanner.scan(image)

    for symbol in image:
        # do something useful with results
        
        #print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data
        #print "Data ",symbol.data
        #print "Type ",symbol.type
        #print "Location ",symbol.location

        Data = symbol.data
        Location = symbol.location 

        
      
    if Data is 0:
        print "No QRCode found in image"

    else:
        print "Found", Data
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for x in range (len(Location)):
            cv2.circle(img, Location[x], 5, (0,0,255),-1)       
        for x in range (len(Location)-1):
            cv2.line(img,Location[x],Location[x+1], (0,0,255),3)
        cv2.line(img,Location[0],Location[3], (0,0,255),3)
        cv2.putText(img,str(symbol.data), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(50)




##################################################################################################
#
# FindWorldCoords
#
# Converts all points in EdgeArray from screen coordinates to world coordinates and returns the
# results as an array of x and y coordinates. Needs head pan and tilt angles.
#
##################################################################################################
def FindWorldCoords(EdgeArray,HeadPanAngle,HeadTiltAngle,Scale):

    WorldArray = []

    for x in range(len(EdgeArray)): #For each point in edge array
        Point = EdgeArray[x]
        XScreen = Point[0]
        XDist = XScreen - 330.00
        XCamAngle = math.atan(XDist/612.00)
        XCamAngleDeg = math.degrees(XCamAngle )

        YScreen = Point[1]
        YDist = 254.00 - YScreen
        YCamAngle = math.atan(YDist/613.00)
        YCamAngleDeg = math.degrees(YCamAngle)

        YAngleTotal = HeadTiltAngle + YCamAngleDeg

        if YAngleTotal >= 0:
            YAngleTotal = -1
        #if YAngleTotal < 0 and YScreen > 0: #YAngle must be less than zero otherwise distance to object on the ground cannot be found

        YAngleTotalRad = math.radians(90+YAngleTotal)
        YCam = math.tan(YAngleTotalRad) * 23.5
        YCam = YCam/math.cos(math.radians(XCamAngle))
        XCam = math.tan(XCamAngle) * YCam
            #Rotate points around z axis by HeadPanAngle degrees
        HeadPanRad = math.radians(HeadPanAngle)
        XWorld = XCam*math.cos(-HeadPanRad) - YCam*math.sin(-HeadPanRad)
        YWorld = XCam*math.sin(-HeadPanRad) + YCam*math.cos(-HeadPanRad)

        XWorld = XWorld/Scale
        YWorld = YWorld/Scale

        WorldArray.append((int(XWorld),int(YWorld)))
            
    
    return WorldArray




##################################################################################################
#
# Find objects - Find objects of colour determined by threshold values passed to function
# Maybe return object location information in the future for use in main program
#
##################################################################################################
def FindObjects(ThresholdArray, MinSize, DistanceAtCentre, HeadPanAngle, HeadTiltAngle, Scale):

    objects = [] #tuple to hold object information
    objectsbox = []

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

            rect = cv2.minAreaRect(contours[x])
            box = cv2.cv.BoxPoints(rect)
            box = numpy.int0(box)
            cv2.drawContours(img,[box],0,(0,255,0),1) 

            boxcentre = rect[0] #get centre coordinates of each object
            boxcentrex = int(boxcentre[0])
            boxcentrey = int(boxcentre[1])
            cv2.circle(img, (boxcentrex, boxcentrey), 5, (0,255,0),-1) #draw a circle at centre point of object
 
            #convert from screen coordinates to robot coordinates
            XCam = boxcentrex - 330
            YCam = boxcentrey - 254
            z = 612.00 #camera focal length
            HeadPanRad = math.radians(HeadPanAngle)
            HeadTiltRad = math.radians(HeadTiltAngle)
            #rotate about y
            XRoboty = XCam*math.cos(HeadPanRad) + z*math.sin(HeadPanRad)
            YRoboty = YCam

            #rotate about x
            XRobot = XRoboty
            YRobot = YRoboty*math.cos(HeadTiltRad) - z*math.sin(HeadTiltRad)

            
            objects.append(XRobot/Scale)
            objects.append(YRobot/Scale)
            objects.append(contourarea)
            objects.append(DistanceAtCentre)

            print "Box Before",box

            for x in range (len(box)):
                box_x = box[x][0] - 330
                box_y = box[x][1] - 254
                #rotate about y
                XRoboty = box_x*math.cos(HeadPanRad) + z*math.sin(HeadPanRad)
                YRoboty = box_y
                #rotate about x
                XRobot = XRoboty
                YRobot = YRoboty*math.cos(HeadTiltRad) - z*math.sin(HeadTiltRad)
                
                box[x][0] = XRobot/Scale
                box[x][1] = YRobot/Scale

                

            print "Box After",box
            objectsbox.append(box)
            

    #write sonar reading to image
    cv2.putText(img,str(DistanceAtCentre)+"cm", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)

    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(50)
    
    print "Objectsbox", objectsbox
    return objectsbox
 

##################################################################################################
#
# NewMap - Experimental
#
##################################################################################################
def NewMap(MapWidth, MapHeight):

    MapArray = numpy.zeros((MapWidth, MapHeight), numpy.float32) #numpy array for map
    for x in range(MapArray.size):
        MapArray.itemset(x,0.5) #set all cell values to 0.5
    
    
    return MapArray

##################################################################################################
#
# AddToMap - 
#
##################################################################################################
def AddToMap(MapArray,Objects):
    
    Width = MapArray.shape[1]
    Height = MapArray.shape[0]
    #need to convert from robot coords to map coords, 0,0 is centre of map.
    print "Length of objects", len(Objects)

    for x in range (len(Objects)):
        for y in range(len(Objects[x])):
            Objects[x][y][0] = (Width/2) + int(Objects[x][y][0])
            Objects[x][y][1] = (Height/2) + int(Objects[x][y][1])
        pts = Objects[x].reshape((-1,1,2))
        cv2.fillPoly(MapArray,[pts],True,1)
        #cv2.drawContours(MapArray,[[194, 516],[194, 456],[285, 456],[285, 516]],0,1,1)

    #for x in range(0,len(Objects),4):
    #    X = (Width/2) +  (int(Objects[x]))
     #   Y = (Height/2) + (int(Objects[x+1]))
     #   cv2.circle(MapArray, (X, Y), 5, 1,-1) #draw a circle at centre point of object
        
    


##################################################################################################
#
# ShowMap - Displays a map in an opencv window
# MapArray is a numpy array where each element has a value between 0 and 1
#
##################################################################################################
def ShowMap(MapArray):
    
    MapDisplay = cv2.cvtColor(MapArray, cv2.COLOR_GRAY2BGR)
    cv2.imshow("map", MapDisplay)
    cv2.waitKey(50)

##################################################################################################
#
# TidyMap - Experimental
# Smooth, blur, threshold of map array
#
##################################################################################################
def TidyMap(MapArray):

    MapArray = cv2.GaussianBlur(MapArray,(5,5),0)

    for x in range(MapArray.size):
        if MapArray.item(x) > 0.6:
            MapArray.itemset(x,1)
        else:
            MapArray.itemset(x,0)
    
    return MapArray




def destroy():
    
    cv2.destroyAllWindows()
   


