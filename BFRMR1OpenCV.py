
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|

#import cv
import time
import cv2
import numpy as np
import sys
import math
import zbar
from colorama import init,Fore
init(autoreset=True)

DisplayImage = True


print "Starting OpenCV"
capture = cv2.VideoCapture(0)

capture.set(3,640) #1024 640 1280 800 384
capture.set(4,480) #600 480 960 600 288

if DisplayImage is True:
    cv2.namedWindow("camera", 0)
    cv2.namedWindow("transform", 0)
    print (Fore.GREEN + "Creating OpenCV windows")
    #cv2.waitKey(50)
    cv2.resizeWindow("camera", 640,480) 
    cv2.resizeWindow("transform", 300,300) 
    print (Fore.GREEN + "Resizing OpenCV windows")
    #cv2.waitKey(50)
    cv2.moveWindow("camera", 400,30)
    cv2.moveWindow("transform", 1100,30)
    print (Fore.GREEN + "Moving OpenCV window")
    cv2.waitKey(50)

##################################################################################################
#
# Set up detectors for symbols
#
##################################################################################################
detector = cv2.SURF(1000)
HomeSymbol = cv2.imread("homesymbol.png")
HomeSymbolKeypoints, HomeSymbolDescriptors = detector.detectAndCompute(HomeSymbol , None)
FoodSymbol = cv2.imread("foodsymbol.png")
FoodSymbolKeypoints, FoodSymbolDescriptors = detector.detectAndCompute(FoodSymbol , None)
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
matcher = cv2. FlannBasedMatcher(index_params, search_params)
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
    cv2.waitKey(10)

##################################################################################################
#
# Reform Contours - Takes an approximated array of 4 pairs of coordinates and puts them in the order
# TOP-LEFT, TOP-RIGHT, BOTTOM-RIGHT, BOTTOM-LEFT
#
##################################################################################################
def ReformContours(contours):
        contours = contours.reshape((4,2))
        contoursnew = np.zeros((4,2),dtype = np.float32)
 
        add = contours.sum(1)
        contoursnew[0] = contours[np.argmin(add)]
        contoursnew[2] = contours[np.argmax(add)]
         
        diff = np.diff(contours,axis = 1)
        contoursnew[1] = contours[np.argmin(diff)]
        contoursnew[3] = contours[np.argmax(diff)]
  
        return contoursnew

##################################################################################################
#
# FindSymbol
#
##################################################################################################

def FindSymbol(ThresholdArray):

    TargetData = -1
    SymbolFound = -1
    time.sleep(0.1)#let image settle
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #convert img to HSV and store result in imgHSVyellow
    lower = np.array([ThresholdArray[0],ThresholdArray[1],ThresholdArray[2]]) #np arrays for upper and lower thresholds
    upper = np.array([ThresholdArray[3], ThresholdArray[4], ThresholdArray[5]])

    imgthreshed = cv2.inRange(imgHSV, lower, upper) #threshold imgHSV

    contours, hierarchy = cv2.findContours(imgthreshed,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
    
    for x in range (len(contours)):
        contourarea = cv2.contourArea(contours[x]) #get area of contour
        if contourarea > 1000: #Discard contours with a small area as this may just be noise
            arclength = cv2.arcLength(contours[x], True)
            approxcontour = cv2.approxPolyDP(contours[x], 0.08 * arclength, True) #Approximate contour to find square objects
            if len(approxcontour) == 4: #if approximated contour has 4 corner points
                if hierarchy[0][x][2] != -1: #if contour has a child contour, which is image in centre of border
                    #find centre point of target
                    rect = cv2.minAreaRect(contours[x])
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
                    boxcentrex = int(rect[0][0])
                    boxcentrey = int(rect[0][1])

                    #correct perspective of found target and output to image named warp      
                    reformedcontour = ReformContours(approxcontour) #make sure coordinates are in the correct order
                    dst = np.array([[0,0],[300,0],[300,300],[0,300]],np.float32)
                    ret = cv2.getPerspectiveTransform(reformedcontour,dst)
                    warp = cv2.warpPerspective(img,ret,(300,300))
                    cv2.imshow("transform", warp)
                    cv2.waitKey(10)

                    #Try and match image to known targets
                    Keypoints, Descriptors = detector.detectAndCompute(warp, None)
                    if Descriptors is None:
                        print "Error - No descriptors found in image"
                    else:
                        matches = matcher.knnMatch(HomeSymbolDescriptors, Descriptors, 2)
                        HomeMatches = []
                        for match in matches:
                            if match[0].distance < match[1].distance * 0.7:
                                HomeMatches.append(match)
                        
                        matches = matcher.knnMatch(FoodSymbolDescriptors, Descriptors, 2)
                        FoodMatches = []
                        for match in matches:
                            if match[0].distance < match[1].distance * 0.7:
                                FoodMatches.append(match)

                        if HomeMatches <= 8 and FoodMatches <= 8: #Not enough matches for either symbol
                            SymbolFound = -1
                        else:
                            if len(HomeMatches) == len(FoodMatches):
                                print (Fore.GREEN + "Unsure of symbol type")
                            elif len(HomeMatches) > len(FoodMatches):
                                SymbolFound = "HOME"
                            else:
                                SymbolFound = "FOOD"
                        print (Fore.GREEN + "Home Matches - " + str(len(HomeMatches)) + " - Food Matches - " + str(len(FoodMatches)))
                        print (Fore.GREEN + "Symbol Found - " + str(SymbolFound))
   
                    #Find lengths of the 4 sides of the target
                    leftedge = reformedcontour[3][1] - reformedcontour[0][1]
                    rightedge = reformedcontour[2][1] - reformedcontour[1][1]
                    topedge = reformedcontour[1][0] - reformedcontour[0][0]
                    bottomedge = reformedcontour[2][0] - reformedcontour[3][0]

                    #Find approximate distance to target
                    if leftedge > rightedge:
                        LongestSide = leftedge
                    else:
                        LongestSide = rightedge
                    if topedge > LongestSide:
                        LongestSide = topedge
                    if bottomedge > LongestSide:
                        LongestSide = bottomedge
                    Distance = (640.00*14)/LongestSide #focal length x Actual Border width / size of Border in pixels
                    print (Fore.GREEN + "Distance= " + str(Distance))

                    #Find which way symbol is facing and width of target to gauge angle
                    EdgeDifference = leftedge - rightedge
                    if EdgeDifference > 0:
                        print (Fore.GREEN + "Symbol is to the robots left")
                        SymbolLocation = "LEFT"
                    elif EdgeDifference == 0:
                        print (Fore.GREEN + "Symbol is dead ahead")
                        SymbolLocation = "AHEAD"
                    else:
                        print (Fore.GREEN + "Symbol is to the robots right")
                        SymbolLocation = "RIGHT"

                    width = (topedge + bottomedge) / 2
                    height = (leftedge + rightedge) / 2
                    whratio = width / height
                    print (Fore.GREEN + "W/H Ratio = " + str(whratio))
                    #time.sleep(1)

                    #draw box around target and a circle to mark the centre point
                    cv2.drawContours(img,[approxcontour],0,(0,0,255),2)
                    cv2.circle(img, (boxcentrex, boxcentrey), 5, (0,0,255),-1) #draw a circle at centre point of object
                    TextForScreen = "Approx. Distance: " + "%.2f" % Distance + "cm"
                    cv2.putText(img,TextForScreen, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0),1)

                    if SymbolFound != -1: #If a symbol has been found
                        #write symbol type to screen
                        TextForScreen = "Found: " + str(SymbolFound)
                        cv2.putText(img,TextForScreen, (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0),1)
                        #Only return data is a target has been identified
                        TargetData = [boxcentrex, boxcentrey, Distance, SymbolFound, SymbolLocation, whratio]
                        break
                        
          
    if DisplayImage is True:
        cv2.imshow("camera", img)
        cv2.waitKey(10)

    return TargetData





##################################################################################################
#
# Check Ground
#
##################################################################################################

def CheckGround():

    StepSize = 8
    EdgeArray = []

    time.sleep(0.1)#let image settle
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    ret,img = capture.read() 
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #5 seems to be enough

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)   #convert img to grayscale and store result in imgGray
    imgGray = cv2.bilateralFilter(imgGray,9,30,30) #blur the image slightly to remove noise             
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
        cv2.waitKey(10)


##################################################################################################
#
# NewMap - Creates a new map
#
##################################################################################################

def NewMap(MapWidth, MapHeight):

    MapArray = np.ones((MapHeight,MapWidth,3), np.uint8)
    MapArray[:MapWidth] = (255,255,255)      # (B, G, R)
    
    
    return MapArray

##################################################################################################
#
# AddToMap - 
#
##################################################################################################

def AddToMap(MapArray,X,Y,Type):
    
    Width = MapArray.shape[1]
    Height = MapArray.shape[0]
    print Type, "in AddToMap"

    if Type == 'FOOD':      
        cv2.circle(MapArray, (X, Y), 2, (0,0,255),-1) #draw a circle
    elif Type == 'HOME':      
        cv2.circle(MapArray, (X, Y), 2, (0,255,0),-1) #draw a circle

    return MapArray
    
##################################################################################################
#
# ShowMap - Displays a map in an opencv window
# MapArray is a numpy array where each element has a value between 0 and 1
#
##################################################################################################

def ShowMap(MapArray):
    
    cv2.imshow("map", MapArray)
    cv2.waitKey(50)




def destroy():
    
    cv2.destroyAllWindows()
   


