#!/usr/bin/env python

import numpy as np
import cv2

print "Starting OpenCV"
capture = cv2.VideoCapture(0)

capture.set(3,640) #1024 640
capture.set(4,480) #600 480

cv2.namedWindow("camera", 0) 
print "Creating OpenCV windows"
cv2.waitKey(200)
cv2.resizeWindow("camera", 600,600) 
print "Resizing OpenCV windows"
cv2.waitKey(200)
cv2.moveWindow("camera", 500,30)
print "Moving OpenCV window"
cv2.waitKey(200)

Successful = 0

pattern_size = (9, 6)
pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)


obj_points = []
img_points = []
h, w = 0, 0

for x in range(40):
    
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent

    img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    cv2.imshow('camera',img)
    cv2.waitKey(200)
    h, w = img.shape[:2]
    found, corners = cv2.findChessboardCorners(img, pattern_size)

    if found:
        term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
        corners2 = cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
        Successful += 1
                  
    if not found:
        print 'chessboard not found'
        continue

    img_points.append(corners.reshape(-1, 2))
    obj_points.append(pattern_points)

    print 'ok'
    imgrgb = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    cv2.drawChessboardCorners(imgrgb, (9, 6), corners, found)
    cv2.imshow('camera',imgrgb)
    cv2.waitKey(200)
        
if Successful > 30:
    print "Processing results. Please wait..."
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    print "RMS:", rms
    print "camera matrix:\n", camera_matrix
    print "distortion coefficients: ", dist_coefs.ravel()
else:
    print "Not enough successful images found"
cv2.destroyAllWindows()







