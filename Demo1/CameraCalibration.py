"""
Demo 1: Obtain camera matrix and distortion ceofficients (CODE NOT USED FOR DEMO 1)
Authors: Hunter Burnham, Joseph Kirby, Jonah Bertolino
Resources: Open cv camera calibration tutorial https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
Date Started: 2/28/2024
Date completed: 3/10/2024
Description:
"""
import numpy as np
import cv2
import glob
import math


# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)

#set dimensions
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#establish folder path
folderName = '/home/seedlab/Demo1/CalibrationImgs/'
images = glob.glob(folderName + '*.jpg')

count = 0
print('Press t to start')
while count < 10:
    ret, image = camera.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow('overlay',gray)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("t"):
        print('Checking for Pattern')
        while True:
            ret, image = camera.read()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cv2.imshow('overlay',gray)
            #this wait key statement makes the overlay showing work
            cv2.waitKey(1)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                #append points
                objpoints.append(objp)
                cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners)
                print("Corners Found!")
                
                # Draw and display the corners
                cv2.drawChessboardCorners(image, (7,6), corners,ret)
                cv2.imshow('img',image)
                cv2.waitKey(1000)
                count += 1
                print('Adjust position, then press t to continue')
                break
        
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

if ret == False:
    print("Calibration failed") 

print(mtx)
print(dist)
fovX = 2*math.atan(640/(2*mtx[0][0]))*180/math.pi

print(fovX)
print(count)
np.save('CameraMatrix.npy',mtx)
np.save('distortionVec.npy',dist)

cv2.destroyAllWindows()
camera.release()
