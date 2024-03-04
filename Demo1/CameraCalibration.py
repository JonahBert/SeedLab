#Demo 1: Detect quadrant the marker is in, display on LCD using threading, send data to arduino
import threading
import queue
import board
import cv2
import glob
from cv2 import aruco
import numpy as np
from smbus2 import SMBus
from random import random
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import math

# Initialise I2C bus.
i2cLCD = board.I2C()

#initialize threading queue
q = queue.Queue()

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

# Initialize SMBus library with I2C bus 1
i2cARD = SMBus(1)

#create instance of aruco library
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)

#set dimensions
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

def Calibrate(camera):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
     
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((5*5,3), np.float32)
    objp[:,:2] = np.mgrid[0:5,0:5].T.reshape(-1,2)
     
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
     
    #establish folder path
    folderName = '/home/seedlab/Demo1/CalibrationImgs/'

    #take 10 images for test patterns
    for i in range(0,10):
        #sleep so time to move board
        sleep(1)
        ret, image = camera.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        fileName = folderName + 'Image' + str(i) + '.jpg'
        #print(fileName)
        try:
            cv2.imwrite(fileName, image)
        except:
            print("Could not save " + fileName)
            pass

    images = glob.glob(folderName + '*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        ret, frame = camera.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        cv2.imshow("Overlay", gray)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (5,5),None)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            #cv2.drawChessboardCorners(frame, (7,6), corners,ret)
            #cv2.imshow('img',frame)
            #cv2.waitKey(500)
            break

    ######################
    #Calibration
    ######################
    #check if calibration works, throw exception if it doesnt
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    if ret == False:
        print("Calibration failed")
        return  
    print(mtx)
    print(dist)
    print(tvecs)
    return mtx, dist

mtx, dist = Calibrate(camera)


fovX = np.arctan2(640,(2*mtx[0][0]))
degres = ((fovX * 180) / (math.pi))
print(degres)
