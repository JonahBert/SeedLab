"""
Demo 1: Find angle of marker from camera axis using pose estimation (CODE NOT USED FOR DEMO 1)
Authors: Hunter Burnham, Joseph Kirby, Jonah Bertolino
Resources:  Open cv aruco pose estimation tutorial https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
            Translation vector wikipedia https://en.wikipedia.org/wiki/Translation_(geometry)
Date Started: 3/8/2024
Date completed: 3/10/2024
Description: Using camera matrix and distortion coefficients from cameraCalibration.py to complete the task for demo 1 using the 
    aruco pose estimation function form open cv. This outputs a translation vector that should be able to be used to compute the angle from the 
    camera axis. This method was deemed infeasible due to our inability to get consistent camera calibration
"""
import threading
import queue
import board
import cv2
import math as m
from cv2 import aruco
import numpy as np
from smbus2 import SMBus
from random import random
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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

#load in camera matrix and distortion vectors obtained from CameraCalibration.py
mtx = np.load('CameraMatrix.npy')
dst = np.load('distortionVec.npy')

def writeToLCD():
    # Initialize LCD
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, lcd_columns, lcd_rows)
    #intialize message
    message = "No Markers"
    lcd.message = message 
    while True:
        if not q.empty():
            angle = q.get()
            #no markers if angle = 1000
            if angle == 1000:
                #clear lcd if no angle detected
                lcd.clear()
                message = "No Markers"
                lcd.message = message
            else:
                #display andle
                message = "Marker at angle:\n" + str(angle)
                lcd.message = message


#start conditional            
myThread = threading.Thread(target=writeToLCD,args=())
myThread.start()


#physical marker length in m
markerLength = 5/100
prevAngle = 0

while True:
    # Marker Detection currently simulated by inputting an integer
    ret, frame = camera.read()
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    cv2.imshow("overlay",grey)
    
    #take pictures until keypress
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    
    #detect markers
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

    #angle of 1000 is attributed to no marker detected for LCD purposes
    angle = 1000

    if corners is not None:
        #itirate through each point recorded in corners
        #for i in range(len(corners)):
        aruco.drawDetectedMarkers(grey,corners,ids)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dst)
        if tvecs is not None:
            #use x and z component of translation vector to compute desired angle
            angle = np.arctan2(tvecs[0][0][0], tvecs[0][0][2]) * 180 / m.pi
            print(angle)

    #if new angle within a 0.05 of the previous, add to the qeue
    #we dont want screen constantly refreshing
    if angle <= prevAngle - 0.05 or angle >= prevAngle + 0.05:
        #clear queue if angle changes
        q.queue.clear()
        q.put(angle)
        prevAngle = angle

        
cv2.destroyAllWindows()
camera.release()
