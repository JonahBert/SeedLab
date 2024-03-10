#Demo 1: Detect quadrant the marker is in, display on LCD using threading, send data to arduino
import threading
import queue
import board
import cv2
import glob
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

#load in camera matrix and distortion vectors
mtx = np.load('CameraMatrix.npy')
dst = np.load('distortionVec.npy')
print(mtx)
def writeToLCD():
    # Initialize LCD
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, lcd_columns, lcd_rows)
    #intialize message
    message = "No markers"
    msgPre = "Marker at angle:\n"
    lcd.message = message
    
    while True:
        if not q.empty():
            #get angle from qeue
            angle = q.get()
            message = msgPre + str(angle)
            lcd.clear()
            lcd.message = message


#start conditional            
myThread = threading.Thread(target=writeToLCD,args=())
myThread.start()

#cv2.drawMarker(camera, (320,240), color=[0,0,0], markerType = cv2.MARKER_CROSS, thickness = 1)

###physical marker length in m
markerLength = 5/100

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
    if corners is not None:
        #itirate through each point recorded in corners
        #for i in range(len(corners)):
        aruco.drawDetectedMarkers(grey,corners,ids)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dst)
        #print(rvecs)
        #print(tvecs)
        if tvecs is not None:
            #tmtx = cv2.Rodrigues(tvecs)
            #alpha = np.arctan2(rmtx[0][1][0],rmtx[0][0][0])*180/m.pi
            #beta = np.arctan2(-rmtx[0][2][0],m.sqrt(rmtx[0][2][1]**2 + rmtx[0][2][2]**2)) *180/m.pi
            #gamma = np.arctan2(rmtx[0][2][1],rmtx[0][2][2])*180/m.pi
            #print(gamma)
            angle = np.arctan2(tvecs[0][0][0], tvecs[0][0][2])*180/m.pi
            print(angle)
        
cv2.destroyAllWindows()
camera.release()
