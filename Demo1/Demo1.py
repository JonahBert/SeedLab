#Mini Project: Detect quadrant the marker is in, display on LCD using threading, send data to arduino
import threading
import queue
import board
import cv2
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

def Calibrate(camera):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    #images = glob.glob('*.jpg')
    
    #for fname in images:
    while True:
        #img = cv2.imread(fname)
        ret, frame = camera.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        cv2.imshow("Overlay", gray)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(frame, (7,6), corners,ret)
            cv2.imshow('img',frame)
            cv2.waitKey(500)


            ######################
            #Calibration
            ######################

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
            return mtx, dist, rvecs, tvecs
def writeToLCD():
    # Initialize LCD
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, lcd_columns, lcd_rows)
    #intialize message
    message = "No markers"
    lcd.message = message
    
    while True:
        if not q.empty():
            #get quadrant from qeue
            quadrant = q.get()
            
            #check quadrant
            if quadrant == 0:#zero means no markers detected
                message = "No markers"
                
            elif quadrant <= 4 and quadrant >= 1:
                #convert to binary string and subtract 1 to get into correct form
                binaryQuad = format(quadrant - 1,'02b')
                message = f"Desired Output:\n[{binaryQuad[0]},{binaryQuad[1]}]"
                
            else:#this shouldnt happen
                message = "error"
                
            lcd.clear()
            lcd.message = message

#wait for key, then calibrate camera
mtx, dist, calRvecs, calTvecs = Calibrate(camera)

#start conditional            
myThread = threading.Thread(target=writeToLCD,args=())
myThread.start()

#cv2.drawMarker(camera, (320,240), color=[0,0,0], markerType = cv2.MARKER_CROSS, thickness = 1)
quadrant = 0
prevQuadrant = 0

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
    #itirate through each point recorded in corners
    for i in range(len(corners)):
        markerDetects = corners[i]
        if len(markerDetects == 4):
            #find center of the marker
            newCorners = corners[0][0]
            markerLength = abs(newCorners[0][0] - newCorners[1][0])
    rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners,markerLength,mtx, dist, rvecs, tvecs)
    print(rvecs)
    print(tvecs)



