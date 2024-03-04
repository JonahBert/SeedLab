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
centerX = 640//2
centerY = 480//2
fullFOV = 57.154316234
halfFOV = fullFOV / 2

#cv2.drawMarker(camera, (320,240), color=[0,0,0], markerType = cv2.MARKER_CROSS, thickness = 1)
#global message variable for threading
message = "No markers"

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

#start conditional            
myThread = threading.Thread(target=writeToLCD,args=())
myThread.start()

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
    
    #if marker is detected, calculate center
    if corners is not None:
        detected = False
        #itirate through each point recorded in corners
        for i in range(len(corners)):
            markerDetects = corners[i]
            if len(markerDetects == 4):
                #find center of the marker
                newCorners = corners[0][0]
                xCoord = (newCorners[0][0] + newCorners[1][0] + newCorners[2][0] + newCorners[3][0]) / 4
                yCoord = (newCorners[0][1] + newCorners[1][1] + newCorners[2][1] + newCorners[3][1]) / 4
                xMarker = xCoord
                yMarker = yCoord
                detected = True
                deltaX = xMarker - centerX
                deltaY = yMarker - centerY

                #Use similar triangles to calculate distance away given angle
                #according to datasheet the field of view is 68.5 Degrees diagonally
                #The horizontal degree value is 57.154316234 Degrees

                #Left side of the screen
                
                angle = halfFOV * (deltaX / centerX)
                print("Angle is: ", angle)
                    
                                       
                
                
    else:
        #no markers detected
        angle = 0

    

    
