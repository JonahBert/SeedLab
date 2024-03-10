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
#fullFOV = 57.154316234 - 4.67
fullFOV = 53.1
halfFOV = fullFOV / 2

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
                lcd.clear()
                message = "No Markers"
                lcd.message = message
            else:
                message = "Marker at angle:\n" + str(angle)
                lcd.message = message

#start conditional            
myThread = threading.Thread(target=writeToLCD,args=())
myThread.start()

#declare variables
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
    angle = 1000
    
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
                angle = round(angle,3)
                
    if angle <= prevAngle - 0.05 or angle >= prevAngle + 0.05:
        q.put(angle)
        prevAngle = angle

                                       

    
cv2.destroyAllWindows()
camera.release()
    

    
