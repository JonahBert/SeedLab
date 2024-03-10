"""
Demo 1: Detect angle of aruco markers (CODE USED FOR DEMO 1)
Authors: Joseph Kirby, Jonah Bertolino, Hunter Burnham
Resources: https://mavicpilots.com/threads/computing-horizontal-field-of-view-fov-from-diagonal-fov.140386/ 
Date Started: 2/28/2024
Date completed: 3/10/2024
Description: Within this file, we are taking our aruco detection from previous projects and using it to now calculate the angle from the center of the camera in the 
    x direction.
    We are integrating both the LCD screen threading, queue, and board to display the camera angles from the camera detecting the aruco marker.
    We configured the angle to be positive when it is on the left of the screen and oppositely when it is on the right being a negative angle.
    The angles we have calculated are in degrees.
"""
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

#according to datasheet the field of view is 68.5 Degrees diagonally
#initialize Fov
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

#declare variables
prevAngle = 0

while True:
    #take picture
    ret, frame = camera.read()
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    cv2.imshow("overlay",grey)

    #run until keypress
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    
    #detect markers
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
    angle = 1000
    
    #if marker is detected, calculate center
    if corners is not None:
        #itirate through each point recorded in corners
        for i in range(len(corners)):
            markerDetects = corners[i]
            if len(markerDetects == 4):
                #sets the variable newCorners to all of the corner values from the aruco marker when it is detected.
                newCorners = corners[0][0]
                #The xCoord variable stores all of the x value coordinates and divides them by 4 to get the center x value.
                xMarker = (newCorners[0][0] + newCorners[1][0] + newCorners[2][0] + newCorners[3][0]) / 4
                #The yCoord variable stores all of the y value coordinates and divides them by 4 to get the center y value.
                yMarker = (newCorners[0][1] + newCorners[1][1] + newCorners[2][1] + newCorners[3][1]) / 4

                #The deltaX variable takes the difference between the center of the aruco marker and the center of the camera to get the total distance between them.
                deltaX = xMarker - centerX

                #The angle calculation of the arcuo marker to the center of the camera.
                #We are taking the half of the fov to split the screen into positive and negative and using the ratio between the difference of X and the center to calculate the total angle away from the center in the x direction. 
                angle = -1 * halfFOV * (deltaX / centerX)
                angle = round(angle,3)

    #if new angle within a 0.05 of the previous, add to the queue
    #we dont want screen constantly refreshing
    if angle <= prevAngle - 0.05 or angle >= prevAngle + 0.05:
        #clear queue if angle changes
        q.queue.clear()
        q.put(angle)
        prevAngle = angle
   
cv2.destroyAllWindows()
camera.release()
    

    
