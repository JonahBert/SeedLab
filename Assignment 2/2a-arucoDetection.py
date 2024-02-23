import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Initialise I2C bus.
i2c = board.I2C()

# Initialize LCD
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#create instance of aruco library
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)

#set dimensions
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

#initialize variables
preMessage = ""

# Let the camera warmup
sleep(0.1)

# Get an image from the camera stream
while(True):
    ret, frame = camera.read()
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    cv2.imshow("overlay",grey)
    #take pictures until keypress
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break

    #detect markers
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

    #create message to display
    if len(corners) > 0:
        numIds = len(ids)
        message = "Aruco ID's found:\n"
        for i in range(numIds):
            message += str(ids[i][0])
            message += " "
    else:
        message = "No aruco markers detected"
        
        
    # Set LCD color to red and display message
    lcd.color = [100, 0, 0]

    #clear lcd if message is different
    if message != preMessage:
        lcd.clear()
    preMessage = message

    #display message
    lcd.message = message

cv2.destroyAllWindows()
lcd.clear()
