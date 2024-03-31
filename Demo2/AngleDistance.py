"""
Demo 2: Detect angle of aruco markers and distance
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
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


# Initialise I2C bus.
i2cLCD = board.I2C()

#initialize threading queue
qAngle = queue.Queue()
qDistance= queue.Queue()
#make max size once to keep qeue filling up with past values that are no longer relevant i.e. the marker moving but old values are still in queue
qAngle = queue.Queue(maxsize=1)
qDistance = queue.Queue(maxsize=1)  


# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

# Initialize SMBus library with I2C bus 1
i2cARD = SMBus(1)

#create instance of aruco library
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)

#exposure value
exp_val = -9

#set dimensions and parameters
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
camera.set(cv2.CAP_PROP_EXPOSURE, exp_val)
centerX = 640//2
centerY = 480//2

#height of marker in pixels 1 foot away from camera
heightAt1ft = 105

#according to datasheet the field of view is 68.5 Degrees diagonally
#initialize Fov
fullFOV = 53.1
halfFOV = fullFOV / 2

def writeToLCDandARD():
    # Initialize LCD
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, lcd_columns, lcd_rows)

    #intialize message
    message = "No Markers"
    lcd.message = message

    #initialize variables
    angle = 0
    distance = 0

    #while loop
    while True:
        marker = False
        #get instruction from arduino, tells what to send
        instruction = i2cARD.read_byte_data(ARD_ADDR, 0)
        if not qAngle.empty() and not qDistance.empty():
            marker = True
            angle = qAngle.get()
            distance = qDistance.get()
        if marker == False:
            message = "No Markers"
        else:
            message = "Angle: " + str(angle) + "\nDist: " + str(distance)
        lcd.message = message
        #create list of different commands
        command = [marker, float(angle), float(distance)]
        instruction = i2cARD.read_byte(ARD_ADDR)
        if instruction != 0:
            try:
                #ask the arduino to take on encoder reading
                #index command list with instruction to send proper value (more elegant than state machine i think)
                i2cARD.write_block_data(ARD_ADDR, 0, command[instruction - 1])
            except IOError:
                print("Could not write data to the to the Arduino.")

#start conditional            
myThread = threading.Thread(target=writeToLCDandARD,args=())
myThread.start()

#declare variables
prevAngle = 0
prevDistance = 0
angle = 0
distance = 0

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

                #calculate height
                height = newCorners[1][1] - newCorners[0][1]
                distance = heightAt1ft / height
                distance = round(distance,3)

                #qAngle.queue.clear()
                qAngle.put(angle)
                prevAngle = angle
                
                #qDistance.queue.clear()
                qDistance.put(distance)
                prevDistance = distance
                
   
cv2.destroyAllWindows()
camera.release()