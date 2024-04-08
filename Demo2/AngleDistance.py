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
import struct
import queue
import board
import cv2
from cv2 import aruco
from smbus2 import SMBus
from time import sleep



# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

# Initialize SMBus library with I2C bus 1
i2cARD = SMBus(1)

#create instance of aruco library
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)

#exposure value
exp_val = -7

#set dimensions and parameters
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
#camera.set(cv2.CAP_PROP_EXPOSURE, exp_val)

#camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
#camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
camera.set(cv2.CAP_PROP_EXPOSURE, exp_val)
centerX = 640//2
centerY = 480//2

#centerX = 1280//2
#centerY = 720//2

#height of marker in pixels 1 foot away from camera
heightAt1ft = 105

#according to datasheet the field of view is 68.5 Degrees diagonally
#initialize Fov
#fullFOV = 53.1

#fullFOV = 63
fullFOV = 51.6
halfFOV = fullFOV / 2

#declare variables
prevAngle = 0
prevDistance = 0
angle = 0
distance = 0
wait = 50

while True:
    wait += 1
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

    #variables
    marker = 0
    instruction = 0
    distance = 0
    angle = 0
    
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
                
                #angle correction
                if angle > 0:
                    angle = angle*0.95
                elif angle < 0:
                    angle = angle*0.85
                    
                angle = round(angle,3)
                #print(angle)
                
                #calculate height and use ratios to calculate distance
                height = newCorners[1][1] - newCorners[0][1]
                if height != 0:
                    distance = heightAt1ft / height
                distance = round(distance,3)

                #set marker detected flag
                marker = 1

        #pack floats for angle and distance
        anglePacked = list(struct.pack('!f', float(angle)))
        distancePacked = list(struct.pack('!f', float(distance)))
        #anglePacked = [ord(character) for character in str(angle)]
        #distancePacked = [ord(character) for character in str(distance)]

        #place values in list to be indexed later
        command = [marker]
        for i in anglePacked:
            command.append(i)
        for i in distancePacked:
            command.append(i)
        if wait > 3 and marker:
            try:
                i2cARD.write_i2c_block_data(ARD_ADDR, 0, command)
                print("DATA SENT SUCCESFULLY: " + str(command))
                print("Angle: " + str(angle))
                #print("Distance: " + str(distance))
            except:
                print("Error" + str(angle))
            wait = 0


  
cv2.destroyAllWindows()
camera.release()
