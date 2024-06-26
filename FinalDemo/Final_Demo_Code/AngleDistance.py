"""
Final Demo: Detect angle of aruco markers and distance
Authors: Joseph Kirby, Jonah Bertolino, Hunter Burnham
Resources: https://mavicpilots.com/threads/computing-horizontal-field-of-view-fov-from-diagonal-fov.140386/ 
Date Started: 3/29/2024
Date completed: 4/29/2024
Description: This program continuously searches for Aruco markers and calaculates the horizontal angle from the camera axis and the distance from
the camera of the marker. It then packs the data into a struct to convert each float value into 4 bytes. It thens sends the bytes from the distance
and angle calculations plus one more that says wether or not a marker is detected and then sends the data to the arduino using i2c.
Pin Connections: Connnect pins A5,A6 and GND on Arduion to the i2c pins on the PI
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
camera.set(cv2.CAP_PROP_EXPOSURE, exp_val)
centerX = 640//2
centerY = 480//2

#height of marker in pixels 1 foot away from camera
heightAt1ft = 110

#according to datasheet the field of view is 68.5 Degrees diagonally
fullFOV = 51.6
halfFOV = fullFOV / 2

#declare variables
angle = 0
distance = 0
closestFound = False


while True:
    #take picture
    ret, frame = camera.read()
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    #cv2.imshow("overlay",grey)

    #run until keypress
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    
    #detect markers
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

    #variables
    marker = 0
    instruction = 0
    angle = 0
    markerVals = []
    #if marker is detected, calculate center
    if corners is not None:
        #itirate through each point recorded in corners
        for i in range(len(corners)):
            markerDetects = corners[i]
            if len(markerDetects == 4):
                #sets the variable newCorners to all of the corner values from the aruco marker when it is detected.
                newCorners = corners[i][0]
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

                
                #calculate height and use ratios to calculate distance
                height = abs(newCorners[2][1] - newCorners[0][1])
                if height != 0:
                    distance = heightAt1ft / height
                distance = round(distance,3)
                
                #add distance and angle of each marker to list
                markerVals.append([distance,angle])
                
                marker = 1
                    
                
        if marker:     
            #find closest marker
            minDistance = 100
            desiredIndex = 0
            for i in range(len(markerVals)):
                if markerVals[i][0] <= minDistance:
                    desiredIndex = i
                    minDistance = markerVals[i][0]
                    
            #first marker will be 4 ft away
            if minDistance < 4.5:
                closestFound = True

            #pack floats for angle and distance
            anglePacked = list(struct.pack('!f', float(markerVals[desiredIndex][1])))
            distancePacked = list(struct.pack('!f', float(markerVals[desiredIndex][0])))

            #place values in list to be indexed later
            command = [marker]
            for i in anglePacked:
                command.append(i)
            for i in distancePacked:
                command.append(i)
        
        #if there is a marker send data to arduino
        if marker and closestFound and markerVals[desiredIndex][0] > 1.5:
            try:
                i2cARD.write_i2c_block_data(ARD_ADDR, 0, command)
                print("DATA SENT SUCCESFULLY: " + str(marker) + ' ' + str(markerVals[desiredIndex][0]) + ' ' + str(markerVals[desiredIndex][1]))
            except:
                print("Error" + str(markerVals[desiredIndex][0]) + ' ' + str(markerVals[desiredIndex][1]))
            #sleep to avoid busy waiting
            sleep(0.1)
  
cv2.destroyAllWindows()
camera.release()
