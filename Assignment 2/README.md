This folder contains all of the code and content used for Assignment 2. 
During the course of Assignment 2, our group completed multiple objectives. These objectives included capturing and storing an image on the Raspberry Pi, using Open CV to process images, detect markers, and determine the angle and distance to markers, using the Arudino IDE to download a program, explaining how the I2C protocol works, writing a program that implements I2C functions on the Raspberry Pi and Arduino, using I2C to commicate between the Raspberry Pi and Arduino, and using I2C to commmunicate with another device and the Raspberry Pi.

Py group completed four different tasks, in Task 1 the Pi sent a string to the Arduino, which led to the Arduino displaying a string of characters followed by ASCII codes. In Task 2 the user input a number between 0 and 100, which the Pi sent to the Arduino, leading the Arduino to add 100 to that number and sent it back to the Pi. The Pi then displayed the number on the LCD screen. In Task 3, we took a picture using the camera, converted it to grayscale, and then ran Aruco detection code. When no markers were foubd, the LCD screen displayed "No markers found." Otherwise, it displayed the ID of the Aruco marker. In addition, we also answered questions on how our code worked. In Task 4, we took a picture of colors.pdf on a different monitor using the camera. On this image, we performed morphological transformations and then displayed the detected shape using a contour.

A short description of each file is provided below:

README.md: Contains a description of Assignment 2 and all tasks completed. Also contains a short description on every file in Assignment 2 and all resources used.

1a-PiToArduino.py: Asks the user for an string input, then sends it to the Arduino.

1b-SendAndReceive.py: Asks the user for an integer between 0 and 100, then sends that integer to the Arduino. Afterwards, displays received integer from Arduino.

2a-arucoDetection.py: Takes a picture using the camera, converts it to grascale, then runs Aruco detection. The LCD screen displays the ID of the marker, but if one isn't found it displays "No markers found."

1a_PiToArduino.ino: Displays received string from Pi.

1b-SendAndReceive.ino: Adds 100 to an integer received from the Pi, then sends it back.

Assignment 2.docx: Contains all documentation for Assignment 2.

