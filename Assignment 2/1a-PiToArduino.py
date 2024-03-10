#1a-PiToArduino.py: Python side of Assignment 1a code
#Authors: Hunter Burnham, Joseph Kirby
#Resources: N/A
#Date Created: 1/25/2024
#Date Completed: 2/8/2024
#Description: Asks the user for an string input, then sends it to the Arduino.

from smbus2 import SMBus
from time import sleep

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)

# Do in a loop
while(True):
    
    # Get user input for offset
    offset = int(input("Enter an offset (7 to quit): "))
    
    # Provide an exit key
    if(offset == 7):
        break
    
    # Get user input for command
    string = input("Enter a string of 32 characters or less:")
    
    # Write a byte to the i2c bus
    command = [ord(character) for character in string]
    
    try:
        #ask the arduino to take on encoder reading
        i2c.write_i2c_block_data(ARD_ADDR,offset,command)
    except IOError:
        print("Could not write data to the to the Arduino.")
