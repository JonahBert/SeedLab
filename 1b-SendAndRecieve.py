from smbus2 import SMBus
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)

#get user input for integer, ensure between 0 and 100
while(True):
    integer = int(input("Enter an integer between 0 and 100: "))
    if integer < 0 or integer > 100:
        print("integer was not between 0 and 100")
    else:
        break

#send to arduino
command = integer

try:
    #ask the arduino to take on encoder reading
    i2c.write_byte_data(ARD_ADDR,0,command)
except IOError:
    print("Could not write data to the to the Arduino.")

sleep(0.1)

#requet byte from arduino
reply = i2c.read_byte_data(ARD_ADDR, 0)
print("Recieved from Arduino: " +str(reply))

#Display on LCD  
lcd.clear()
lcd.message = "Recieved: " +str(reply)

