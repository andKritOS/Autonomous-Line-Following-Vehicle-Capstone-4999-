import gpiozero as gpio #gpio handling library
import board #PI gpio access
import adafruit_blinka #circuit python compatability for RGB sensor libraries
import adafruit_tcs34725 #RGB sensors
import adafruit_tca9548a #multiplexer
from time import sleep

#setup I2C connection
i2c = board.I2C() #creates I2C bus 
tca = adafruit_tcs34725.TCS34725(i2c) #creates a singular device

#create color sensor objects

rgbLeft = adafruit_tcs34725.TCS34725(tca[0])
rgbCenter = adafruit_tcs34725.TCS34725(tca[1])
rgbRight = adafruit_tcs34725.TCS34725(tca[2])

pinAsgn = {
    #------------------COLOR_SENSOR------------------
    #declares I2C communication
    "Select_A0": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
    "Select_A1": (23,0,0), 
    "Select_A2": (27,1,0),
    "I2CReset": (10,0,0),
    "I2C_SDA": (22,1,0),
    "I2C_SCL": (24,0,0),
}

def readColorSensors():
    adafruit_tcs34725.TCS34725() #you may need this if you want to use the multiplexer
    s.sensordata["ultSonic"][0] = rgbLeft.color_raw #THIS FUNCTION SELECTS THE CURRENT DEVICE YOU WANT TO SEND COMMANDS


#Left
clrSens_L = rgbLeft.color
gpio.Device()

#Right
clrSens_R = rgbRight.color

#program start

while(1):
    sleep (1)
    print("Blinking Lights. \n")
    sleep (1)
    print("Left RGB Sensor: {}".format(clrSens_L), end = "/n")
    print("Right RGB Sensor: {}".format(clrSens_L), end = "/n")
