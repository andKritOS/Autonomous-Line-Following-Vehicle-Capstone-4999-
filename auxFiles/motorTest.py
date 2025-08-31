import gpiozero as gpio
import board
import time

motorBR = gpio.Motor(5,6,12,True,None)

SpeedTrck = 0

while (1):
    while(SpeedTrck < 1023):
        motorBR.forward(SpeedTrck/1023) #setspeed forward
        time.sleep(0.1)
        SpeedTrck += 1
    while(SpeedTrck > 0):
        motorBR.forward(SpeedTrck/1023) #setspeed forward
        time.sleep(0.1)
        SpeedTrck -= 1
    