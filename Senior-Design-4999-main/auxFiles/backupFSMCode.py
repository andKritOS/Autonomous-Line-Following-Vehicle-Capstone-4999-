import camera_comp
import interfaceGPIO
import 
import time
class Transition:
    def __init__(self,toState):
        self.toState = toState

    def Execute(self):
        # to add things here
        print("Transitioning now")

class FSM:

    def __init__(self,char,states,transitions, inital):
        self.char = char
        self.states = states #the states available
        self.transitions = {} #list of next states
        self.currentSt = inital
        self.transCond = None

    def setCurrState(self,stateName):
        #sets the current state
        self.currentSt = self.states[stateName]

    def setTransition(self,transName):
        #sets the next state to go to
        self.transCond = self.transitions[transName]

    def Execute(self):
        if(self.transCond):
            self.transCond.Execute()
            self.SetState(self.nextSt)
            self.transCond = None
        self.currentSt.Execute()

class State:
    def __init__(self,FSM):
        self.FSM = FSM
        

#states tuple
states = {
    "Reset", #Initial state, sets global variables, initializes camera and sensors
    "Idle_Stop", #regular car stop, activates for obstacles or intermediate 
    "FollowingLine", 
    "Stop", 
    "IdentifyIntersection",
    "YieldtoLeft",
    "DetermineLight",
    "ExecuteTurn",
    "Emergency" #bumper activation, stops car, changes lights, disconnects motors
    }

initialState = "Reset"

transitions = {
    (): "",

}

fsm = FSM() #constructing main FSM







#---------GPIO INTERFACE CODE---------
#-------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

import gpiozero as gpio #gpio handling library
import board #PI gpio access
import adafruit_blinka #circuit python compatability for RGB sensor libraries
import adafruit_tcs34725 #RGB sensors
import adafruit_tca9548a #multiplexer
from time import sleep

#--------------------[[USER EDITABLE VARIABLES]]-------------------------------

#setup I2C connection
i2c = board.I2C() #creates I2C bus 
tca = adafruit_tcs34725.TCS34725(i2c) #creates a singular device

#create color sensor objects

rgbLeft = adafruit_tcs34725.TCS34725(tca[0])
rgbRight = adafruit_tcs34725.TCS34725(tca[1])

#class  variables
_hardwarePWMFrequency = 200
_softwarePWMFrequency = 200
_ultrasonicThreshDist = 0.3 #float (meters) of the distance when the vehicle should stop behind an obstacle
_servoMaxTurnAngle = 45 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
_servoCalibrationAngle = 0 #angle (degrees) offset for which servo will return upon leaving
_motorMaxSpeed = 3 #speed at which the robot moves forward at full power (meters/sec).
_MotorIncIntervalSeconds = 0.05 #the amount of time (seconds) with which the incremental motion is to wait before checking total distance

#------------[[NON-EDITABLE VARIABLES, DO NOT EDIT PAST THIS LINE]]-------------

#--------------------[[USER EDITABLE VARIABLES]]--------------------------------

# ALL WILL BE SET TO OUTPUTS
# ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
# FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))


pinAsgn = {
    #---------------------MOTORS---------------------
    "cameraGimbalServo": (9,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
    #---------------------LEDS-----------------------
    "frontRGB_Red": (4,1,0), # 
    "frontRGB_Green": (17,1,0), #
    #"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
    #------------------COLOR_SENSOR------------------
    #declares I2C communication
    "Select_A0": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
    "Select_A1": (23,0,0), 
    "Select_A2": (27,1,0),
    "I2CReset": (10,0,0),
    "I2C_SDA": (22,1,0),
    "I2C_SCL": (24,0,0),
    #------------------ULTRASONIC--------------------
    "ultraTrig": (26,1,0), #ultrasonic trigger pin
    "ultraEcho": (16,0,0) #ultrasonic feedback pin
}

# pin assignments

#CAMERA SERVO
camServo = gpio.AngularServoservo(
    pinAsgn["cameraGimbalServo"][0]
    _servoMaxTurnAngle, #angle on reset
    -_servoCalibrationAngle, #min servo angle
    _servoCalibrationAngle, #max servo angle
    0.01, #min pw
    1, #max pw
    0.020, #frame width
    None
)

#FRONT RED LED
fntRed = gpio.LED(pinAsgn["frontRGB_Red"[0]])

#FRONT GREEN LED
fntGreen = gpio.LED(pinAsgn["frontRGB_Green"[0]])

#Color sensors

#Left
clrSens_L = rgbLeft.color

#Right
clrSens_R = rgbRight.color

#Ultrasonic Sensor
ultSon = gpio.DistanceSensor(
    pinAsgn["ultraTrig"][0],
    pinAsgn["ultraEcho"][0]
    #9, #length of queue of read valuespinAsgn
    #4.0, #max readable distance (meters)
    #_ultrasonicThreshDist, #IMPORTANT! THRESHOLD DISTANCE! TRIGGER IN RANGE DISTANCE FOR SENSOR
    #False, #FALSE = report values ONLY after the queue has filled up
    #None #pin factory
)
# -------------------------------BASIC SENSOR POLL FUNCTIONS-----------------------------------

def pollUltrasonic(position):
    
    print (ultSon.value)
    print("Polling Ultrasonic Sensor \n")
# -------------------------------I2C SETUP FUNCTIONS------------------------------------- 

# -------------------------------COMPOUND FUNCTIONS---------------------------------

def resetGimbal():
    camServo.value = _servoCalibrationAngle
    print("Camera gimbal has been reset.\n")

def lookBothWays():
    resetGimbal()
    sleep(0.001)
    camServo.value(-_servoMaxTurnAngle)
    sleep(0.001)    
    pollUltrasonic(0) #polls when angle is turned left
    sleep(0.001)
    camServo.value(_servoCalibrationAngle)
    sleep(0.001)
    pollUltrasonic(1) #polls when angle is centered
    sleep(0.001)
    camServo.value(_servoMaxTurnAngle)
    sleep(0.001)
    pollUltrasonic(1) #polls when angle is centered
    print("Finished looking both ways. \n")

#program start

while(1):
    lookBothWays()
    sleep (1)
    print("Blinking Lights. \n")
    sleep (1)
    print("Left RGB Sensor: {}".format(clrSens_L), end = "/n")
    print("Right RGB Sensor: {}".format(clrSens_L), end = "/n")

#-----------------------------------------------------------------------------------

#---------GPIO INTERFACE CODE---------
#-------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

import gpiozero as gpio
import sensorData
import board
import adafruit_tcs34725
from time import sleep

#--------------------[[USER EDITABLE VARIABLES]]-------------------------------

#class  variables
_hardwarePWMFrequency = 200
_softwarePWMFrequency = 1000
_ultrasonicThreshDist = 0.3 #float (meters) of the distance when the vehicle should stop behind an obstacle
_servoMaxTurnAngle = 45 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
_servoCalibrationAngle = 0 #angle (degrees) offset for which servo will return upon leaving
_motorMaxSpeed = 3 #speed at which the robot moves forward at full power (meters/sec).
_maxRev = 90 #units of revolutions per minute
_MotorIncIntervalSeconds = 0.05 #the amount of time (seconds) with which the incremental motion is to wait before checking total distance
wheelRadius = 3.175 #wheel radius in cm
travelSpeed = (_maxRev*(2 * 3.14 * wheelRadius)/100)/60 #units in meters per second
ctrl_PD = {'P': 1,'D': 1} #PD calibration for smooth motor control

#------------[[NON-EDITABLE VARIABLES, DO NOT EDIT PAST THIS LINE]]-------------

#--------------------[[USER EDITABLE VARIABLES]]--------------------------------

motorFL_EN = 18
motorFR_EN = 12
motorBL_EN = 13
motorBR_EN = 19

motorServo = 9

#"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
frontRGB_r = 4 #front RGB pin RED
frontRGB_g = 17 #front RGB pin GREEN

bumperL = 24 #bumper LEFT
bumperC = 10 #bumper CENTER
bumperR = 25 #bumper RIGHT

#declares I2C communication
muxA0 = 27 
muxA1 = 22 
muxA2 = 23
muxReset = None
muxSDA = 2 #i2c data line
muxSCL = 3 #i2c select line

ultraTrig = 14 #trigger pin
ultraEcho = 15 #echo pin

#sets the pin labling scheme to Broadcom
gpio.setmode(gpio.BCM)

# ---------------------GPIO DECLARATIONS----------------------------

#channel initiation

#FRONT LEFT MOTOR
gpio.setup(18,gpio.OUT) #motorFL_FW_DIGTL
gpio.setup(12,gpio.OUT) #motorFL_BW_DIGTL
gpio.setup(motorFL_EN,gpio.OUT) #motorFL_EN(PWM)
obj_motorFL = gpio.PWM(motorFL_EN,_softwarePWMFrequency) #enable (pin PWM)

#FRONT RIGHT MOTOR
gpio.setup(5,gpio.OUT) #motorFL_FW_DIGTL
gpio.setup(6,gpio.OUT) #motorFL_BW_DIGTL
gpio.setup(motorFR_EN,gpio.OUT) #motorFL_EN(PWM)
obj_motorFL = gpio.PWM(motorFR_EN,_softwarePWMFrequency) #enable (pin PWM)

#BACK LEFT MOTOR
gpio.setup(19,gpio.OUT) #motorFL_FW_DIGTL
gpio.setup(16,gpio.OUT) #motorFL_BW_DIGTL
gpio.setup(motorBL_EN,gpio.OUT) #motorFL_EN(PWM)
obj_motorFL = gpio.PWM(motorBL_EN,_softwarePWMFrequency) #enable (pin PWM)

#BACK RIGHT MOTOR
gpio.setup(26,gpio.OUT) #motorFL_FW_DIGTL
gpio.setup(20,gpio.OUT) #motorFL_BW_DIGTL
gpio.setup(motorBR_EN,gpio.OUT) #motorFL_EN(PWM)
obj_motorFL = gpio.PWM(motorBR_EN,_softwarePWMFrequency) #enable (pin PWM)

#CAMERA SERVO
gpio.setup(motorServo,gpio.OUT)
camServo = gpio.PWM(motorServo,_softwarePWMFrequency) #enable (pin PWM)

#FRONT RED LED
gpio.setup(frontRGB_r,gpio.OUT) #red LED
#FRONT GREEN LED
gpio.setup(frontRGB_g,gpio.OUT) #green LED

#Bumper Switch Left
gpio.setup(bumperL,gpio.OUT) 
#Bumper Switch Center
gpio.setup(bumperC,gpio.OUT)
#Bumper Switch Right
gpio.setup(bumperR,gpio.OUT)

#Color sensors

#Select Lines
gpio.setup(muxA0,gpio.OUT) #A0
gpio.setup(muxA1,gpio.OUT) #A1
gpio.setup(muxA2,gpio.OUT) #A2

#Data and Clock
gpio.set(muxSDA,gpio.OUT) #i2c data line
gpio.set(muxSCL,gpio.OUT) #i2c clock line

#Ultrasonic Sensor
gpio.setup(14,gpio.OUT) #trigger pin
gpio.setup(15,gpio.IN) #echo pin

# -------------------------------BASIC SENSOR POLL FUNCTIONS-----------------------------------

def pollBumpers():

    sensorData["bumperSWL"][0] = 
    sensorData["bumperSWC"][0] = bumperSWC.value
    sensorData["bumperSWR"][0] = bumperSWR.value
    print("Polling Bumpers \n")

def pollUltrasonic(position):
    gpio.output()
# -------------------------------I2C SETUP FUNCTIONS------------------------------------- 

# -------------------------------COMPOUND FUNCTIONS---------------------------------
def percToSpd(speed):
    #speed is taken as percentage and converted into 0 to 1 PWM value
    speed = speed/100
    return speed

def halt():
    #stops all motion
    .stop()
    motorFR.stop()
    motorBL.stop()
    motorBR.stop()
    print("Halting Motors \n")
    
def moveIncremental(direction,distanceMeters,speedPrcnt):
    
    distanceMoved = 0

    if speedPrcnt is None:
        #if no speed provided, default to full power
        speedPrcnt = 100

    match direction:
        case 'f':
            motorFL.forward(speed = percToSpd(speedPrcnt))
            motorFR.forward(speed = percToSpd(speedPrcnt))
            motorBL.forward(speed = percToSpd(speedPrcnt))
            motorBR.forward(speed = percToSpd(speedPrcnt))
        case 'b':
            motorFL.backward(speed = percToSpd(speedPrcnt))
            motorFR.backward(speed = percToSpd(speedPrcnt))
            motorBL.backward(speed = percToSpd(speedPrcnt))
            motorBR.backward(speed = percToSpd(speedPrcnt))
        case 'clk':
            motorFL.forward(speed = percToSpd(speedPrcnt))
            motorFR.backward(speed = percToSpd(speedPrcnt))
            motorBL.forward(speed = percToSpd(speedPrcnt))
            motorBR.backward(speed = percToSpd(speedPrcnt))
        case 'cntr_clk':
            motorFL.backward(speed = percToSpd(speedPrcnt))
            motorFR.forward(speed = percToSpd(speedPrcnt))
            motorBL.backward(speed = percToSpd(speedPrcnt))
            motorBR.forward(speed = percToSpd(speedPrcnt))
        case _:
            print("ERROR INVALID DIRECTION ON INCREMENTAL MOVEMENT FUNCTION")
            distanceMeters = 0

    while(distanceMoved < distanceMeters):
        #d = v * t
        sleep(_MotorIncIntervalSeconds) #delays time in seconds
        distanceMoved += ((travelSpeed * speedPrcnt)) * (_MotorIncIntervalSeconds)

    halt()

#turns left or right incrementally
def followContinuous():
    
    motorFL.forward()
    motorFR.forward()
    motorBL.forward()
    motorBR.forward()

def resetGimbal():
    camServo.value = _servoCalibrationAngle
    print("Camera gimbal has been reset.\n")

def lookBothWays():
    resetGimbal()
    sleep(0.001)
    camServo.value(-_servoMaxTurnAngle)
    sleep(0.001)    
    pollUltrasonic(0) #polls when angle is turned left
    sleep(0.001)
    camServo.value(_servoCalibrationAngle)
    sleep(0.001)
    pollUltrasonic(1) #polls when angle is centered
    sleep(0.001)
    camServo.value(_servoMaxTurnAngle)
    sleep(0.001)
    pollUltrasonic(1) #polls when angle is centered
    print("Finished looking both ways. \n")

def emergencyStop():
    #stops and then disconnects power from motors, program wont operate again until reset
    halt()
    motorFL.close()
    motorFR.close()
    motorBL.close()
    motorBR.close()
    camServo.value = None #will be able to freely move
    print("EMERGENCY STOP HAS BEEN DEPLOYED!\n")
    
    
def moonWalk():
    print("WALKIN' ON THE MOON, BABY!\n")
    #turns wheels inward and accomplishes nothing to test motors
    motorFL.backward()
    motorFR.backward()
    motorBL.forward()
    motorBR.forward()
    for i in range(10):
        camServo.value = -0.5
        sleep(1)
        camServo.value = 0.5
        sleep(1)
    halt()