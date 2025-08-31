#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS 2025

from picamera2 import Picamera2
from gpiozero.pins.pigpio import PiGPIOFactory
from colorama import Fore, Back, Style, init
import cv2 as cv
import numpy as np
import gpiozero as gpio #gpio handling library
import sensorData
import pigpio
import time
import os

#-----------FILE INITIALIZATIONS----------
init()#initializes colorama for colored terminal text
try:
    os.system("sudo pigpiod") #starts pigpio daemon
    time.sleep(2)
    #pigpio initialization
    pi = pigpio.pi()
    gpio.Device.pin_factory = PiGPIOFactory('35.50.3.164')
except:
    print("Error initializing pigpio DAEMON")

#----------------------------------------------------------------USER-DEFINED VARAIBLES---------------------------------------------------------------
#camera feed settings
cameraWidth = 640
cameraHeight = 480
cameraChannelCnt = 3
thresh_value = 200

#-----Screen sector variables
x_scrn, y_scrn = 0, 0
#cuts the image into thirds verticaly
w_tri_1 = round(cameraWidth * (1/3))
w_tri_2 = round(cameraWidth * (2/3)) 
w_half = round(cameraWidth * (1/2))
w_scrn = round(cameraWidth + x_scrn)
#cuts the image into thirds horizontally
h_tri_1 = round(cameraHeight * (1/3))
h_tri_2 = round(cameraHeight * (2/3))
h_half = round(cameraHeight * (1/2))
h_scrn = round(cameraHeight + y_scrn)

#-----HSV color definitions

#"yellowLo" : np.array([0, 27, 255]),
#"yellowHi" : np.array([30, 255, 255]),
#"greenLo" : np.array([38, 28, 173]),
#"greenHi" : np.array([79, 255, 255]),

hsvColors = {
    "yellowLo" : np.array([7, 19, 250]),
    "yellowHi" : np.array([84, 255, 255]),
    "greenLo" : np.array([7, 19, 250]),
    "greenHi" : np.array([84, 255, 255]),
    "blueLo" : np.array([30, 130, 115]),
    "blueHi" : np.array([115, 255, 255])
}

pinAsgn = {
    #---------------------MOTORS---------------------
    "frontLeftForward": (11,1,0),  # controller 1, MotorA, IN1_0
    "frontLeftBackward": (8,1,0),  # controller 1, MotorA, IN2_0
    "frontRightForward": (7,1,0),  # controller 1, MotorB, IN3_0
    "frontRightBackward": (5,1,0),  # controller 1, MotorB, IN4_0

    "frontLeftPWM": (18,1,1),  # controller 1, MotorB, ENA_0
    "frontRightPWM": (12,1,1),  # controller 1, MotorB, ENB_
    "cameraGimbalServo": (9,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
    #---------------------BUMPERS-----------------------
    "leftBumper": (24,1,0), # 
    "centerBumper": (10,1,0), #
    "rightBumper": (25,1,0), #
    #---------------------LEDS-----------------------
    "frontRGB_Red": (17,1,0), # 
    "frontRGB_Green": (4,1,0), #
    #------------------COLOR_SENSOR------------------
    #------------------ULTRASONIC--------------------
    "ultraTrig": (14,1,0), #ultrasonic trigger pin
    "ultraEcho": (15,0,0) #ultrasonic feedback pin
}

#-----PD CONTROL TUNING
stopLineDotThresh = 4 # The number of corners the camera needs to give a postive result
stopLineTimeThresh = 0.2 #time in seconds the state machine will wait before registering a positive on a possible stopline
stopLineThreshNear = 35 #pixel space threshold for stopline detection for ahead stop lines
stopLineThreshFar = 10 #threshold for stopline detection for side stop lines
servoInterDelay = 1.75 #Time between servo flicks

biasMax = 100 
biasMin = -100
pd_Bias = 0
turnOffset = 0.0 #higher value increases turn sharpness
turnBiasFactor = 1 # Magnitude scalar for affecting turns when biased. MIN VALUE 1, MAX VALUE 2
kp = 0.755 #proportional control variable for pd control OLD 0.76
kd = 0.31 #differential control variable for pd control OLD 0.31
skewBias = 0.5 #value to be added or subtracted for when performing PD control on directional device turn
aheadPathConfLines = 5 #number of confident lines that must exist to register a path forward confidently

#-----INCREMENTATL MOTION TUNING3
motorMaxSpeed = 3 #speed at which the robot moves forward at full power (meters/sec).
MotorIncIntervalSeconds = 0.05 #the amount of time (seconds) with which the incremental motion is to wait before checking total distance

#-----SERVO TUNING
ultrasonicThreshDist = 0.35 #float (meters) of the distance when the vehicle should stop behind an obstacle
servoMaxTurnAngle = 0.85 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
servoMinTurnAngle = -1 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
servoCalibrationAngle = 0.15 #angle (degrees) offset for which servo will return upon leaving
servoInterDelay = 0.5 # Time (seconds) for which to delay in between gimbal tilts

#-----STATE MACHINE
interStDelay = 2 #default time (seconds) to delay by default between states

#----------THRESHOLD VALUES----------
blueThreshMin = (86,105,140)
blueThreshMax = (0,0,255)
ultSonicThreshObstacle = 25 #25 milimeters threshold

#------------------------------------------------------------NON-USER-DEFINED VARAIBLES---------------------------------------------------------------

stopLineWatch = None #deboucne for stopLines
servoWatch = None
servoIncrement = 0

#-----PD CONTROL
deltaTime = 1
lastTime = 0
currentTime = 0
E_now = 0
E_last = 0

sensorData = {"ultraSonic":[0,0,0]} #ultrasonic latest readings [Left,Center,Right]
startTime = None
endTime = None

#----------STATE CONDITIONAL VARS----------
currentState = "state_reset"
nextState = None

#----------TRANSITIONAL CONDITIONAL BOOLEANS----------

#-----Camera Exclusive Truths
leftLightDetected = False
yellowLightDetected = False
rightLightDetected = False
isTurnComplete = False

#-----Ultrasonic Exclusive Truths
obstacleOnLeft = False
obstacleOnRight = False
obstacleAhead = False
obstaclePresent = False
lookedBothWays = False

#-----Compound and State Machine Dependent Truths
lineIsVisible = False
confirmBlueAhead = False
foundSharpTurn = False

#---STOPLINE STUFF
foundLeftStopLine = False
foundRightStopLine = False
foundAheadStopLine = False
stopLineDetected = False

gaveUpSearch = False
searchTries = 0

foundPathForward = False
trafficLightDetected = False

turnDirection = None #string type

#----------INTERSTATE CONDITIONS----------
stateIsTransitioning = False
involvedInIntersection = False
isDelayOver = False

#-----------COMPOUND TRANSITIONS----------
trans_returnTurn = False
trans_stopLineDetected = False
trans_returnSafeStop = False
trans_beginDriving = False
trans_goToIdentifyFeatures = False
trans_foundStraightThrough = False
trans_yieldLeftThenForeward = False
trans_intersectionFound = False
trans_foundLeft90Turn = False
trans_foundDirectionalDevice = False
trans_foundYieldLeft = False
trans_lightTypeDetermined = False
trans_yieldLefttoTurnRight = False
trans_turnFinished = False

#---------STATE IDENTITIES
stateNames = {

    # FORMAT:
        # Transition Conidtions [0] <- List of conditions that are searched through to determine if the state should change
        # Requisite Sensor Groups [1] <- List of sensors or parts of camera code that are searched through so the state only has to refresh important data
        # Post-Transition TRUE Resets [2] <- List of variables that need to be reset to TRUE after state transition happens
        # Post-Transition FALSE Resets [3] <- List of variables that need to be reset to FALSE after state transition happens
        # Transition Locations [4] <- List of conditions that are refered BY INDEX when a transition condition is found

    "state_Safety_Stop": (
        ("trans_beginDriving","trans_returnTurn"),
        ("cameraLines","ultrasonic"),
        (None),
        (None),
        ("state_FollowingLine","state_ExecuteTurn")),
    "state_FollowingLine": (
        ("trans_returnSafeStop","trans_stopLineDetected"),
        ("cameraLines","cameraCorners","ultrasonic"),
        (None),
        ("obstacles","lines"),
        ("state_Safety_Stop","state_StopLine")),
    "state_StopLine": (
        ("trans_intersectionFound",),
        ("ultrasonic",),
        (None),
        ("obstacles",),
        ("state_IdentifyIntersection",)),
    "state_IdentifyIntersection": (#ORDER IN THIS STATE MATTERS FOR TRANSITION CHECKS
        ("trans_foundDirectionalDevice","trans_foundYieldLeft","trans_foundLeft90Turn","trans_foundStraightThrough"),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        (None),
        ("obstacles","lines"),
        ("state_DetermineLight","state_YieldtoLeft","state_ExecuteTurn","state_FollowingLine")),
    "state_YieldtoLeft": (
        ("trans_yieldLeftThenForeward","trans_yieldLefttoTurnRight"),
        ("cameraLines","cameraCorners","ultrasonic"),
        (None),
        ("obstacles","lines"),
        ("state_FollowingLine","state_ExecuteTurn")),
    "state_DetermineLight": (
        ("trans_lightTypeDetermined",),
        ("cameraColors","ultrasonic"),
        (None),
        ("obstacles","lines"),
        ("state_ExecuteTurn",)),
    "state_ExecuteTurn": (
        ("trans_turnFinished","trans_returnSafeStop"),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        (None),
        ("obstacles","lines"),
        ("state_FollowingLine","state_Safety_Stop"))
}

#------------------------------SENSOR DECLARATIONS---------------------------------------------------

#-----VIDEO CAPTURE
camera = Picamera2(camera_num=0)
camera.configure()
camera.controls.AnalogueGain = 1
camera.resolution = (cameraWidth,cameraHeight)
camera.start()

#FRONT LEFT
motorFL = gpio.Motor(
    forward= pinAsgn["frontLeftForward"][0], #forward
    backward= pinAsgn["frontLeftBackward"][0], #backward
    enable= pinAsgn["frontLeftPWM"][0], #enable pin
    pwm=True
)
#FRONT RIGHT
motorFR = gpio.Motor(
    forward= pinAsgn["frontRightForward"][0], #forward
    backward= pinAsgn["frontRightBackward"][0], #backward
    enable= pinAsgn["frontRightPWM"][0], #enable pin
    pwm=True
)
#CAMERA SERVO
camServo = gpio.AngularServo(
    initial_angle= servoCalibrationAngle,
    min_angle= servoMinTurnAngle,
    max_angle= servoMaxTurnAngle,
    pin= pinAsgn["cameraGimbalServo"][0]
)

#-----FRONT RED LED
fntRed = gpio.DigitalOutputDevice(pin= pinAsgn["frontRGB_Red"][0], initial_value= False)
#-----FRONT GREEN LED
fntGreen = gpio.DigitalOutputDevice(pin= pinAsgn["frontRGB_Green"][0], initial_value= True)

#-----BUMPERS
#Callback function
def bumperCallBack():
    global stateIsTransitioning
    #stops and then disconnects power from motors, program wont operate again until reset
    halt()
    motorFL.close()
    motorFR.close()
    camServo.value = 0 #will be able to freely move
    fntRed.on()
    fntGreen.off()
    stateIsTransitioning = True
    print("EMERGENCY STOP HAS BEEN DEPLOYED!\n")
    while(1):
        pass

#Left Bumper Switch
bumperSWL = gpio.Button(pin= pinAsgn["leftBumper"][0], pull_up = False)
bumperSWL.when_pressed = bumperCallBack
#Center Bumper Switch
bumperSWC = gpio.Button(pin= pinAsgn["centerBumper"][0], pull_up = False)
bumperSWC.when_pressed = bumperCallBack
#Right Bumper Switch
bumperSWR = gpio.Button(pin= pinAsgn["rightBumper"][0], pull_up = False)
bumperSWR.when_pressed = bumperCallBack

#-----ULTRASONIC SENSOR
ultSon = gpio.DistanceSensor(
    pinAsgn["ultraEcho"][0],
    pinAsgn["ultraTrig"][0]
)

#----------FILE-VISIBLE WRITE FUNCTIONS----------

def writeSonicSensorData(sensorString: str, data):
    match sensorString:
        case "left":
            sensorData["ultraSonic"][0] = round(data * 100)
        case "center":
            sensorData["ultraSonic"][1] = round(data * 100)
        case "right":
            sensorData["ultraSonic"][2] = round(data * 100)

#----------INTERPRET FUNCTIONS----------

def interpretSonicSensor():

    global obstacleOnLeft,obstacleAhead,obstacleOnRight,obstaclePresent

    if (sensorData["ultraSonic"][0] <= ultSonicThreshObstacle):
        obstacleOnLeft =  True
    else:
        obstacleOnLeft =  False
    if (sensorData["ultraSonic"][1] <= ultSonicThreshObstacle):
        obstacleAhead =  True
    else:
        obstacleAhead =  False
    if (sensorData["ultraSonic"][2] <= ultSonicThreshObstacle):
        obstacleOnRight =  True
    else:
        obstacleOnRight =  False

    if (currentState == "state_Safety_Stop" or  currentState == "state_FollowingLine" or currentState == "state_StopLine"):
        obstaclePresent = obstacleAhead
    elif(currentState == "state_IdentifyIntersection"):
        obstaclePresent = obstacleOnLeft
    else:  
        obstaclePresent = obstacleAhead

def interpretCamera_Colors():
    global turnDirection, trafficLightDetected

    if (leftLightDetected):
        turnDirection = "left"
        trafficLightDetected = True
    elif (rightLightDetected):
        turnDirection = "right"
        trafficLightDetected = True
    else:
        turnDirection = "none"
        trafficLightDetected = False

def interpretIntersection():
    global foundPathForward, gaveUpSearch, searchTries
    global foundLeftStopLine, foundRightStopLine
    print(startTime)
    print(gaveUpSearch)
    if(isDelayOver and (startTime is None)):#inner loop wont run until after new delay has been delcared and won't consider anything after delay is over
        gaveUpSearch = True
        searchTries = 0
    else:
        gaveUpSearch = False
        searchTries += 1

    #foundPathForward = ((not foundLeftStopLine) ^ (not foundRightStopLine))
    foundPathForward = foundLeftStopLine ^ foundRightStopLine

#----------SECONDARY CONTROL-FLOW FUNCTIONS----------
def initalize():
    global currentState
    currentState = "state_Safety_Stop"
    updateCompTrans()
    fntRed.off()
    fntGreen.on()
    resetGimbal()

def shutdown():
    halt()
    closeCvWindows()
    resetGimbal()
    global pi
    try:
        pi.stop()
        time.sleep(1)
        os.system("sudo killall pigpiod") #stops pigpio daemon
    except:
        print("Error STOPPING pigpio DAEMON")

def createDelay(customTime=interStDelay):
    global endTime,isDelayOver,startTime
    isDelayOver = False

    startTime = time.monotonic()
    if customTime is not None:
        endTime = time.monotonic() + customTime
    else:
        endTime = time.monotonic() + interStDelay

def checkDelay():
    global isDelayOver,endTime,startTime
    currentTime = time.monotonic()

    if (currentTime >= endTime):
        print ("State machine delay is now over!")
        isDelayOver = True
        startTime = None
        endTime = 0
    else:
        isDelayOver = False
        
def resetStateVars():
    #reset all variables in current state list to respective TRUE or FALSE

    if stateNames[currentState][2] is not None:
        for item in stateNames[currentState][2]:
                resetTruthData(item, True)
    if stateNames[currentState][3] is not None:
        for item in stateNames[currentState][3]:
                resetTruthData(item, False)

def updateCompTrans():
    global trans_leaveReset,trans_returnTurn,trans_stopLineDetected,trans_returnSafeStop,trans_returnSafeStop,trans_beginDriving,trans_goToIdentifyFeatures
    global trans_foundStraightThrough, trans_yieldLeftThenForeward, trans_intersectionFound, trans_yieldLeftThenForeward, trans_foundLeft90Turn, trans_foundDirectionalDevice
    global trans_foundYieldLeft, trans_lightTypeDetermined, trans_yieldLefttoTurnRight, trans_turnFinished

    #----------COMPOUND TRANSITION COMBINATIONS
    trans_leaveReset = True #CONDITIONLESS, MEANT TO GO IMMEDIATELY
    trans_returnTurn = (involvedInIntersection) and (not obstaclePresent) and (lineIsVisible) #path is taken incase turn is interrupted by obstacle
    trans_stopLineDetected = stopLineDetected #2
    trans_returnSafeStop = (obstaclePresent) or (not lineIsVisible)
    trans_beginDriving = (not obstaclePresent) and (lineIsVisible)
    trans_goToIdentifyFeatures = not obstaclePresent
    trans_intersectionFound = (not obstaclePresent) and (isDelayOver)

    #============PROBLEM VARIABLES============
    #trans_yieldLeftThenForeward = False
    #trans_foundDirectionalDevice = False
    #trans_foundStraightThrough = False
    #trans_foundLeft90Turn = False
    #trans_foundYieldLeft = False

    trans_foundDirectionalDevice = (trafficLightDetected) and (isDelayOver)
    trans_yieldLeftThenForeward = (obstaclePresent) and (foundPathForward) and (isDelayOver)  
    trans_foundLeft90Turn = ((not foundPathForward) and (not foundLeftStopLine) and (foundRightStopLine) and (isDelayOver))
    trans_foundYieldLeft = ((obstaclePresent) and (foundLeftStopLine) and (not foundRightStopLine) and (isDelayOver))
    trans_foundStraightThrough = (not obstaclePresent) and (foundPathForward) and (isDelayOver)

    #==================POST DETERMINE VARS===================

    trans_lightTypeDetermined = (leftLightDetected or rightLightDetected) and (isDelayOver)
    trans_yieldLefttoTurnRight = (not obstaclePresent) and (not foundPathForward)
    trans_turnFinished = (isTurnComplete) and (isDelayOver)

def checkTransitionTruth(transitionName: str):
    
    retVal = False

    match transitionName:
        case "trans_leaveReset":
            if (trans_leaveReset): retVal = True
        case "trans_returnTurn":
            if (trans_returnTurn): retVal = True
        case "trans_stopLineDetected":
            if (trans_stopLineDetected): retVal = True
        case "trans_returnSafeStop":
            if (trans_returnSafeStop): retVal = True
        case "trans_beginDriving":
            if (trans_beginDriving): retVal = True
        case "trans_goToIdentifyFeatures":
            if (trans_goToIdentifyFeatures): retVal = True
        case "trans_foundStraightThrough":
            if (trans_foundStraightThrough): retVal = True
        case "trans_yieldLeftThenForeward":
            if (trans_yieldLeftThenForeward): retVal = True
        case "trans_intersectionFound":
            if (trans_intersectionFound): retVal = True
        case "trans_foundLeft90Turn":
            if (trans_foundLeft90Turn): retVal = True
        case "trans_foundDirectionalDevice":
            if (trans_foundDirectionalDevice): retVal = True
        case "trans_foundYieldLeft":
            if (trans_foundYieldLeft): retVal = True
        case "trans_lightTypeDetermined":
            if (trans_lightTypeDetermined): retVal = True
        case "trans_yieldLefttoTurnRight":
            if (trans_yieldLefttoTurnRight): retVal = True
        case "trans_turnFinished":
            if (trans_turnFinished): retVal = True

    return retVal

def fetchSensorData():
    
    for stringVal in stateNames[currentState][1]:
        print("Checking data from ", {stringVal}, "\n")

        match stringVal:
            case "ultrasonic":
                pollUltrasonic("center")
                interpretSonicSensor()
            case "cameraLines":
                findLinePath()
            case "cameraCorners":
                detectStopLines()
                interpretIntersection()
            case "cameraColors":
                detectLEDS()
                interpretCamera_Colors()
            case None:
                pass
            case _:
                pass
                print("INVALID SENSOR WAS ACCESSED\n")
                #raise Exception("INVALID SENSOR WAS ACCESSED")
            
    updateCompTrans()

def printDebugInfo():
    os.system('clear')
    print("========== DEBUG INFO ========= \n")
    print("Current State:", {currentState})
    print("Next State: ", {nextState}, "/n")
    print("State is transitioning: ", {stateIsTransitioning})
    print("STOP LINES", {foundLeftStopLine}, "(<)", {stopLineDetected},"(^)", {foundRightStopLine},"(>)")
    print("STOP Inference",{foundAheadStopLine})
    print("Obstacle is present: ", {obstaclePresent})
    print("Line is visible: ", {lineIsVisible}, "\n")
    print("Intersection Features: ", "Traffic Light: ", {trafficLightDetected})
    print("Ultrasonic Data: ",{sensorData["ultraSonic"][0]},{sensorData["ultraSonic"][1]},{sensorData["ultraSonic"][2]},"\n")
    print("=============================== \n")

def checkForChangeStates():
    global stateIsTransitioning, currentState, nextState, involvedInIntersection

    if (currentState == "state_FollowingLine"):
        involvedInIntersection = False
    else:
        involvedInIntersection = True

    printDebugInfo()

    for i, transName in enumerate(stateNames[currentState][0]):
        if (checkTransitionTruth(transName)):
            print("////////////////////////STATE TRANSITION/////////////////////// \n")
            nextState = stateNames[currentState][4][i]
    if (nextState is not None):
        resetStateVars() #resets the required values for the state that was last exited
        currentState = nextState
        nextState = None #resets back to unchanged
        stateIsTransitioning = True        

def resetTruthData(sensorString: str, PARITY: bool):

    #IF THERE IS EVER A PROBLEM WITH DATA WRITING OR ANY OTHER DETECTION, CHECK IT HERE

    global obstacleOnLeft,obstacleAhead,obstacleOnRight,lookedBothWays
    global lineIsVisible,stopLineDetected,obstaclePresent,lightDirectionDetermined
    global rightLightDetected,leftLightDetected,yellowLightDetected
    global foundLeftStopLine,foundRightStopLine,foundAheadStopLine,turnDirection,isTurnComplete,gaveUpSearch

    match sensorString:
        case "obstacles":
            obstacleOnLeft = PARITY
            obstacleOnRight = PARITY
            obstacleOnRight = PARITY
            obstaclePresent = PARITY
            lookedBothWays = PARITY
        case "lines":
            lineIsVisible = PARITY
        case "intersection":
            lightDirectionDetermined = PARITY
            turnDirection = None
            gaveUpSearch = False
        case "colors":
            rightLightDetected = PARITY
            leftLightDetected = PARITY
            yellowLightDetected = PARITY
        case "stopLines":
            lineIsVisible = PARITY
            stopLineDetected = PARITY
            foundLeftStopLine = PARITY
            foundRightStopLine = PARITY
            foundAheadStopLine = PARITY
            isTurnComplete = PARITY
        case _:
            pass
            print("There was an invalid data reset, it was ", {sensorString})

def resetSensorData():
    global sensorData
    sensorData["ultraSonic"][0] = 0
    sensorData["ultraSonic"][1] = 0
    sensorData["ultraSonic"][2] = 0

    sensorData["colorSensor"][0] = 0
    sensorData["colorSensor"][1] = 0
    sensorData["colorSensor"][2] = 0

#----------SENSOR FUNCTIONS----------

def pollUltrasonic(position= None):

    if position is None:
        position = "center"

    match position:
        case "left":
            writeSonicSensorData("left", ultSon.value)
        case "center":
            writeSonicSensorData("center", ultSon.value)
        case "right":
            writeSonicSensorData("right", ultSon.value)

#----------PRIMARY IMAGE PROCESSING FUNCTIONS----------

def findLinePath():

    global pd_Bias, frameRAW, lineIsVisible, confirmBlueAhead

    grabNewFrame() 

    detectedEdges = isolateLines()

    lines = cv.HoughLinesP(detectedEdges,1,np.pi/180, 50, minLineLength= 10, maxLineGap=20)

    pd_Bias = 0
    pathAheadCount = 1

    length_1st = length_2nd = length_3rd = 0
    vector_1st = vector_2nd = vector_3rd = (w_half,h_scrn,w_half,h_tri_2)
    vector_result = None

    if lines is not None:
        lineIsVisible = True

        for line in lines:

            if (checkLineWithinROI(line[0])):

                x1,y1,x2,y2 = line[0] #extracts line coordinate points
                length = ((x2-x1) **2 + (y2 -y1) **2) ** 0.5 #calculates magnitude 

                if (length > length_1st):
                    length_3rd = length_2nd
                    length_2nd = length_1st
                    length_1st = length

                    vector_3rd = vector_2nd
                    vector_2nd = vector_1st
                    vector_1st = line[0]

                elif (length > length_2nd):
                    length_3rd = length_2nd
                    length_2nd = length

                    vector_3rd = vector_2nd
                    vector_2nd = line[0]

                elif (length > length_3rd):
                    length_3rd = length

                    vector_3rd = line[0]

        vector_result = (
            (int((vector_1st[0] + vector_2nd[0] + vector_3rd[0])/3)),
            (int((vector_1st[1] + vector_2nd[1] + vector_3rd[1])/3)),
            (int((vector_1st[2] + vector_2nd[2] + vector_3rd[2])/3)),
            (int((vector_1st[3] + vector_2nd[3] + vector_3rd[3])/3))
        )
    else:
        vector_result = None
        lineIsVisible = False
    
    if ((vector_result is not None) and len(vector_result) == 4):

        x1,y1,x2,y2 = vector_result
        vector_midpoint = ((x2 + x1) //2 , (y2 + y1) //2)
        center = (w_half,h_scrn)

        cv.line(frameRAW, (x1,y1), (x2, y2), (0, 255, 0), 2) #prints the largest line
        cv.arrowedLine(frameRAW, center,vector_midpoint, (0, 255, 255), 4) #prints the direction vector

        if(vector_midpoint[0] <= w_half):
            print("Left Side")
        elif(vector_midpoint[0] > w_half):
            print("Right Side")
            
    else:
        vector_midpoint = (w_scrn,h_tri_2)
        print("Line is not visible")
    
    pd_Bias = (vector_midpoint[0] - w_half)/(w_scrn/2)

    print(pd_Bias)

    #debugFrameLineFollow = drawROI(frameRAW)
    #cv.imshow("lineFollow", debugFrameLineFollow)
    #key = cv.waitKey(1)
    #if key == 27:
    #    pass

def detectStopLines():
    global foundAheadStopLine, frameFilterCorners, foundLeftStopLine, foundRightStopLine
    global corners, dotsInfront, dotsOnLeft, dotsOnRight

    grabNewFrame()
    #global frameFilterCorners

    detectedEdges = isolateLines()
    dotsInfront = 0
    dotsOnLeft = 0
    dotsOnRight = 0
    fixThresh = None

    if (currentState == "state_FollowingLine"):
        fixThresh = stopLineThreshNear
    else:
        fixThresh = stopLineThreshFar

    corners = cv.goodFeaturesToTrack(detectedEdges, maxCorners = 40, qualityLevel = 0.02, minDistance=fixThresh ,useHarrisDetector=True, k=0.1)
    if corners is not None:
        corners = np.int0(corners)
        frameFilterCorners = checkWithinROI(corners,frameRAW)
    else:
        frameFilterCorners = detectedEdges

    foundLeftStopLine = False
    foundRightStopLine = False
    foundAheadStopLine = False

    #determines location of dots
    if (dotsOnLeft >= stopLineDotThresh):
        print(Fore.CYAN + "[<=] Stopline LEFT was detected." + Style.RESET_ALL)
        foundLeftStopLine = True
    if (dotsOnRight >= stopLineDotThresh):
        print(Fore.CYAN + "[=>] Stopline RIGHT was detected." + Style.RESET_ALL)
        foundRightStopLine = True
    if (dotsInfront >= stopLineDotThresh):
        print(Fore.GREEN + "[^] Stopline AHEAD was detected." + Style.RESET_ALL)
        foundAheadStopLine = True

    debounceStopLineAhead()

    print("\r")
    print("\r")

    debugFrameCorners = drawROI(frameFilterCorners)
    #cv.imshow("stopLine", debugFrameCorners)

    #key = cv.waitKey(1)
    #if key == 27:
    #    pass

def detectLEDS():
    global leftLightDetected, rightLightDetected, rightLightDetected, isTurnComplete

    grabNewFrame()

    leftLightDetected = False
    rightLightDetected = False

    frameResized = cv.resize(frameRAW,(cameraWidth,cameraHeight))
    baseFrameGray = cv.cvtColor(frameResized,cv.COLOR_BGR2GRAY) #GRAY

    maskGreenHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)
    maskYellowHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)

    maskGreenHSV = createHSVMasks(maskGreenHSV,"green") #Green HSV mask
    maskYellowHSV = createHSVMasks(maskYellowHSV,"yellow") #Yellow HSV mask

    #creates a blurry frame with which to apply to the circles
    maskGreenHSV = cv.medianBlur(maskGreenHSV, 5) # HSV only takes 3 and 5 as kernel size when using uint8
    maskYellowHSV = cv.medianBlur(maskYellowHSV, 5) # HSV only takes 3 and 5 as kernel size when using uint8

    baseFrameGray = cv.medianBlur(baseFrameGray, 3) # HSV only takes 3 and 5 as kernel size when using uint8
    ret, maskBright = cv.threshold(baseFrameGray,210,255,cv.THRESH_BINARY) #filters all brightest pixels from the screen given a certain threshold

    for i in (maskGreenHSV,maskYellowHSV):
        cntFrame, _ = cv.findContours(i, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for cnt in cntFrame:
            (x,y), radius = cv.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv.circle(i, center, radius, (255, 255, 255), cv.FILLED)

    greenLEDPixels = apply_AND_Mask(maskGreenHSV,maskBright)
    yellowLEDPixels = apply_AND_Mask(maskYellowHSV,maskBright)

    #calculates the mean x position of the x pixels between the two photos
    totalArray = np.add(greenLEDPixels,yellowLEDPixels)
    major = cv.moments(totalArray)

    cX = 0
    if major["m00"] != 0:  # No division by zero
        cX = int(major["m10"] / major["m00"])
        
    if(w_scrn*(1/4) < cX < w_half):
        leftLightDetected = True
        print("Left light detected", {cX})
    elif(w_scrn*(3/4) > cX > w_half):
        print("Right light detected", {cX})
        rightLightDetected = True
    else:
        print("There were no traffic lights detected in allocated range", {cX})
        leftLightDetected = False
        rightLightDetected = False

    #turn completion check
    if(currentState == "state_ExecuteTurn"):
        if(cX <= (w_scrn * (1/4)) or cX >= (w_scrn * (3/4))):
            #if the positional average is outside the middle 2 quarters of the screen, turn manuever is complete
            isTurnComplete = True
        else:
            isTurnComplete = False

    #key = cv.waitKey(1)
    #if key == 27:
    #    pass
    
    #cv.imshow("greenLEDPixels", maskGreenHSV)
    #cv.imshow("yellowLEDPixels", maskYellowHSV)

#----------SECONDARY IMAGE PROCESSING FUNCTIONS----------

def createHSVMasks(baseFrame,frameColorName):
     loCase = frameColorName.lower()
     newImage = cv.inRange(baseFrame, hsvColors[loCase + "Lo"], hsvColors[loCase + "Hi"])
     return newImage

def apply_AND_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def checkWithinROI(cornerImage,frameRAW):
        
    global dotsOnLeft,dotsOnRight,dotsInfront
    dotsOnLeft = 0
    dotsOnRight = 0
    dotsInfront = 0

    #stop line detector
    for corner in cornerImage:
        x, y = corner.ravel()
        frameFilterCorners = cv.circle(frameRAW, center=(x,y), radius = 8, color=(0,0,255), thickness=-1)

        if ((x >= (x_scrn)) and (x <= (x_scrn+w_tri_1)) and (y >= (y_scrn+h_tri_1)) and (y <= (y_scrn+h_tri_2))): #checks for left third of the screen
                dotsOnLeft += 1
        
        if ((x >= (x_scrn + w_tri_2)) and (x <= (w_scrn)) and (y >= (y_scrn+h_tri_1)) and (y <= (y_scrn+h_tri_2))): #checks for right third of the screen
                dotsOnRight += 1
#(x >= (x_scrn + w_tri_1)) and (x <= (x_scrn + w_tri_2)) and (y >= (y_scrn + h_tri_2)) and (y <= (h_scrn)
        if ((x >= (w_scrn/4)) and (x <= (w_scrn*3/4))) and (y >= (h_tri_2)) and (y <= (h_scrn)): #checks for bottom third of the screen
                dotsInfront += 1

    return frameFilterCorners

def checkLineWithinROI(testpoint):
    #line within bottom third of screen
    x1,y1,x2,y2 = testpoint

    if ((x1 >= (0)) and (x1 <= (w_scrn)) and (y1 >= (y_scrn + h_tri_2)) and (y2 <= (h_scrn))):
        if ((x2 >= (0)) and (x2 <= (w_scrn)) and (y2 >= (y_scrn + h_tri_2)) and (y2 <= (h_scrn))):
            return True
    else:
        return False

def drawROI(baseFrame):
    baseFrame = cv.rectangle(baseFrame,(x_scrn,y_scrn),(x_scrn + w_tri_1,y_scrn + h_scrn),(0,255,0),thickness = 4) #left vertical tri
    baseFrame = cv.rectangle(baseFrame,(x_scrn + w_tri_2,y_scrn),(x_scrn + cameraWidth,y_scrn + h_scrn),(0,255,0),thickness = 4) #right vertical tri
    baseFrame = cv.rectangle(baseFrame,(x_scrn,y_scrn+h_tri_1),(x_scrn + cameraWidth,y_scrn + h_tri_2),(0,0,255),thickness = 4) #middle horizontal tri
    return baseFrame

def isolateLines():
    global frameHSV, frameBlue, blurFrame, detectedEdges

    frameHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV)
    frameBlue = createHSVMasks(frameHSV,"blue")
    blurFrame = cv.GaussianBlur(frameBlue,(51,51),3)
    detectedEdges = cv.Canny(blurFrame,50,180)
    detectedEdges = cv.GaussianBlur(detectedEdges,(5,5),3)

    #cv.imshow("lineEdges", detectedEdges)
    return detectedEdges

def grabNewFrame():
    global frameRAW
    frameRAW = cv.cvtColor(camera.capture_array(), cv.COLOR_RGB2BGR)

def debounceStopLineAhead():
    global stopLineDetected, foundAheadStopLine, stopLineWatch
    #function is responsible for creating a delay to determine if a stopline has actually been present

    if (foundAheadStopLine and (stopLineWatch is None)):
        stopLineWatch = time.monotonic()
    elif (foundAheadStopLine and (stopLineWatch is not None)):
        if((time.monotonic() - stopLineWatch) >= stopLineTimeThresh):
            stopLineDetected = True
    else:   
        stopLineDetected = False
        stopLineWatch = None

def closeCvWindows():
    cv.destroyAllWindows()

#----------MOTOR CONTROL FUNCTIONS----------
def halt():
    #stops all motion
    motorFL.stop()
    motorFR.stop()
    print("Halting Motors \n")
    
def updateMotion(mode,speedPrcnt,skewBias = None):

    global pd_Bias,leftBias,rightBias,turnOffset,turnBiasFactor,distanceMeters
    
    #bias works on a scale of 0 to 100
    # 0 is directing all forward motion to the left motors
    # 100 is directing all forward motion to the right motorsspeedPrcnt
    # 50 is direction all forward motion evenly to both sets

    if speedPrcnt is None:
        #if no speed provided, default to full powerresetCaresetCa
        speedPrcnt = 100

    if skewBias is None:
        #if not skew provided, a normal line following ensues, this value is for sharp turns
        skewBias = 0
        
    match mode:
        case 'pd': #PID controlled  
            pd_Bias = pd_Bias + skewBias
            motorBias = findMotorBias(pd_Bias)

            if (motorBias < 0):
                rightBias = 1
                leftBias = 1 + (motorBias - turnOffset)/(100 * turnBiasFactor)
                leftBias = max(0, min(leftBias, 1))
            elif(motorBias >= 0):
                rightBias = 1 - (motorBias + turnOffset)/(100 * turnBiasFactor)
                rightBias = max(0, min(rightBias, 1))
                leftBias = 1

            motorFL.forward(speed = leftBias*(speedPrcnt/100))
            motorFR.forward(speed = rightBias*(speedPrcnt/100))
            print(f"{leftBias},{rightBias},{motorBias},{pd_Bias},{skewBias}")

        case 'cw': #clockwise
            motorFL.forward(speed = (speedPrcnt))
            motorFR.backward(speed = (speedPrcnt))
        case 'ccw': #anticlockwise
            motorFL.backward(speed = (speedPrcnt))
            motorFR.forward(speed = (speedPrcnt))
        case _:
            print("ERROR INVALID DIRECTION ON INCREMENTAL MOVEMENT FUNCTION")
            distanceMeters = 0

def moveForDist(distanceMeters):
    # possibly look for monotonic approach if doesn't work?
    distanceMoved = 0
    while(distanceMoved < distanceMeters):
        #d = v * t
        distanceMoved += ((motorMaxSpeed * time.speedPrcnt)) * (MotorIncIntervalSeconds)
        time.sleep(MotorIncIntervalSeconds) #delays time in seconds

def resetGimbal():
    camServo.value = servoCalibrationAngle
    #print("Camera gimbal has been reset.\n")

def lookBothWays():
    global servoWatch,servoInterDelay,servoIncrement,lookedBothWays
    #function is responsible for creating a delay to determine if a stopline has actually been present

    if ((servoWatch is None) and (lookedBothWays == False)):
        servoWatch = time.monotonic() + 1   
    elif((servoWatch is not None) and (lookedBothWays == False)):
        if(time.monotonic() >= servoWatch):
            servoIncrement += 1
            servoWatch = None

        match servoIncrement:
            case 0:
                camServo.value = servoMinTurnAngle
                pollUltrasonic("right") #polls when angle is turned left 
            case 1:
                camServo.value = servoCalibrationAngle
                pollUltrasonic("center") #polls when angle is centered
            case 2:
                camServo.value = servoMaxTurnAngle
                pollUltrasonic("left") #polls when angle is centered
            case 3:
                camServo.value = servoCalibrationAngle
                servoIncrement = 0
                servoWatch = None
                lookedBothWays = True
    
#----------INTERPRETATION FUNCTIONS----------
def findMotorBias(pd_Val):

    global E_now,E_last,E_Dt,deltaTime, lastTime, currentTime

    #find delta time
    lastTime = currentTime
    currentTime = time.monotonic()
    deltaTime = currentTime - lastTime

    #setting last value times
    E_last = E_now
    #setting current value times
    E_now = 100.0 * pd_Val
    
    #creating differential term
    E_Dt = (E_now - E_last)/deltaTime

    #bias calculation
    motorBias = kp*(E_now) + kd*(E_Dt)

    return max(biasMin, min(motorBias, biasMax))

#----------STATE DECLARATIONS----------

def state_Safety_Stop():
    createDelay(2)
    halt()

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_FollowingLine():
    createDelay(1)

    while(not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()
        updateMotion("pd",speedPrcnt=35,skewBias=None)
        print("Inside main loop of following. \n")

def state_StopLine():
    createDelay(2)
    halt()

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_IdentifyIntersection():

    print("Attempting to identify intersection...\n")
    while(lookedBothWays == False):
        lookBothWays()
    interpretIntersection()
    createDelay(3)
    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

    resetGimbal()

def state_YieldtoLeft():
    createDelay(2)
    print("Currently yielding to the left...")

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_DetermineLight():
    createDelay(2)
    print("Attempting to determine traffic light...")

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_ExecuteTurn():
    halt()
    createDelay(1) 
    print("[[[EXECUTING TURN WARNING!!!!]]]")
    while (not stateIsTransitioning):
        if(turnDirection == "left"):
            updateMotion("pd",speedPrcnt=40,skewBias=-skewBias)
        elif(turnDirection == "right"):
            updateMotion("pd",speedPrcnt=40,skewBias=skewBias)
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

#----------PRIMARY CONTROL-FLOW FUNCTIONS----------

def ctrlLoop():
    global stateIsTransitioning, currentState

    initalize()
    resetStateVars() #reset all functions

    while(1): #Primary loop
        match currentState:
            case "state_Safety_Stop":
                state_Safety_Stop()
            case "state_FollowingLine":
                state_FollowingLine()
            case "state_StopLine":
                state_StopLine()
            case "state_IdentifyIntersection":
                state_IdentifyIntersection()
            case "state_YieldtoLeft":
                state_YieldtoLeft()
            case "state_DetermineLight":
                state_DetermineLight()
            case "state_ExecuteTurn":
                state_ExecuteTurn()
            case _:
                raise Exception("INVALID STATE ID HAS BEEN ACCESSED")
            
        stateIsTransitioning = False
        print ("State has now been changed to {}".format(currentState))
                
    print ("FSM HAS LEFT THE CONTROL LOOP")
    shutdown()

#----------PROGRAM INITIALIZATION----------

#ctrlLoop() #run FSM and rest of program

#=============PLAYGROUND CODE===========
# DONT FORGET EACH STATE WILL RUN IN ITS OWN LOOP INFINTLY, YOU DONT NEED A LOOP HERE
#initalize()
# time.sleep(2)
# currentState = "state_FollowingLine"
# something = False
# LoopActive = True

try:
    ctrlLoop()
    #while(1):
    #    grabNewFrame()
    #    detectStopLines()
except RuntimeError:
    shutdown()
