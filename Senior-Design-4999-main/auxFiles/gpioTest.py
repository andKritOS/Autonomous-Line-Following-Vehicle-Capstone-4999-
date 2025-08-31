import gpiozero as gpio
import board
import adafruit_tcs34725
from time import sleep

        self.I2C_data = None #information recieved from the I2C 

        i2c = board.I2C()

        # ALL WILL BE SET TO OUTPUTS
        # ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
        # FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))

        self.pinAsgn = {
            #---------------------MOTORS---------------------
            "frontLeftForward": (8,1,0),  # controller 1, MotorA, IN1_0
            "frontLeftBackward": (7,1,0),  # controller 1, MotorA, IN2_0
            "frontRightForward": (5,1,0),  # controller 1, MotorB, IN3_0
            "frontRightBackward": (6,1,0),  # controller 1, MotorB, IN4_0
            "frontLeftPWM": (11,1,1),  # controller 1, MotorB, ENA_0
            "frontRightPWM": (12,1,1),  # controller 1, MotorB, ENB_0
            "backLeftForward": (19,1,0),  # controller 2, MotorA, IN1_1
            "backLeftBackward": (16,1,0),  # controller 2, MotorA, IN2_1
            "backRightForward": (26,1,0),  # controller 2, MotorB, IN3_1
            "backRightBackward": (20,1,0),  # controller 2, MotorB, IN4_1
            "backLeftPWM": (13,1,1),  # controller 2, MotorB, ENA_1
            "backRightPWM": (21,1,1),  # controller 2, MotorB, ENB_1
            "cameraGimbalServo": (9,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
            #---------------------LEDS-----------------------
            "frontRGB_Red": (4,1,0), # 
            "frontRGB_Green": (17,1,0), #
            #"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
            #---------------------BUTTONS--------------------
            "frontBumper": (25,1,0),
            #------------------COLOR_SENSOR------------------
            #declares I2C communication
            "Select_A0": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
            "Select_A1": (23,0,0), 
            "Select_A2": (27,1,0),
            "I2CReset": (10,0,0),
            "I2C_SDA": (22,1,0),
            "I2C_SCL": (24,0,0),
            #------------------ULTRASONIC--------------------
            "ultraTrig": (2,1,0), #ultrasonic trigger pin
            "ultraEcho": (3,0,0) #ultrasonic feedback pin
        }

        #----------------------------[[NON-EDITABLE VARIABLES, DO NOT EDIT PAST THIS LINE]]--------------------------------------

        self.__efcServoMaxAngle = ((self._servoMaxTurnAngle + self._servoCalibrationAngle)/360 )
        self.__efcServoMinAngle = -self.__servoMaxAngle #operational fraction for gimbal turning left and right
        self.__

        # pin assignments
        #FRONT LEFT
        self.motorFL = gpio.Motor(
            self.pinAsgn["frontLeftForward"[0]], #forward
            self.pinAsgn["frontLeftBackward"[0]], #backward
            self.pinAsgn["frontLeftPWM"[0]], #backward
            True,
            None
            )

        #FRONT RIGHT
        self.motorFR = gpio.Motor(
            self.pinAsgn["frontRightForward"[0]], #forward
            self.pinAsgn["frontRightBackward"[0]], #backward
            self.pinAsgn["frontRightPWM"[0]], #backward
            True,
            None
            )

        #BACK LEFT
        self.motorBL = gpio.Motor(
            self.pinAsgn["backLeftForward"[0]], #forward
            self.pinAsgn["backLeftBackward"[0]], #backward
            self.pinAsgn["backLeftPWM"[0]], #backward
            True,
            None
            )

        #BACK RIGHT
        self.motorBR = gpio.Motor(
            self.pinAsgn["backRightForward"[0]], #forward
            self.pinAsgn["backRightBackward"[0]], #backward
            self.pinAsgn["backRightPWM"[0]], #backward
            True,
            None
            )

        #CAMERA SERVO
        self.camServo = gpio.servo(
            self.pinAsgn["cameraGimbalServo"[0]],
            self._servoCalibrationAngle, #initial value on startup
            0.01, #min pw
            1, #max pw
            0.020, #frame width
            None
        )

        #FRONT RED LED
        self.fntRed = gpio.LED(self.pinAsgn["frontRGB_Red"[0]])

        #FRONT GREEN LED
        self.fntRed = gpio.LED(self.pinAsgn["frontRGB_Green"[0]])

        #Bumper Switch
        self.bumperSW = gpio.Button(
            self.pinAsgn["frontBumper"[0]],
            False, #False = pulldown resistor
            True, #True = Active High
            None, #no bounce time because the bumper should NEVER be touched
            1, #time to wait until executing "when held"
            False, #No hold repeat
            None #Pin factory, used for SPI
            )
        
        #Color sensors
        
        #Left
        clrSens_L = gpio.InputDevice()

        #Center
        clrSens_Cnt = adafruit_tcs34725.TCS34725()
        
        #Right
        clrSens_R = gpio.InputDevice()

        #Ultrasonic Sensor
        self.ultSon = gpio.DistanceSensor(
            self.pinAsgn["ultraTrig"[0]],
            self.pinAsgn["ultraEcho"[0],
            9, #length of queue of read valuespinAsgn
            4.0, #max readable distance (meters)
            0.4, #IMPORTANT! THRESHOLD DISTANCE! TRIGGER IN RANGE DISTANCE FOR SENSOR
            False, #FALSE = report values ONLY after the queue has filled up
            None #pin factory
            ])
    # -------------------------------I2C SETUP FUNCTIONS------------------------------------- 
    
    # -------------------------------MOTOR COMPOUND FUNCTIONS------------------------------------- 
    def percToSpd(self,speed):
        #speed is taken as percentage and converted into 0 to 1 PWM value
        speed = speed/100
        return speed

    def halt(self):
        #stops all motion
        self.motorFL.stop()
        self.motorFR.stop()
        self.motorBL.stop()
        self.motorBR.stop()
        
    def moveOrTurnIncremental(self,direction,distanceMeters,speedPrcnt):
        
        self.distanceMoved = 0

        if speedPrcnt is None:
            #if no speed provided, default to full power
            speedPrcnt = 100

        match direction:
            case 'f':
                self.motorFL.forward(speed = self.percToSpd(speedPrcnt))
                self.motorFR.forward(speed = self.percToSpd(speedPrcnt))
                self.motorBL.forward(speed = self.percToSpd(speedPrcnt))
                self.motorBR.forward(speed = self.percToSpd(speedPrcnt))
            case 'b':
                self.motorFL.backward(speed = self.percToSpd(speedPrcnt))
                self.motorFR.backward(speed = self.percToSpd(speedPrcnt))
                self.motorBL.backward(speed = self.percToSpd(speedPrcnt))
                self.motorBR.backward(speed = self.percToSpd(speedPrcnt))
            case 'clk':
                self.motorFL.forward(speed = self.percToSpd(speedPrcnt))
                self.motorFR.backward(speed = self.percToSpd(speedPrcnt))
                self.motorBL.forward(speed = self.percToSpd(speedPrcnt))
                self.motorBR.backward(speed = self.percToSpd(speedPrcnt))
            case 'cntr_clk':
                self.motorFL.backward(speed = self.percToSpd(speedPrcnt))
                self.motorFR.forward(speed = self.percToSpd(speedPrcnt))
                self.motorBL.backward(speed = self.percToSpd(speedPrcnt))
                self.motorBR.forward(speed = self.percToSpd(speedPrcnt))
            case _:
                print("ERROR INVALID DIRECTION ON INCREMENTAL MOVEMENT FUNCTION")
                distanceMeters = 0

        while(self.distanceMoved < distanceMeters):
            #d = v * t
            sleep(self._MotorIncIntervalSeconds) #delays time in seconds
            self.distanceMoved += ((self.travelSpeed * speedPrcnt)) * (self._MotorIncIntervalSeconds)

        self.halt()

    #turns left or right incrementally
    def followContinuous(self,ctrl_PD,):
        self.motorFR.forward() 

    def resetGimbal(self):
        self.camServo.detach()

    def emergencyStop(self):
        #stops and then disconnects power from motors, program wont operate again until reset
        self.halt()

        self.motorFL.close()
        self.motorFR.close()
        self.motorBL.close()
        self.motorBR.close()
        self.camServo.value = None #will be able to freely move
        
    def moonWalk(self):
        #turns wheels inward and accomplishes nothing to test motors
        self.motorFL.backward()
        self.motorFR.backward()
        self.motorBL.forward()
        self.motorBR.forward()
        for i in range(10):
            self.camServo.value = -0.5
            sleep(1)
            self.camServo.value = 0.5
            sleep(1)
        self.halt()


class lightModes
    def __init__(self):


class compoundOperations:
    def _


mainPins = gpioDevices()


