import cv2 as cv
import numpy as np
import os
from colorama import Fore, Back, Style, init

init() #initializes colorama for colored terminal text
os.chdir('/home/user/Documents/Oakland/4999Code/auxFiles') #ensures checking correct path for image test files

#stop line detection variables
dotsOnLeft = 0
dotsOnRight = 0
foundStopLineLeft = False
foundStopLineRight = False
foundPathForward = False
foundDirectionalDevice = False

path_linePhoto1 = 'LinePhoto1.jpg'
linePhoto1 = cv.imread(path_linePhoto1)
path_linePhoto2 = 'LinePhoto2.jpg'
linePhoto2 = cv.imread(path_linePhoto2)
path_linePhoto3 = 'LinePhoto3.jpg'
linePhoto3 = cv.imread(path_linePhoto3)

#camera feed settings
cameraWidth = 480
cameraHeight = 640
cameraChannelCnt = 3

#camera feed settings
#cameraWidth, cameraHeight, cameraChannelCnt = linePhoto1.shape

#HSV color definitions

hsvColors = {
"blueLo" : np.array([30, 130, 115]),
"blueHi" : np.array([115, 255, 255]),
"yellowHi" : np.array([30, 118, 255]),
"greenLo" : np.array([38, 28, 173]),
"greenHi" : np.array([79, 255, 255])
}

#create regions of interest
# creates region left most vertical third of the screen 
x_scrn, y_scrn, h_scrn = 0, 0, cameraHeight
#cuts the image into thirds verticaly
w_tri_1 = round(cameraWidth * (1/3))
w_tri_2 = round(cameraWidth * (2/3)) 
w_tri_3 = round(cameraWidth)
#cuts the image into thirds horizontally
h_tri_1 = round(cameraHeight * (1/3))
h_tri_2 = round(cameraHeight * (2/3))
h_tri_3 = round(cameraHeight)

#roiScreen = frameResized[y:yr+hr, x:xr+wr]

def createHSVMasks(baseFrame,frameColorName):
     loCase = frameColorName.lower()
     newImage = cv.inRange(baseFrame, hsvColors[loCase + "Lo"], hsvColors[loCase + "Hi"])
     return newImage

def apply_AND_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def apply_OR_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_or(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def drawBoxesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y,w,h) = cv.boundingRect(cnt)
        cv.rectangle(baseFrame, (x,y), (x + w, y + h), (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y-10), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame

def reportForDots():
    os.system('clear')
    if (dotsOnLeft >= 4):
        print(Fore.CYAN + "LEFT STOPLINE WAS DETECTED" + Style.RESET_ALL)
    if (dotsOnRight >= 4):
        print(Fore.CYAN + "RIGHT STOPLINE WAS DETECTED" + Style.RESET_ALL)    

def checkWithinROI(cornerImage):
        
        global dotsOnLeft,dotsOnRight
        dotsOnLeft = 0
        dotsOnRight = 0

        #stop line detector
        for corner in corners:
            x, y = corner.ravel()
            frameFilterCorners = cv.circle(frameResized, center=(x,y), radius = 8, color=(0,0,255), thickness=-1)

            if ((x >= (x_scrn)) and (x <= (x_scrn + w_tri_1)) and (y >= (y_scrn)) and (y <= (y_scrn+h_scrn))): #checks for left third of the screen
               if (y >= (y_scrn + h_tri_1)) and (y <= (y_scrn + h_tri_2)):
                    dotsOnLeft += 1
            
            if ((x >= (x_scrn + w_tri_2)) and (x <= (x_scrn + w_tri_3)) and (y >= (y_scrn)) and (y <= (y_scrn+h_scrn))): #checks for left third of the screen
               if (y >= (y_scrn + h_tri_1)) and (y <= (y_scrn + h_tri_2)):
                    dotsOnRight += 1

        return frameFilterCorners

def drawROI(baseFrame):
    baseFrame = cv.rectangle(baseFrame,(x_scrn,y_scrn),(x_scrn + w_tri_1,y_scrn + h_scrn),(0,255,0),thickness = 4) #left vertical tri
    baseFrame = cv.rectangle(baseFrame,(x_scrn + w_tri_2,y_scrn),(x_scrn + w_tri_3,y_scrn + h_scrn),(0,255,0),thickness = 4) #right vertical tri
    baseFrame = cv.rectangle(baseFrame,(x_scrn,y_scrn+h_tri_1),(x_scrn + w_tri_3,y_scrn + h_tri_2),(0,0,255),thickness = 4) #middle horizontal tri
    return baseFrame

def drawBoxesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y,w,h) = cv.boundingRect(cnt)
        cv.rectangle(baseFrame, (x,y), (x + w, y + h), (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y-10), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame


#initialize camera settings
cap = cv.VideoCapture(0)
camera_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
camera_height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
camera_channel_count = cap.get(cv.CAP_PROP_VIDEO_TOTAL_CHANNELS)

while True:
        #frameRAW = linePhoto1
        ret,frameRAW = cap.read(0)

        #frameResized = cv.resize(frameRAW,(cameraWidth,cameraHeight))
        frameResized = cv.resize(frameRAW,(cameraWidth,cameraHeight))

        #frameGray = cv.cvtColor(frameResized,cv.COLOR_BGR2GRAY)
        frameHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)
        frameBlue = createHSVMasks(frameHSV,"blue")
        blurFrame = cv.GaussianBlur(frameBlue,(51,51),3)
        detectedEdges = cv.Canny(blurFrame,50,180)
        detectedEdges = cv.GaussianBlur(detectedEdges,(5,5),3)

        corners = cv.goodFeaturesToTrack(detectedEdges, maxCorners = 100, qualityLevel = 0.02, minDistance=20.0,useHarrisDetector=True,k=0.1)
        if corners is not None:
            corners = np.int0(corners)
            frameFilterCorners = checkWithinROI(corners)
        else:
            frameFilterCorners = detectedEdges
        
        #frameFilterCorners = drawROI(frameRAW)
        frameFilterCorners = drawROI(frameResized)
        #to identify stop lines, I first need to isolate all lines that are associated with the color blue

        cv.imshow("BaseFrame", frameResized)
        cv.imshow("BlueFrame", frameBlue)
        cv.imshow("BlurFrame", blurFrame)
        cv.imshow("EdgeFrame", detectedEdges)
        cv.imshow("Corners", frameFilterCorners)

        reportForDots()

        key = cv.waitKey(1)
        if key == 27:
            break
        
cap.release()
cv.destroyAllWindows()
