import cv2 as cv
import numpy as np
import string

#camera feed settings
cameraWidth = 640
cameraHeight = 480
cameraChannelCnt = 3
thresh_value = 200

lightDirection = None

#HSV color definitions

hsvColors = {
    "yellowLo" : np.array([0, 27, 255]),
    "yellowHi" : np.array([30, 255, 255]),
    "greenLo" : np.array([38, 28, 173]), #38, 28, 173
    "greenHi" : np.array([79, 255, 255])
}

def createHSVMasks(baseFrame,frameColorName):
     loCase = frameColorName.lower()
     newImage = cv.inRange(baseFrame, hsvColors[loCase + "Lo"], hsvColors[loCase + "Hi"])
     return newImage

def apply_AND_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def drawCirclesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y), radius = cv.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv.circle(baseFrame, center, radius, (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame

cap = cv.VideoCapture(0) #frameRaw is BGR

cameraWidth = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cameraHeight = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
#camera_channel_count = cap.get(cv.CAP_PROP_VIDEO_TOTAL_CHANNELS)

while True:
        ret, frameRAW = cap.read()
        
        #DISCARD
        mask = np.zeros((cameraWidth, cameraHeight), dtype=np.uint8) #for reference in size for masks later on

        #baseFrameHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV) #HSV
        baseFrameGray = cv.cvtColor(frameRAW,cv.COLOR_BGR2GRAY) #GRAY

        maskGreenHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV)
        maskYellowHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV)

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
                #cv.circle(mask, center, radius, (0, 255, 255), cv.FILLED)
                cv.circle(i, center, radius, (255, 255, 255), cv.FILLED)
                #cv.drawContours(maskGreenHSV,[cnt],0,(255,255,255),cv.FILLED)
            #cv.putText(baseFrame, boxText, (x, y), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        greenLEDPixels = apply_AND_Mask(maskGreenHSV,maskBright)
        yellowLEDPixels = apply_AND_Mask(maskYellowHSV,maskBright)

        cv.imshow("frameRaw", frameRAW)
        cv.imshow("frameGray", baseFrameGray)
        cv.imshow("Mask Green ", greenLEDPixels)
        cv.imshow("Mask Yellow ", yellowLEDPixels)
        cv.imshow("maskBright", maskBright)

        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()