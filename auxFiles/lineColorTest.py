import cv2 as cv
import numpy as np
import os

os.chdir('/home/user/Documents/Oakland/4999Code/auxFiles') #ensures checking correct path for image test files

#camera feed settings
cameraWidth = 480
cameraHeight = 640
cameraChannelCnt = 3
Winname = "Frame:"

def nothing(x):
    pass

path_linePhoto1 = 'LinePhoto1.jpg'
linePhoto1 = cv.imread(path_linePhoto1)

cv.namedWindow('Frame:')
# H, S,V are for Lower Boundaries
#H2,S2,V2 are for Upper Boundaries
cv.createTrackbar('H',Winname,0,255,nothing)
cv.createTrackbar('S',Winname,0,255,nothing)
cv.createTrackbar('V',Winname,0,255,nothing)
cv.createTrackbar('H2',Winname,0,255,nothing)
cv.createTrackbar('S2',Winname,0,255,nothing)
cv.createTrackbar('V2',Winname,0,255,nothing)

cap = linePhoto1
cap = cv.resize(cap,(cameraWidth,cameraHeight))

while True:
    H = cv.getTrackbarPos('H', 'Frame:')
    S = cv.getTrackbarPos('S', 'Frame:')
    V = cv.getTrackbarPos('V', 'Frame:')
    H2 = cv.getTrackbarPos('H2', 'Frame:')
    S2 = cv.getTrackbarPos('S2', 'Frame:')
    V2 = cv.getTrackbarPos('V2', 'Frame:')
    hsv = cv.cvtColor(cap, cv.COLOR_BGR2HSV)
    lower_boundary = np.array([H, S, V])
    upper_boundary = np.array([H2,S2,V2])
    mask = cv.inRange(hsv, lower_boundary, upper_boundary)
    final = cv.bitwise_and(cap, cap, mask=mask)
    cv.imshow("Frame:", final)

    if cv.waitKey(1) == ord('q'): break

cap.release()
cv.destroyAllWindows()
