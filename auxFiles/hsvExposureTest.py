import cv2 as cv
import numpy as np

Winname = "Frame:"

def nothing(x):
    pass

cv.namedWindow('Frame:')
# H, S,V are for Lower Boundaries
#H2,S2,V2 are for Upper Boundaries
cv.createTrackbar('H',Winname,0,255,nothing)
cv.createTrackbar('S',Winname,0,255,nothing)
cv.createTrackbar('V',Winname,0,255,nothing)
cv.createTrackbar('H2',Winname,0,255,nothing)
cv.createTrackbar('S2',Winname,0,255,nothing)
cv.createTrackbar('V2',Winname,0,255,nothing)


cap = cv.VideoCapture(0)

while cap.isOpened():
    _, frame = cap.read()
    H = cv.getTrackbarPos('H', 'Frame:')
    S = cv.getTrackbarPos('S', 'Frame:')
    V = cv.getTrackbarPos('V', 'Frame:')
    H2 = cv.getTrackbarPos('H2', 'Frame:')
    S2 = cv.getTrackbarPos('S2', 'Frame:')
    V2 = cv.getTrackbarPos('V2', 'Frame:')
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_boundary = np.array([H, S, V])
    upper_boundary = np.array([H2,S2,V2])
    mask = cv.inRange(hsv, lower_boundary, upper_boundary)
    final = cv.bitwise_and(frame, frame, mask=mask)
    cv.imshow("Frame:", final)

    if cv.waitKey(1) == ord('q'): break

cap.release()
cv.destroyAllWindows()