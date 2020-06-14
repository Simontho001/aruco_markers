import cv2
import numpy as np
def nothing(x):
    # any operation
    pass
cap = cv2.VideoCapture(0)



lower_red = np.array([0, 0, 80])
upper_red = np.array([0, 0, 100])
mask = cv2.inRange('hsv', lower_red, upper_red)
kernel = np.ones((5, 5), np.uint8)
mask = cv2.erode(mask, kernel)
_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


area = cv2.contourArea(cnt)
approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
x = approx.ravel()[0]
y = approx.ravel()[1]
if area > 400:
    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)
    if len(approx) == 3:
        cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))
    elif len(approx) == 4:
        cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
    elif 10 < len(approx) < 20:
        cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))