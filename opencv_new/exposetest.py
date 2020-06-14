#!/local/bin/python3 

import cv2 
import subprocess

# open the capture
cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# set the exposure after opening the capture to
# avoid opencv modifying settings
command = "v4l2-ctl -d 0 -c auto_exposure=1 -c exposure_time_absolute=10"
output = subprocess.call(command, shell=True)

# watch your changes!
while(True): 
    ret, frame = cap.read() 
    cv2.imshow('frame', frame) 
    if( cv2.waitKey(1) & 0xFF == ord('q') ): 
        break 
cap.release() 
cv2.destroyAllWindows()