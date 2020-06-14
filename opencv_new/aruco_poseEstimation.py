"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""
import subprocess
import time
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

#--- Define Tag

marker_size  = 10 #- [cm]


#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])




#--- Get the camera calibration path
calib_path  = "/home/pi/Desktop/opencv_new/calibrationimages/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_7X7_250)
parameters  = aruco.DetectorParameters_create()
# [array([[[144., 370.],[197., 378.],
#      [179., 421.],
#         [123., 412.]]], dtype=float32), array([[[283., 396.],
#         [338., 401.],
#         [335., 451.],
#         [275., 444.]]], dtype=float32)] 

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
command = "v4l2-ctl -d 0 -c auto_exposure=1 -c exposure_time_absolute=200"
output = subprocess.call(command, shell=True)
#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
class find_marker_pos:
    def __init__(self,id_to_find,ret, frame,corners, ids,gray,calib_markers ):
        self.camera_x=0
        self.camera_y=0
        self.camera_z=0
        self.marker_x=0
        self.marker_y=0
        self.marker_z=0
        self.frame=frame
        self.run=False
        if ids is not None and  id_to_find in ids :
            id_pos=np.where(ids==id_to_find)
            
            id_pos=id_pos[0][0]
            corner=corners[id_pos:id_pos+1].copy()
            #corner	=	cv.cornerSubPix(	frame, cornera, (2,2), (-1,-1), criteria	)

            ret = aruco.estimatePoseSingleMarkers(corner, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
            pos_camera = -R_tc*np.matrix(tvec).T
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            self.camera_x=pos_camera[0].item((0,0))+calib_markers[0]
            self.camera_y=pos_camera[1].item((0,0))+calib_markers[1]
            self.camera_z=pos_camera[2].item((0,0))+calib_markers[2]
            self.marker_x=tvec[0]+calib_markers[0]
            self.marker_y=tvec[1]+calib_markers[1]
            self.marker_z=tvec[2]+calib_markers[2]
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            self.frame=frame
            # self.marker_x=pos_camera[0].item((0,0))+calib_markers[0]
            # self.marker_y=pos_camera[1].item((0,0))+calib_markers[1]
            # self.marker_z=pos_camera[2].item((0,0))+calib_markers[2]
            # self.camera_x=tvec[0]+calib_markers[0]
            # self.camera_y=tvec[1]+calib_markers[1]
            # self.camera_z=tvec[2]+calib_markers[2]
            self.run=True
        else:
            self.run=False

        
      
#calib_markers=[ [[0,0,0,False],[0,0,0,False],[0,0,0,False]],[[0,0,0,False],[0,0,0,False]],[[0,0,0,False],[0,0,0,False]],[[0,0,0,False],[0,0,0,False]]    ]
loop=True
count=0
max_dif=5
sample_times=20
while True:
    try:

        #-- Read the camera frame
        ret, frame = cap.read()

        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
        
        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        
        if ids is not None and 0 in ids and 1 in ids:
            while loop:
                calib_markers = ([[ [0.0 for col in range(4)] for col in range(len(ids[:,0]))] for row in range(len(ids[:,0]))])
                calib=np.array(calib_markers)
                #calib_markers = [[ [0 for col in range(4)] for col in range(2)] for row in range(2)]
                while loop:
                    # print(len(ids[:,0]),ids)
                    for x_id in ids:
                        orig_id=x_id[0]
                        for y_id in ids:
                            targ_id=y_id[0]
                            # print(orig_id,targ_id)
                            a=find_marker_pos(orig_id,ret, frame,corners, ids,gray,[0,0,0])
                            b=find_marker_pos(targ_id,ret, frame,corners, ids,gray,[0,0,0])
                            if a.run and b.run:
                                # for i in range(3):
                                #     print(orig_id,targ_id,i)
                                #     calib_markers[orig_id][targ_id][i]=((a.camera_x-b.camera_x)+calib_markers[orig_id][targ_id][i])
                                calib_markers[orig_id][targ_id][0]=((a.camera_x-b.camera_x)+calib_markers[orig_id][targ_id][0])
                                calib_markers[orig_id][targ_id][1]=((a.camera_y-b.camera_y)+calib_markers[orig_id][targ_id][1])
                                calib_markers[orig_id][targ_id][2]=((a.camera_z-b.camera_z)+calib_markers[orig_id][targ_id][2])
                                calib_markers[orig_id][targ_id][3]=calib_markers[orig_id][targ_id][3]+1

                    print(calib_markers)
                    
                    time.sleep(.5)
                    count+=1
                    if count == 10:
                        loop=False
                        x,y,z=calib.shape
                        for x in range(x):
                            for y in range(y):
                                print('a',calib_markers[x][y])
                                for i in range(3):
                                    if calib_markers[x][y][3]!=0:
                                        calib_markers[x][y][i]=calib_markers[x][y][i]/calib_markers[x][y][3]
                        print('here',calib_markers)
                        
        if ids is not None and count==10:                  
            a=find_marker_pos(0,ret, frame,corners, ids,gray,calib_markers[0][0])
            b=find_marker_pos(1,ret, frame,corners, ids,gray,calib_markers[0][1])
            frame=a.frame
            # if a.run:
            #     print(a.camera_x,a.camera_y,a.camera_z)
            
            # print(a.camera_x,a.camera_y,a.camera_z,b.camera_x,b.camera_y,b.camera_z)

            # b=find_marker_pos(1,ret, frame,corners, ids,gray,calib_markers[1])
            # c=find_marker_pos(2,ret, frame,corners, ids,gray,calib_markers[2])
            # d=find_marker_pos(3,ret, frame,corners, ids,gray)
            


                # print(a-11.280051523489965 ,e,b-(-15.876589060429176),f,c-5.4688461601026574,g)
                # print(a-e,b-f,c-g)
                # print(a-10.6666653724,e)
            # if a.run and b.run:
            #print(a.camera_x,a.camera_y,a.camera_z,b.camera_x,b.camera_y,b.camera_z)
            print('{:07.3f}'.format(a.camera_x-b.camera_x))
            #,a.camera_y-b.camera_y,a.camera_z-b.camera_z)
            # X=a.camera_x-b.camera_x
            # Y=a.camera_y-b.camera_y
            # Z=a.camera_z-b.camera_z
            # if abs(X)< max_dif and abs(Y)< max_dif and abs(Z)< max_dif:
            #     print(X,Y,Z)
            # else:
            #--- Display the frame
            #cv2.imshow('frame', frame)


            #     print(a.camera_x,b.camera_x,a.camera_y,b.camera_y,a.camera_z,b.camera_z)


           
    except KeyboardInterrupt:
        break
        exit()










    # if ids is not None and ids[0] == id_to_find:
    #     #print(ids[0],ids[1])
    #     #-- ret = [rvec, tvec, ?]
    #     #-- array of rotation and position of each marker in camera frame
    #     #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
    #     #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
    #     ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

    #     #-- Unpack the output, get only the first
    #     rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

    #     #-- Draw the detected marker and put a reference frame over it
    #     #aruco.drawDetectedMarkers(frame, corners)
    #     #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

    #     #-- Print the tag position in camera frame
    #     #str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
    #     #cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
    #     #-- Obtain the rotation matrix tag->camera
    #     R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
    #     R_tc    = R_ct.T

    #     #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
    #     roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

    #     #-- Print the marker's attitude respect to camera frame
    #     # str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker), math.degrees(yaw_marker))
    #     #cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        

    #     #-- Now get Position and attitude f the camera respect to the marker
    #     pos_camera = -R_tc*np.matrix(tvec).T

    #     #str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
    #     #cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    #     #-- Get the attitude of the camera respect to the frame
    #     roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
    #     #str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),math.degrees(yaw_camera))

    #     #cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    #     print(tvec[0], tvec[1], tvec[2])



    


    #--- Display the frame
    #cv2.imshow('frame', frame)

    #--- use 'q' to quit
    #key = cv2.waitKey(1) & 0xFF
    # if key == ord('q'):
    #     cap.release()
    #     cv2.destroyAllWindows()
    #     break
  


