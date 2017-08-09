import numpy as np
import serial
import cv2
import glob
import yaml
from cv2 import aruco
import os
import sys
import time

from picamera.array import PiRGBArray
from picamera import PiCamera
camera = PiCamera()
camera.resolution = (1280,720)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(1280,720))
time.sleep(0.1)





with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)
mtxloaded = loadeddict.get('camera_matrix')
distloaded = loadeddict.get('dist_coeff')

camera_matrix = np.array(mtxloaded)
dist_coeffs = np.array(distloaded)



image_size = (1280,720) #size of input image.  From webcam, this will be 1920, 1080
aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 ) #Must be same as dictionary I made boards from
markerLength = 6.25 #Dimensions can be anything, but will match what is later returned in tvec
markerSeparation = .7 #only matters for boards
board = aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
arucoParams = aruco.DetectorParameters_create() #Just use this


for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port=True): #Can also be a while True loop
    imgRemapped = frame.array
    imgRemapped_gray = cv2.cvtColor(imgRemapped, cv2.COLOR_BGR2GRAY)    # aruco.detectMarkers() requires gray image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(imgRemapped_gray, aruco_dict, parameters=arucoParams) # Detect aruco marker.
    if ids != None: # if aruco marker detected
        tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)[1] # For a single marker
        rvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)[0]
        imgWithAruco = aruco.drawDetectedMarkers(imgRemapped, corners, ids, (0,255,0)) #Draws a box around the marker based on its corners, in green. Labels it with its id.
        imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 12)    # axis length is last parameter.  The unit of this matches input unit. Can be feet, inches, anything.
        time.sleep(.001)
        x,y,z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2] #tvec stores position
        print(x,y,z, ids[0][0])

        #ser.write(' ')
        if(ids[0][0] == 1): #If I see the first marker, draw red
            color = 'r'
        if(ids[0][0] == 2): #If I see second marker, draw blue
            color = 'b'
        else:
            color = 'g'
        #print(rvec[0][0][1])
        ##ax.scatter(x, z, -y, c=color, marker='o') #Plots my points in a way that's easier to visualize.
       
    else:   # if aruco marker is NOT detected
        imgWithAruco = imgRemapped
        time.sleep(.001)# assign imRemapped_color to imgWithAruco directly
        
    cv2.imshow("aruco", imgWithAruco)   # display
    rawCapture.truncate(0)
    if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
        break
##ax.set_xlabel('Left or Right')
##ax.set_ylabel('Depth')
##ax.set_zlabel('Height')

##plt.show()

##while True:
##    plt.pause(.05)


