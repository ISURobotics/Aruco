import numpy as np
import cv2
import glob
import yaml
from cv2 import aruco
import os
import sys
import time

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOCUS, 0)
mystring = 'checkerboardcals'
for i in range(0,50):
    retval, img = cap.read()
    time.sleep(1)
    tempstring = mystring + str(i) + '.jpg'
    print(tempstring)
    cv2.imshow('im',img)
    cv2.waitKey(250)
    cv2.imwrite(tempstring,img)
cap.release()
