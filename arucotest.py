import sys
import cv2
import numpy as np
from cv2 import aruco
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
args = vars(ap.parse_args())
 
img1 = cv2.imread(args["image"])
img2 = img1
img3 = img1
img4 = img1

boardsize = 1000 #size of the board, in pixels.

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000) #This defines which dictionary we choose to use.

img1 = aruco.drawMarker(aruco_dict, 1, boardsize, img1, 1)#Puts a marker in each image.
img2 = aruco.drawMarker(aruco_dict, 2, boardsize, img2, 1)
img3 = aruco.drawMarker(aruco_dict, 3, boardsize, img3, 1)
img4 = aruco.drawMarker(aruco_dict, 4, boardsize, img4, 1)

cv2.imwrite('board1.png',img1)#This saves each board as an image
cv2.imwrite('board2.png',img2)
cv2.imwrite('board3.png',img3)
cv2.imwrite('board4.png',img4)
cv2.imshow('board1',img1)#This shows each board to your screen.
cv2.imshow('board2',img2)
cv2.imshow('board3',img3)
cv2.imshow('board4',img4)

cv2.waitKey(0)#Press space to exit
