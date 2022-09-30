import cv2
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
# Dimensions in cm
marker_length = 2.25
marker_separation = 0.3
arucoParams = aruco.DetectorParameters_create()
board = aruco.GridBoard_create(5, 7, marker_length, marker_separation, aruco_dict)