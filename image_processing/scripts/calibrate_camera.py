# import numpy as np
# import cv2
# import cv2.aruco as aruco
# import pathlib
# import os

# def calibrate_aruco(marker_length, marker_separation):
    
#     aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
#     arucoParams = aruco.DetectorParameters_create()
#     board = aruco.GridBoard_create(4, 2, marker_length, marker_separation, aruco_dict)

#     counter, corners_list, id_list = [], [], []
#     first = 0
#     # Find the ArUco markers inside each image
#     home = os.getenv("HOME")
#     data_path = home+'/copa5/video/cmd_video.mkv'
#     iter_ = 0
#     cap = cv2.VideoCapture(data_path, cv2.CAP_FFMPEG)
#     while(cap.isOpened()):
#         print("read img")
#         ret, frame = cap.read()
#         iter_ += 1
#         if ret != True:
#             break
#         img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         if iter_ >= 10:
#             iter_ = 0
#             corners, ids, rejected = aruco.detectMarkers(
#                 img_gray, 
#                 aruco_dict, 
#                 parameters=arucoParams
#             )
#             if ids is not None and len(ids)>0:
#                 if first == 0:
#                     corners_list = corners
#                     id_list = ids
#                 else:
#                     corners_list = np.vstack((corners_list, corners))
#                     id_list = np.vstack((id_list,ids))
#                 first = first + 1
#                 counter.append(len(ids))
#                 print("calculated")
#         if cv2.waitKey(25) & 0xFF == ord('q'):
#             break
#     print("begin calibrating")
#     counter = np.array(counter)
#     # Actual calibration
#     print(type(id_list))
#     ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(
#         corners_list, 
#         id_list,
#         counter, 
#         board, 
#         img_gray.shape, 
#         None, 
#         None 
#     )
#     return [ret, mtx, dist, rvecs, tvecs]

# def save_coefficients(mtx, dist, path):
#     '''Save the camera matrix and the distortion coefficients to given path/file.'''
#     home = os.getenv("HOME")
#     path = str(home)+'/copa5/ws/src/image_matching/image_processing/camera_params/params.yml'
#     cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
#     cv_file.write('K', mtx)
#     cv_file.write('D', dist)
#     # note you *release* you don't close() a FileStorage object
#     cv_file.release()

# def load_coefficients(path):
#     '''Loads camera matrix and distortion coefficients.'''
#     # FILE_STORAGE_READ
#     cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

#     # note we also have to specify the type to retrieve other wise we only get a
#     # FileNode object back instead of a matrix
#     camera_matrix = cv_file.getNode('K').mat()
#     dist_matrix = cv_file.getNode('D').mat()

#     cv_file.release()
#     return [camera_matrix, dist_matrix]

# if __name__ == '__main__':
#     # IMAGES_DIR = 'path_to_images'
#     # IMAGES_FORMAT = '.jpg'
#     # # Dimensions in m
#     MARKER_LENGTH = 0.044
#     MARKER_SEPARATION = 0.022

#     # Calibrate 
#     ret, mtx, dist, rvecs, tvecs = calibrate_aruco(
#         MARKER_LENGTH,
#         MARKER_SEPARATION
#     )
#     print(ret)
#     print("skdlsd")
#     print(mtx)
#     print("skdlsd")
#     print(dist)
#     # Save coefficients into a file
#     save_coefficients(mtx, dist, "calibration_aruco.yml")

#     # # Load coefficients
#     # mtx, dist = load_coefficients('calibration_aruco.yml')
#     # original = cv2.imread('image.jpg')
#     # dst = cv2.undistort(img, mtx, dist, None, None)
#     # cv2.imwrite('undist.jpg', dst)

import cv2
# assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
print(cv2.__version__)
import numpy as np
import os
import glob
CHECKERBOARD = (5,8)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')
home = os.getenv("HOME")
data_path = home+'/copa5/video/cmd_video_chess_1.mkv'
iter_ = 0
cap = cv2.VideoCapture(data_path, cv2.CAP_FFMPEG)
while(cap.isOpened()):
    ret, img = cap.read()
    if ret != True:
        break
    iter_ += 1
    if iter_ > 5:
        iter_ = 0
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        print(ret, corners)
        # If found, add object points, image points (after refining them)
        # cv2.drawChessboardCorners(img, (5, 8), corners, ret)
        # cv2.imshow('img',img)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
            imgpoints.append(corners)

N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")
