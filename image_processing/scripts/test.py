#!/usr/bin/env python3  
from numpy import reshape
import rospy
import cv2
import numpy as np
from PIL import Image
import rospkg
import gdal
from image_processing import image_processing
from decimal import Decimal
from match_finder import match_finder
from time import time
import copy

def match_from_image():
    # main_map = image_processing('UD_data/fon/Kar_19_03k.TIF', 0)
    # main_map = image_processing('UD_data/known/map/poligon_24_09.jpg', 0)
    main_map = image_processing('fon/26_12_2021_nn.TIF', 0)
    map_pixel_size = main_map.find_pixel_size()
    # cadr = image_processing('UD_data/known/photo/C0002_023_0000037158.jpg', 0)
    # cadr = image_processing('UD_data/known/foto/17_39_42_4.jpg', 0)
    cadr = image_processing('foto/13_12_54_15.jpg', 0)
    cadr_pixel_size = cadr.find_pixel_size()
    matcher = match_finder()
    scale, map_pixel_bigger = matcher.find_scale(map_pixel_size, cadr_pixel_size)
    if map_pixel_bigger is True:
        cadr.rasterArray, cadr.pixel_size = matcher.resize_by_scale(
                                cadr.rasterArray, cadr.pixel_size, scale)
    else:
        #need copy of full size map for future
        main_map.rasterArray, main_map.pixel_size = matcher.resize_by_scale(
                            main_map.rasterArray, main_map.pixel_size, scale)

    roi = matcher.find_map_roi_by_coordinates(main_map, cadr)
    # print(roi)
    cadr_rescaled = copy.copy(cadr)
    roi, cadr_rescaled = matcher.rescale_for_optimal_sift(roi, cadr_rescaled)
    kp_1, kp_2, good = matcher.find_matches(roi.img, cadr_rescaled.rasterArray)
    percent_of_good = (len(good)/len(kp_1))*100
    print(percent_of_good)
    if percent_of_good > matcher.percent_of_good_value:
        x_center, y_center, roll, pitch, yaw, M = matcher.find_keypoints_transform(kp_1, kp_2, good, roi.img, cadr_rescaled.rasterArray)
        height = 2000
        lat, lon = matcher.solve_IK(x_center, y_center, roll, pitch, yaw, height, roi, main_map)
        img = matcher.resize_img(main_map.rasterArray, 0.2)
        pixel_size = main_map.pixel_size/Decimal(0.2)
        img = matcher.draw_circle_on_map_by_coord(img,
                                    (main_map.main_points[0].lat, main_map.main_points[0].lon),
                                    (lat, lon), pixel_size)
        cv2.imshow('img', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    match_from_image()


if __name__ == '__main__':
    main()


#LOAD



# rospack = rospkg.RosPack()
# data_path = rospack.get_path('image_processing') + '/data/fon'
# raster = gdal.Open(data_path+'/26_12_2021_nn.TIF')
# cols = raster.RasterXSize
# rows = raster.RasterYSize
# bands = raster.RasterCount
# rasterArray = raster.ReadAsArray()
# #CONVERT TO IMAGE
# rasterArray = np.dstack((rasterArray[0],rasterArray[1],rasterArray[2]))
# #RESIZE
# scale_percent = 5 # percent of original size
# width = int(rasterArray.shape[1] * scale_percent / 100)
# height = int(rasterArray.shape[0] * scale_percent / 100)
# dim = (width, height)  
# resized = cv2.resize(rasterArray, dim, interpolation = cv2.INTER_AREA)
# #CANNY
# gray = cv2.cvtColor(resized,cv2.COLOR_RGB2GRAY)
# edges = cv2.Canny(gray,100,200)
# #SHOW
# cv2.imshow('gray', gray)
# cv2.imshow('canny', edges)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

