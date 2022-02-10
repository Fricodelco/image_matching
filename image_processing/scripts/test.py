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

def main():
    main_map = image_processing('/fon/26_12_2021_nn.TIF', 0)
    map_pixel_size = main_map.find_pixel_size()
    print(map_pixel_size)
    cadr = image_processing('/foto/13_12_54_15.jpg', 0)
    cadr_pixel_size = cadr.find_pixel_size()
    print(cadr_pixel_size)
    matcher = match_finder(main_map, cadr, 90)
    matcher.resize_cadr_by_scale()
    matcher.find_map_roi_by_coordinates()
    matcher.rescale_for_optimal_sift()
    matcher.find_matches()
    # matcher.show_cadr_on_map()
    # sift = cv2.xfeatures2d.SIFT_create()


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

