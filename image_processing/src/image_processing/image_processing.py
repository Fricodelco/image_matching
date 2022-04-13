#!/usr/bin/env python3  
from cv2 import resize
from numpy import reshape
import rospy
import cv2
import numpy as np
from PIL import Image
import rospkg
import gdal
from dataclasses import dataclass
from geodetic_conv import GeodeticConvert
from decimal import Decimal
import os
from time import time

@dataclass
class img_point:
    pixel_y: int = 0
    pixel_x: int = 0
    lat: float = 0.0
    lon: float = 0.0

class image_processing():
    def __init__(self, filename = None, img = None):
        self.cuda = self.is_cuda_cv()
        self.main_points = []
        self.g_c = GeodeticConvert()
        self.img = None
        self.pixel_size = 0
        self.kp = None
        self.dp = None
        self.cadr_scale = 0.0
        time_start = time()
        if filename is not None:
            home = os.getenv("HOME")
            data_path = home+'/copa5/map'
            file_exists = os.path.exists(data_path+'/'+filename+'.tif')
            # print("start job")
            try:
                if file_exists is True:
                    # raster = gdal.Open(data_path+'/'+filename+'.tif')
                    self.img = cv2.imread(data_path+'/'+filename+'.tif')
                else:
                    # raster = gdal.Open(data_path+'/'+filename+'.TIF')
                    self.img = cv2.imread(data_path+'/'+filename+'.TIF')
            except:
                # print("NO MAP FILE")
                return None
            # print("map loaded", time() - time_start)
            time_start = time()
            # self.img = raster.ReadAsArray()
            # self.img = np.dstack((self.img[0],self.img[1],self.img[2]))
            # self.img = self.img[0]
            self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)
            # print("to gray complete", time() - time_start)
            time_start = time()
            with open(data_path+'/'+filename+'.@@@') as f:
                lines = f.readlines()
                for i in range(2, len(lines)):
                    sub_str = lines[i].split(' ')
                    sub_str = [j for j in sub_str if j]
                    try:
                        sub_str.remove('\n')
                    except:
                        e = 0
                    sub_str = [float(k) for k in sub_str]
                    point = img_point(sub_str[0], sub_str[1],
                                    sub_str[2], sub_str[3])
                    self.main_points.append(point)
        else:
            self.img = img
            # self.img = self.img[:,:,2]

    def find_pixel_size(self):
        self.g_c.initialiseReference(self.main_points[0].lat, self.main_points[0].lon, 0)
        x_1, y_1, z_1 = self.g_c.geodetic2Ned(self.main_points[1].lat, self.main_points[1].lon, 0)
        x_2, y_2, z_2 = self.g_c.geodetic2Ned(self.main_points[3].lat, self.main_points[3].lon, 0)
        if abs(x_1) > abs(x_2):
            x = x_1
        else:
            x = x_2
        if abs(y_1) > abs(y_2):
            y = y_1
        else:
            y = y_2
        pixel_size_1 = Decimal((abs(x)))/Decimal(self.img.shape[0])
        pixel_size_2 = Decimal((abs(y)))/Decimal(self.img.shape[1])
        pixel_size = (Decimal(pixel_size_1) + Decimal(pixel_size_2))/Decimal(2)
        self.pixel_size = pixel_size
        return pixel_size
    
    def find_pixel_size_by_height(self, height, poi):
        x = Decimal(np.tanh(poi/2)*2*height)
        self.pixel_size = x/Decimal(self.img.shape[1])
    
    def is_cuda_cv(self):
        try:
            count = cv2.cuda.getCudaEnabledDeviceCount()
            if count > 0:
                # print("CUDA IS ENABLED")
                return True
            else:
                # print("CUDA IS DISABLED")
                return False
        except:
            # print("CUDA IS DISABLED")
            return False

    def find_kp_dp_scale(self, match_finder):
        self.img, self.cadr_scale, self.pixel_size = match_finder.rescale_cadr(self.img, self.pixel_size)
        self.kp, self.dp, self.img = match_finder.find_kp_dp(self.img)

        
# def main():
    # map_ = image_processing(filename = '26_12_2021_nn')
    # map_.find_pixel_size()

# if __name__ == '__main__':
    # main()


