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

@dataclass
class img_point:
    pixel_y: int = 0
    pixel_x: int = 0
    lat: float = 0.0
    lon: float = 0.0

class image_processing():
    def __init__(self, filename = None, height = None, img = None):
        self.main_points = []
        self.alt = 0
        self.g_c = GeodeticConvert()
        self.rasterArray = None
        self.pixel_size = 0
        if filename is not None:
            rospack = rospkg.RosPack()
            data_path = rospack.get_path('image_processing') + '/data'
            raster = gdal.Open(data_path+'/'+filename)
            self.rasterArray = raster.ReadAsArray()
            self.rasterArray = np.dstack((self.rasterArray[0],self.rasterArray[1],self.rasterArray[2]))
            # self.rasterArray = self.brightness_norm(self.rasterArray)
            # self.rasterArray = self.clahe_m(self.rasterArray, 5)
            # self.rasterArray = cv2.cvtColor(self.rasterArray, cv2.COLOR_RGB2GRAY)
            self.rasterArray = self.rasterArray[:,:,0]
            # norm_img = np.zeros(self.rasterArray.shape)
            # self.rasterArray = cv2.normalize(self.rasterArray,  norm_img, 0, 255, cv2.NORM_MINMAX)
            # print(self.rasterArray.shape)
            with open(data_path+'/'+filename[:-4]+'.@@@') as f:
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
            self.rasterArray = img
            # self.rasterArray = self.clahe_m(self.rasterArray, 1)
            # self.rasterArray = self.brightness_norm(self.rasterArray)
            # self.rasterArray = np.dstack((self.rasterArray[0],self.rasterArray[1],self.rasterArray[2]))
            # self.rasterArray = cv2.cvtColor(self.rasterArray, cv2.COLOR_BGR2GRAY)
            self.rasterArray = self.rasterArray[:,:,2]
            # norm_img = np.zeros(self.rasterArray.shape)
            # self.rasterArray = cv2.normalize(self.rasterArray,  norm_img, 0, 255, cv2.NORM_MINMAX)
        if height is not None:
            self.alt = height
        else:
            height = 2000
    
    def find_pixel_size(self):
        self.g_c.initialiseReference(self.main_points[0].lat, self.main_points[0].lon, self.alt)
        x_1, y_1, z_1 = self.g_c.geodetic2Ned(self.main_points[1].lat, self.main_points[1].lon, self.alt)
        x_2, y_2, z_2 = self.g_c.geodetic2Ned(self.main_points[3].lat, self.main_points[3].lon, self.alt)
        if abs(x_1) > abs(x_2):
            x = x_1
        else:
            x = x_2
        if abs(y_1) > abs(y_2):
            y = y_1
        else:
            y = y_2
        pixel_size_1 = Decimal((abs(x)))/Decimal(self.rasterArray.shape[0])
        pixel_size_2 = Decimal((abs(y)))/Decimal(self.rasterArray.shape[1])
        pixel_size = (Decimal(pixel_size_1) + Decimal(pixel_size_2))/Decimal(2)
        self.pixel_size = pixel_size
        return pixel_size

# def main():
    # map_ = image_processing(filename = '26_12_2021_nn')
    # map_.find_pixel_size()

# if __name__ == '__main__':
    # main()


