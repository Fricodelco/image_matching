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
class roi:
    img: np.ndarray
    lat_center: float = 0.0
    lon_center: float = 0.0
    pixel_x_main_map: int = 0
    pixel_y_main_map: int = 0
    pixel_size: float = 0.0


class match_finder():
    def __init__(self, map_, cadr, aproximate_angle):
        self.map = map_
        self.cadr = cadr 
        self.scale = Decimal(self.cadr.pixel_size/self.map.pixel_size)
        self.optimal_size_x = 400
        self.search_scale = 2
        self.aproximate_angle = aproximate_angle
        self.roi_img = None

    def find_map_roi_by_coordinates(self):
        #find the center of cadr coordinates at map
        x, y, z, lat, lon = self.find_center_of_image()
        x = -x
        if x < 0 or y < 0:
            print("no match")
            return 0
        #find the center pixel
        pixel_x = int(Decimal(x) / self.map.pixel_size) 
        pixel_y = int(Decimal(y) / self.map.pixel_size) 
        #find the corners
        width = self.cadr.rasterArray.shape[1]*self.search_scale
        height = self.cadr.rasterArray.shape[0]* self.search_scale
        x_min = int(pixel_x - height/2)
        x_max = int(pixel_x + height/2)
        y_min = int(pixel_y - width/2)
        y_max = int(pixel_y + width/2)
        #save to structure
        img = self.map.rasterArray[x_min:x_max, y_min:y_max]
        self.roi_img = roi(img, lat, lon, int(x_min+x_max)/2, int(y_min+y_max)/2, self.map.pixel_size)
        # img = self.resize_img(img, 0.3)
        # self.cadr.rasterArray = self.resize_img(self.cadr.rasterArray, 0.3)
        # cv2.imshow('cut', img)
        # cv2.imshow('cadr', self.cadr.rasterArray)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()        
    
    def rescale_for_optimal_sift(self):
        optimal_scale_for_cadr = self.optimal_size_x/self.cadr.rasterArray.shape[1]
        self.cadr.rasterArray = self.resize_img(self.cadr.rasterArray, optimal_scale_for_cadr)
        self.cadr.pixel_size = self.cadr.pixel_size/Decimal(optimal_scale_for_cadr)
        self.roi_img.img = self.resize_img(self.roi_img.img, optimal_scale_for_cadr)
        self.roi_img.pixel_size = self.roi_img.pixel_size/Decimal(optimal_scale_for_cadr)
        # cv2.imshow('cut', self.roi_img.img)
        # cv2.imshow('cadr', self.cadr.rasterArray)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()        
        # print(self.roi_img.img.shape, optimal_scale_for_cadr)
        
    def find_matches(self):
        img1 = self.roi_img.img
        img2 = self.cadr.rasterArray
        sift = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
            nOctaveLayers = 4,
            contrastThreshold = 0.04,
            edgeThreshold = 10,
            sigma = 1.6 )
        keypoints_1, descriptors_1 = sift.detectAndCompute(img1,None)
        
        keypoints_2, descriptors_2 = sift.detectAndCompute(img2,None)

        # print(len(keypoints_1), len(keypoints_2))
        # bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

        # matches = bf.match(descriptors_1,descriptors_2)
        # matches = sorted(matches, key = lambda x:x.distance)
        # print(len(matches), matches[0])
        # img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, good[:50], img2, flags=2)
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors_1,descriptors_2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.45*n.distance:
                good.append([m])
        print(len(good))
        img3 = cv2.drawMatchesKnn(img1,keypoints_1,img2,keypoints_2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow('img', img3)
        cv2.waitKey(0)
        cv2.destroyAllWindows()        
        
    def show_cadr_on_map(self):
        g_c = GeodeticConvert()
        g_c.initialiseReference(self.map.main_points[0].lat, self.map.main_points[0].lon, 0)
        img = self.map.rasterArray
        img = self.resize_img(img, 0.1)
        i = 0
        for point in self.cadr.main_points:
            x, y, _ = g_c.geodetic2Ned(point.lat, point.lon, 0)
            x = -x
            pixel_x = int(Decimal(0.1*x) / self.map.pixel_size)
            pixel_y = int(Decimal(0.1*y) / self.map.pixel_size)
            print(pixel_x, pixel_y)
            cv2.circle(img, (pixel_y, pixel_x), 10, 63*i, 10)
            i+=1
        lat = (self.cadr.main_points[0].lat + self.cadr.main_points[2].lat)/2
        lon = (self.cadr.main_points[0].lon + self.cadr.main_points[2].lon)/2
        x, y, _ = g_c.geodetic2Ned(lat, lon, 0)
        x = -x
        pixel_x = int(Decimal(0.1*x) / self.map.pixel_size)
        pixel_y = int(Decimal(0.1*y) / self.map.pixel_size)
        cv2.circle(img, (pixel_y, pixel_x), 10, 255, 10)
        cv2.imshow('img',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    

    def find_center_of_image(self):
        lat = (self.cadr.main_points[0].lat + self.cadr.main_points[2].lat)/2
        lon = (self.cadr.main_points[0].lon + self.cadr.main_points[2].lon)/2
        g_c = GeodeticConvert()
        g_c.initialiseReference(self.map.main_points[0].lat, self.map.main_points[0].lon, 0)
        x, y, z = g_c.geodetic2Ned(lat, lon, 0)
        return x, y, z, lat, lon
        

    def resize_cadr_by_scale(self):
        self.cadr.rasterArray = self.resize_img(self.cadr.rasterArray, self.scale)
        self.cadr.pixel_size = Decimal(self.cadr.pixel_size) / Decimal(self.scale)

    def resize_img(self, img, scale):
        width = int(img.shape[1] * Decimal(scale))
        height = int(img.shape[0] * Decimal(scale))
        dim = (width, height)  
        resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        return resized