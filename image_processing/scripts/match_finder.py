#!/usr/bin/env python3  
import rospy
import cv2
import numpy as np
from PIL import Image
import rospkg
import gdal
from dataclasses import dataclass
from geodetic_conv import GeodeticConvert
from decimal import Decimal
import matplotlib.pyplot as plt
from utils import resize_img, rotate_image, line_intersection, isConvex
import math
@dataclass
class roi:
    img: np.ndarray
    pixel_x_main_map: int = 0
    pixel_y_main_map: int = 0
    pixel_size: float = 0.0


class match_finder():
    def __init__(self):
        #parameters
        self.optimal_size_x = 250
        self.points_quality = 0.9
        
        self.search_scale = 2
        self.roi_img = None
        self.percent_of_good_value = 1.0

    def find_scale(self, pixel_size_map, pixel_size_cadr):
        if pixel_size_map > pixel_size_cadr:
            scale = Decimal(pixel_size_cadr/pixel_size_map)
            map_pixel_bigger = True
        else:
            map_pixel_bigger = False
            scale = Decimal(pixel_size_map/pixel_size_cadr)
        return scale, map_pixel_bigger

    def find_map_roi_by_coordinates(self, map_, cadr, lat, lon, search_scale):
        #find the center of cadr coordinates at map
        g_c = GeodeticConvert()
        g_c.initialiseReference(map_.main_points[0].lat, map_.main_points[0].lon, 0)
        y, x, z = g_c.geodetic2Ned(lat, lon, 0)
        # if x < 0 or y < 0:
            # print("no match")
            # return 0
        #find the center pixel
        pixel_x = int(Decimal(x) / map_.pixel_size)
        pixel_y = int(Decimal(-y) / map_.pixel_size)
        #find the corners
        width = cadr.rasterArray.shape[1]*search_scale
        height = cadr.rasterArray.shape[0]*search_scale
        x_min = int(pixel_x - width/2)
        x_max = int(pixel_x + width/2)
        y_min = int(pixel_y - height/2)
        y_max = int(pixel_y + height/2)
        if x_min < 0: x_min = 0
        if y_min < 0: y_min = 0
        if x_max > map_.rasterArray.shape[1]: x_max = map_.rasterArray.shape[1]
        if y_max > map_.rasterArray.shape[0]: y_max = map_.rasterArray.shape[0]
        # print(x_min, x_max, y_min, y_max)
        #save to structure
        img = map_.rasterArray[y_min:y_max, x_min:x_max]
        roi_img = roi(img, x_min, y_min, map_.pixel_size)
        return roi_img

    def roi_from_map(self, map_, rl_size_x, rl_size_y):
        map_shape_x = map_.rasterArray.shape[0]
        map_shape_y = map_.rasterArray.shape[1]
        count_x = math.ceil(map_shape_x/rl_size_x)
        count_y = math.ceil(map_shape_y/rl_size_y)
        borders = []
        for i in range(0, count_x*2 - 2):
            for j in range(0, count_y*2 - 2):
                x_min = int(((rl_size_x/2)*i))
                x_max = int(x_min + (rl_size_x))
                y_min = int(((rl_size_y/2)*j))
                y_max = int(y_min+(rl_size_y))
                if x_min < 0: x_min = 0
                if y_min < 0: y_min = 0
                if x_max > map_.rasterArray.shape[0]: x_max = map_.rasterArray.shape[0]
                if y_max > map_.rasterArray.shape[1]: y_max = map_.rasterArray.shape[1]
                borders.append([x_min, x_max, y_min, y_max])
        # roi_img = roi(img, x_min, y_min, map_.pixel_size)
        # return roi_img
        return borders
    
    def create_roi_from_border(self, map_, border):
        x_min = border[0]
        x_max = border[1]
        y_min = border[2]
        y_max = border[3]
        img = map_.rasterArray[x_min:x_max, y_min:y_max]
        roi_ = roi(img, x_min, y_min, map_.pixel_size)
        return roi_    

    def roi_full_map(self, map_):
        # shape = map_.rasterArray.shape
        # img = map_.rasterArray[:int(0.7*shape[0]),:]
        img = map_.rasterArray
        roi_img = roi(img, 0, 0, map_.pixel_size)
        return roi_img

    def roi_from_last_xy(self, map_, x_meter, y_meter, cadr, search_scale, yaw):
        #find the corners
        # if(abs(np.cos(yaw)) < np.cos(np.pi/4)):
        pixel_x = x_meter/float(map_.pixel_size)
        pixel_y = y_meter/float(map_.pixel_size)
        width = cadr.rasterArray.shape[1]*search_scale
        height = cadr.rasterArray.shape[0]*search_scale
        if width > height:
            height = width
        else:
            width = height
        # else:
            # width = cadr.rasterArray.shape[0]*search_scale
            # height = cadr.rasterArray.shape[1]*search_scale
        x_min = int(pixel_x - width/2)
        x_max = int(pixel_x + width/2)
        y_min = int(pixel_y - height/2)
        y_max = int(pixel_y + height/2)
        if x_min < 0: x_min = 0
        if y_min < 0: y_min = 0
        if x_max > map_.rasterArray.shape[1]: x_max = map_.rasterArray.shape[1]
        if y_max > map_.rasterArray.shape[0]: y_max = map_.rasterArray.shape[0]
        #save to structure
        img = map_.rasterArray[y_min:y_max, x_min:x_max]
        roi_img = roi(img, x_min, y_min, map_.pixel_size)
        return roi_img

    def rescale_for_optimal_sift(self, roi, cadr):
        optimal_scale_for_cadr = self.optimal_size_x/cadr.rasterArray.shape[1]
        cadr.rasterArray = resize_img(cadr.rasterArray, optimal_scale_for_cadr)
        cadr.pixel_size = cadr.pixel_size/Decimal(optimal_scale_for_cadr)
        roi.img = resize_img(roi.img, optimal_scale_for_cadr)
        roi.pixel_size = roi.pixel_size/Decimal(optimal_scale_for_cadr)
        return roi, cadr
        

    def find_matches(self, img2, img1):
        surf = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
            nOctaveLayers = 5,
            contrastThreshold = 0.1,
            edgeThreshold = 40,
            sigma = 1.6)
        keypoints_1, descriptors_1 = surf.detectAndCompute(img1,None)
        keypoints_2, descriptors_2 = surf.detectAndCompute(img2,None)

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors_1,descriptors_2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < self.points_quality*n.distance:
                good.append([m])
        img3 = cv2.drawMatchesKnn(img1,keypoints_1,img2,keypoints_2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)     
        return keypoints_1, keypoints_2, good, img3, descriptors_1, descriptors_2

    def find_good_matches(self, descriptors_1, descriptors_2):
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors_2,descriptors_1,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < self.points_quality*n.distance:
                good.append([m])
        return good

    def find_keypoints_transform(self, kp1, kp2, matches, img2, img1):
        src_pts = np.float32([ kp1[m[0].queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m[0].trainIdx].pt for m in matches]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.USAC_MAGSAC, 1.0)
        matchesMask = [[0,0] for i in range(len(matches))]
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        pts_vis = [[0,0],[0,h-1],[w-1,h-1],[w-1,0]]
        dst = cv2.perspectiveTransform(pts,M)
        if isConvex(dst, img1.shape) is True:
            roll, pitch, yaw = self.get_angles_from_homography(M)
            x_center, y_center = line_intersection((dst[0][0], dst[2][0]), (dst[1][0], dst[3][0]))
            #draw
            img2 = cv2.circle(img2, (int(x_center), int(y_center)), 10, 255, 5)
            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            return x_center, y_center, roll, pitch, yaw, M, img2
        else:
            return None, None, None, None, None, None, None
    
    def solve_IK(self, x_center, y_center, height, roll, pitch, yaw, roi, map_):
        delta_pitch_pixels = -1*height*np.sin(pitch+0.3)/float(roi.pixel_size)
        delta_roll_pixels = height*np.sin(roll)/float(roi.pixel_size)

        x_center = x_center + delta_pitch_pixels*np.cos(yaw) + delta_roll_pixels*np.sin(yaw)
        y_center = y_center - delta_pitch_pixels*np.sin(yaw) + delta_roll_pixels*np.cos(yaw)
        scale_roi_to_map = roi.pixel_size/map_.pixel_size
        roi_shape_x = roi.img.shape[1] * float(scale_roi_to_map)
        roi_shape_y = roi.img.shape[0] * float(scale_roi_to_map)
        scaled_x_center = Decimal(Decimal(float(x_center))*scale_roi_to_map)
        scaled_y_center = Decimal(Decimal(float(y_center))*scale_roi_to_map)
        y = Decimal(roi.pixel_y_main_map) + Decimal(scaled_y_center)
        x = Decimal(roi.pixel_x_main_map) + Decimal(scaled_x_center)
        x_meter = x*map_.pixel_size
        y_meter = y*map_.pixel_size         
        g_c = GeodeticConvert()
        g_c.initialiseReference(map_.main_points[0].lat, map_.main_points[0].lon, 0)
        lat, lon, _ = g_c.ned2Geodetic(north=float(-y_meter), east=float(x_meter), down=0)
        return lat, lon, x, y, x_meter, y_meter
        

    def get_angles_from_homography(self, H):
        #eject the yaw transform
        #[c -s 0 0]
        #[s c  0 0]
        #[0 0  1 0]
        #[0 0  0 1]
        u, _, vh = np.linalg.svd(H[0:2, 0:2])
        R = u @ vh
        yaw = np.arctan2(R[0,0], R[1,0])
        # yaw = np.arctan2(H[1,0], H[0,0]) - np.pi
        # roll
        #[1 0 0  0]
        #[0 c -s 0]
        #[0 s c  0]
        #[0 0 0  1]
        # u, _, vh = np.linalg.svd(H[1:3, 1:3])
        # R = u @ vh
        # roll = np.arctan2(R[1,1], R[0,1])
        pitch = np.arctan2(-H[2,1], H[1,1])
        #pitch
        #[c  0 s 0]
        #[0  1 0 0]
        #[-s 0 c 0]
        #[0  0 0 1]
        # u, _, vh = np.linalg.svd(H[0:3, 0:3])
        # R = u @ vh
        # picth = np.arctan2(R[2,2], R[0,2])
        roll = np.arctan2(H[0,2], H[2,2])
        return roll, pitch, yaw

    def resize_by_scale(self, rasterArray, pixel_size, scale):
        rasterArray = resize_img(rasterArray, scale)
        pixel_size = Decimal(pixel_size) / Decimal(scale)
        return rasterArray, pixel_size

    def normalize_images(self, roi, cadr):
        # roi = self.basicLinearTransform(roi, 1.0, 0.5)
        # cadr = self.basicLinearTransform(cadr, 1.5, 0.0)
        sum_roi = (np.sum(roi))/(roi.shape[0]*roi.shape[1])
        sum_cadr = (np.sum(cadr))/(cadr.shape[0]*cadr.shape[1])
        # print(sum_roi, sum_cadr)
        rel = sum_roi/sum_cadr
        if rel <= 1:
            roi = self.basicLinearTransform(roi, 1/rel, 0.0)
        else:
            cadr = self.basicLinearTransform(cadr, rel, 0.0)
        clahe = cv2.createCLAHE(clipLimit=30.0, tileGridSize=(8,8))
        roi = clahe.apply(roi)
        cadr = clahe.apply(cadr)
        return roi, cadr

    def basicLinearTransform(self, img_original, alpha, beta):
        res = cv2.convertScaleAbs(img_original, alpha=alpha, beta=beta)
        return res
    
    def gammaCorrection(self, img_original, gamma):
        lookUpTable = np.empty((1,256), np.uint8)
        for i in range(256):
            lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)

        res = cv2.LUT(img_original, lookUpTable)
        return res
    
