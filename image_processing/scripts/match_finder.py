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
from utils import resize_img, rotate_image, line_intersection
import math
@dataclass
class roi:
    img: np.ndarray
    pixel_x_main_map: int = 0
    pixel_y_main_map: int = 0
    pixel_size: float = 0.0


class match_finder():
    def __init__(self):
        self.optimal_size_x = 500
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

    def find_map_roi_by_coordinates(self, map_, cadr):
        #find the center of cadr coordinates at map
        x, y, z, lat, lon = self.find_center_of_image(map_.main_points, cadr.main_points)
        x = -x
        if x < 0 or y < 0:
            print("no match")
            return 0
        #find the center pixel
        pixel_x = int(Decimal(x) / map_.pixel_size)
        pixel_y = int(Decimal(y) / map_.pixel_size)
        #find the corners
        width = cadr.rasterArray.shape[1]*self.search_scale
        height = cadr.rasterArray.shape[0]*self.search_scale
        x_min = int(pixel_x - height/2)
        x_max = int(pixel_x + height/2)
        y_min = int(pixel_y - width/2)
        y_max = int(pixel_y + width/2)
        if x_min < 0: x_min = 0
        if y_min < 0: y_min = 0
        if x_max > map_.rasterArray.shape[0]: x_max = map_.rasterArray.shape[0]
        if y_max > map_.rasterArray.shape[1]: y_max = map_.rasterArray.shape[1]
        print(x_min, x_max, y_min, y_max)
        #save to structure
        img = map_.rasterArray[x_min:x_max, y_min:y_max]
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
        img = map_.rasterArray
        roi_img = roi(img, 0, 0, map_.pixel_size)
        return roi_img


    def rescale_for_optimal_sift(self, roi, cadr):
        optimal_scale_for_cadr = self.optimal_size_x/cadr.rasterArray.shape[1]
        cadr.rasterArray = resize_img(cadr.rasterArray, optimal_scale_for_cadr)
        cadr.pixel_size = cadr.pixel_size/Decimal(optimal_scale_for_cadr)
        roi.img = resize_img(roi.img, optimal_scale_for_cadr)
        roi.pixel_size = roi.pixel_size/Decimal(optimal_scale_for_cadr)
        return roi, cadr
        

    def find_matches(self, img2, img1):
        # sift = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
        #     nOctaveLayers = 4,
        #     contrastThreshold = 0.04,
        #     edgeThreshold = 10,
        #     sigma = 1.6)
        # # sift = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
        #     nOctaveLayers = 4,
        #     contrastThreshold = 0.04,
        #     edgeThreshold = 30,
        #     sigma = 1.6)
        # surf = cv2.xfeatures2d.SURF_create(hessianThreshold = 400,
        #                             nOctaves = 4,
        #                             nOctaveLayers = 4,
        #                             extended = True,
        #                             upright = True)

        keypoints_1, descriptors_1 = surf.detectAndCompute(img1,None)
        keypoints_2, descriptors_2 = surf.detectAndCompute(img2,None)

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors_1,descriptors_2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.8*n.distance:
                good.append([m])
        img3 = cv2.drawMatchesKnn(img1,keypoints_1,img2,keypoints_2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # img3 = cv2.drawKeypoints(img1, keypoints_2, img2, color = (255, 20, 147))
        # cv2.imshow('img', img3)
        # cv2.waitKey(0) 
        # cv2.destroyAllWindows()        
        return keypoints_1, keypoints_2, good, img3
        

    def find_keypoints_transform(self, kp1, kp2, matches, img2, img1):
        src_pts = np.float32([ kp1[m[0].queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m[0].trainIdx].pt for m in matches]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.USAC_MAGSAC,5.0)
        matchesMask = [[0,0] for i in range(len(matches))]
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        pts_vis = [[0,0],[0,h-1],[w-1,h-1],[w-1,0]]
        dst = cv2.perspectiveTransform(pts,M)
        roll, pitch, yaw = self.get_angles_from_homography(M)
        x_center, y_center = line_intersection((dst[0][0], dst[2][0]), (dst[1][0], dst[3][0]))
        #draw
        img2 = cv2.circle(img2, (int(x_center), int(y_center)), 10, 255, 5)
        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
        # img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None, **draw_params)
        # img = rotate_image(img1, (yaw/np.pi)*180)
        # cv2.imshow('img2', img2)
        # cv2.imshow('img1', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()        
        return x_center, y_center, roll, pitch, yaw, M
    
    def solve_IK(self, x_center, y_center, roll, pitch, yaw, roi, map_):
        scale_roi_to_map = roi.pixel_size/map_.pixel_size
        roi_shape_x = roi.img.shape[1] * float(scale_roi_to_map)
        roi_shape_y = roi.img.shape[0] * float(scale_roi_to_map)
        scaled_x_center = Decimal(Decimal(float(x_center))*scale_roi_to_map)
        scaled_y_center = Decimal(Decimal(float(y_center))*scale_roi_to_map)
        x = Decimal(roi.pixel_x_main_map) + Decimal(scaled_y_center)
        y = Decimal(roi.pixel_y_main_map) + Decimal(scaled_x_center)
        x_meter = x*map_.pixel_size
        y_meter = y*map_.pixel_size         
        g_c = GeodeticConvert()
        g_c.initialiseReference(map_.main_points[0].lat, map_.main_points[0].lon, 0)
        # # print(x_center*float(roi.pixel_size), y_center*float(roi.pixel_size))
        print(x_meter, y_meter)
        lat, lon, _ = g_c.ned2Geodetic(float(x_meter), float(y_meter), 0)
        return lat, lon
        

    def get_angles_from_homography(self, H):
        #eject the yaw transform
        #[c -s 0 0]
        #[s c  0 0]
        #[0 0  1 0]
        #[0 0  0 1]
        u, _, vh = np.linalg.svd(H[0:2, 0:2])
        R = u @ vh
        yaw = np.arctan2(-R[1,0], R[0,0])
        #roll
        #[1 0 0  0]
        #[0 c -s 0]
        #[0 s c  0]
        #[0 0 0  1]
        u, _, vh = np.linalg.svd(H[1:3, 1:3])
        R = u @ vh
        roll = np.arctan2(R[1,1], R[0,1])
        #pitch
        #[c  0 s 0]
        #[0  1 0 0]
        #[-s 0 c 0]
        #[0  0 0 1]
        u, _, vh = np.linalg.svd(H[0:3, 0:3])
        R = u @ vh
        picth = np.arctan2(R[2,2], R[0,2])
        return roll, picth, yaw

    def show_cadr_on_map(self):
        g_c = GeodeticConvert()
        g_c.initialiseReference(self.map.main_points[0].lat, self.map.main_points[0].lon, 0)
        img = self.map.rasterArray
        img = resize_img(img, 0.1)
        i = 0
        for point in self.cadr.main_points:
            x, y, _ = g_c.geodetic2Ned(point.lat, point.lon, 0)
            x = -x
            pixel_x = int(Decimal(0.1*x) / self.map.pixel_size)
            pixel_y = int(Decimal(0.1*y) / self.map.pixel_size)
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
    

    def find_center_of_image(self, map_main_points, cadr_main_points):
        lat = (cadr_main_points[0].lat + cadr_main_points[2].lat)/2
        lon = (cadr_main_points[0].lon + cadr_main_points[2].lon)/2
        g_c = GeodeticConvert()
        g_c.initialiseReference(map_main_points[0].lat, map_main_points[0].lon, 0)
        x, y, z = g_c.geodetic2Ned(lat, lon, 0)
        return x, y, z, lat, lon
        

    def resize_by_scale(self, rasterArray, pixel_size, scale):
        rasterArray = resize_img(rasterArray, scale)
        pixel_size = Decimal(pixel_size) / Decimal(scale)
        return rasterArray, pixel_size

