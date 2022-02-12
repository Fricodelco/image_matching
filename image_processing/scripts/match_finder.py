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
import matplotlib.pyplot as plt
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
        self.percent_of_good_value = 3.5

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
        img2 = self.roi_img.img # trainImage
        img1 = self.cadr.rasterArray # queryImage
        # img1 = self.rotate_image(img1, self.aproximate_angle)
        sift = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
            nOctaveLayers = 4,
            contrastThreshold = 0.04,
            edgeThreshold = 10,
            sigma = 1.6 )
        keypoints_1, descriptors_1 = sift.detectAndCompute(img1,None)
        
        keypoints_2, descriptors_2 = sift.detectAndCompute(img2,None)

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors_1,descriptors_2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.5*n.distance:
                good.append([m])
        percent_of_good = (len(good)/len(keypoints_1))*100
        print(percent_of_good)
        if percent_of_good > self.percent_of_good_value:
            self.find_keypoints_transform(keypoints_1, keypoints_2, good, img1, img2)
        # img3 = cv2.drawMatchesKnn(img1,keypoints_1,img2,keypoints_2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # img3 = cv2.drawKeypoints(img2, keypoints_2, img2, color = (255, 20, 147))
        # cv2.imshow('img', img3)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()        
    
    def find_keypoints_transform(self, kp1, kp2, matches, img1, img2):
        src_pts = np.float32([ kp1[m[0].queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m[0].trainIdx].pt for m in matches]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.USAC_MAGSAC,5.0)
        matchesMask = [[0,0] for i in range(len(matches))]
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        pts_vis = [[0,0],[0,h-1],[w-1,h-1],[w-1,0]]
        dst = cv2.perspectiveTransform(pts,M)
        roll, picth, yaw = self.get_angles_from_homography(M)
        print(roll, picth, yaw)
        img = cv2.warpPerspective(img1, M, (img1.shape[0], img1.shape[1]), cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        
        # print(np.linalg.inv(M))
        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None, **draw_params)
        img = self.rotate_image(img1, yaw)
        cv2.imshow('img2', img3)
        cv2.imshow('img1', img)
        # cv2.imshow('dst', warp_dst)
        cv2.waitKey(0)
        cv2.destroyAllWindows()        
       
    def get_angles_from_homography(self, H):
        #eject the yaw transform
        #[c -s 0 0]
        #[s c  0 0]
        #[0 0  1 0]
        #[0 0  0 1]
        u, _, vh = np.linalg.svd(H[0:2, 0:2])
        R = u @ vh
        yaw = np.arctan2(R[1,0], R[0,0])
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
    

    def rotate_image(self, mat, angle):
        height, width = mat.shape[:2] # image shape has 3 dimensions
        image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

        rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

        # rotation calculates the cos and sin, taking absolutes of those.
        abs_cos = abs(rotation_mat[0,0]) 
        abs_sin = abs(rotation_mat[0,1])

        # find the new width and height bounds
        bound_w = int(height * abs_sin + width * abs_cos)
        bound_h = int(height * abs_cos + width * abs_sin)

        # subtract old image center (bringing image back to origo) and adding the new image center coordinates
        rotation_mat[0, 2] += bound_w/2 - image_center[0]
        rotation_mat[1, 2] += bound_h/2 - image_center[1]

        # rotate image with the new bounds and translated rotation matrix
        rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
        return rotated_mat
