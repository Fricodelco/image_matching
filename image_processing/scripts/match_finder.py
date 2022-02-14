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
    def __init__(self):
        self.optimal_size_x = 400
        self.search_scale = 2
        self.roi_img = None
        self.percent_of_good_value = 2.5

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
        height = cadr.rasterArray.shape[0]* self.search_scale
        x_min = int(pixel_x - height/2)
        x_max = int(pixel_x + height/2)
        y_min = int(pixel_y - width/2)
        y_max = int(pixel_y + width/2)
        if x_min < 0: x_min = 0
        if y_min < 0: y_min = 0
        if x_max > map_.rasterArray.shape[0]: x_max = map_.rasterArray.shape[0]
        if y_max > map_.rasterArray.shape[1]: y_max = map_.rasterArray.shape[1]

        #save to structure
        img = map_.rasterArray[x_min:x_max, y_min:y_max]
        roi_img = roi(img, lat, lon, int(x_min+x_max)/2, int(y_min+y_max)/2, map_.pixel_size)
        return roi_img

    def rescale_for_optimal_sift(self, roi, cadr):
        optimal_scale_for_cadr = self.optimal_size_x/cadr.rasterArray.shape[1]
        cadr.rasterArray = self.resize_img(cadr.rasterArray, optimal_scale_for_cadr)
        cadr.pixel_size = cadr.pixel_size/Decimal(optimal_scale_for_cadr)
        roi.img = self.resize_img(roi.img, optimal_scale_for_cadr)
        roi.pixel_size = roi.pixel_size/Decimal(optimal_scale_for_cadr)
        return roi, cadr
        

    def find_matches(self, img2, img1):
        sift = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
            nOctaveLayers = 4,
            contrastThreshold = 0.04,
            edgeThreshold = 10,
            sigma = 1.6)
        keypoints_1, descriptors_1 = sift.detectAndCompute(img1,None)
        keypoints_2, descriptors_2 = sift.detectAndCompute(img2,None)
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors_1,descriptors_2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append([m])
        img3 = cv2.drawMatchesKnn(img1,keypoints_1,img2,keypoints_2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # img3 = cv2.drawKeypoints(img1, keypoints_2, img2, color = (255, 20, 147))
        cv2.imshow('img', img3)
        cv2.waitKey(0)
        cv2.destroyAllWindows()        
        return keypoints_1, keypoints_2, good
        

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
        x_center, y_center = self.line_intersection((dst[0][0], dst[2][0]), (dst[1][0], dst[3][0]))
        #draw
        img2 = cv2.circle(img2, (int(x_center), int(y_center)), 10, 255, 5)
        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None, **draw_params)
        img = self.rotate_image(img1, (yaw/np.pi)*180)
        cv2.imshow('img2', img3)
        cv2.imshow('img1', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()        
        return x_center, y_center, roll, pitch, yaw, M
    
    def solve_IK(self, x_center, y_center, roll, pitch, yaw, height, roi):
        a = 1

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
    

    def find_center_of_image(self, map_main_points, cadr_main_points):
        lat = (cadr_main_points[0].lat + cadr_main_points[2].lat)/2
        lon = (cadr_main_points[0].lon + cadr_main_points[2].lon)/2
        g_c = GeodeticConvert()
        g_c.initialiseReference(map_main_points[0].lat, map_main_points[0].lon, 0)
        x, y, z = g_c.geodetic2Ned(lat, lon, 0)
        return x, y, z, lat, lon
        

    def resize_by_scale(self, rasterArray, pixel_size, scale):
        rasterArray = self.resize_img(rasterArray, scale)
        pixel_size = Decimal(pixel_size) / Decimal(scale)
        return rasterArray, pixel_size

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
    
    def line_intersection(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y
