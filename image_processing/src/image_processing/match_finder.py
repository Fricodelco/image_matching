#!/usr/bin/env python3  
from pickletools import uint8
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
from time import time
@dataclass
class roi:
    img: np.ndarray
    kp: tuple
    dp: np.ndarray
    pixel_size: None
    pixel_x_main_map: int = 0
    pixel_y_main_map: int = 0
    
    
class match_finder():
    def __init__(self):
        #parameters
        self.optimal_size_x = rospy.get_param("image_size_sift")
        self.points_quality = rospy.get_param("points_quality_sift")
        self.nOctaveLayers = rospy.get_param("nOctaveLayers_sift")
        self.contrastThreshold = rospy.get_param("contrastThreshold_sift")
        self.edgeThreshold = rospy.get_param("edgeThreshold_sift")
        self.sigma = rospy.get_param("sigma_sift")
        self.angle_restriction_homography = rospy.get_param("angle_restriction_homography")
        self.low_scale_restriction_homography = rospy.get_param("low_scale_restriction_homography")
        self.high_scale_restriction_homography = rospy.get_param("high_scale_restriction_homography")
        self.camera_pitch_angle = rospy.get_param("camera_pitch_angle")
        self.cuda_enabled = self.is_cuda_cv()
        # if self.cuda_enabled is True:
            # print("CUDA IS ENABLED")
        # else:
            # print("CUDA IS DISABLED")
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
        width = cadr.img.shape[1]*search_scale/cadr.cadr_scale
        height = cadr.img.shape[0]*search_scale/cadr.cadr_scale
        if width > height:
            height = width
        else:
            width = height
        
        x_min = int(pixel_x - width/2)
        x_max = int(pixel_x + width/2)
        y_min = int(pixel_y - height/2)
        y_max = int(pixel_y + height/2)
        if x_min < 0: x_min = 0
        if y_min < 0: y_min = 0
        if x_max > map_.img.shape[1]: x_max = map_.img.shape[1]
        if y_max > map_.img.shape[0]: y_max = map_.img.shape[0]
        # print(x_min, x_max, y_min, y_max)
        #save to structure
        img = map_.img[y_min:y_max, x_min:x_max]
        roi_img = roi(img = img, kp = None, dp = None,
                pixel_size = map_.pixel_size, pixel_x_main_map = x_min, pixel_y_main_map = y_min)    
        return roi_img

    def roi_from_last_xy(self, map_, x_meter, y_meter, cadr, search_scale, yaw):
        #find the corners
        # if(abs(np.cos(yaw)) < np.cos(np.pi/4)):
        pixel_x = x_meter/float(map_.pixel_size)
        pixel_y = y_meter/float(map_.pixel_size)
        width = cadr.img.shape[1]*search_scale/cadr.cadr_scale
        height = cadr.img.shape[0]*search_scale/cadr.cadr_scale
        if width > height:
            height = width
        else:
            width = height
        # else:
            # width = cadr.img.shape[0]*search_scale
            # height = cadr.img.shape[1]*search_scale
        x_min = int(pixel_x - width/2)
        x_max = int(pixel_x + width/2)
        y_min = int(pixel_y - height/2)
        y_max = int(pixel_y + height/2)
        if x_min < 0: x_min = 0
        if y_min < 0: y_min = 0
        if x_max > map_.img.shape[1]: x_max = map_.img.shape[1]
        if y_max > map_.img.shape[0]: y_max = map_.img.shape[0]
        #save to structure
        img = map_.img[y_min:y_max, x_min:x_max]
        roi_img = roi(img = img, kp = None, dp = None,
                pixel_size = map_.pixel_size, pixel_x_main_map = x_min, pixel_y_main_map = y_min)    
        return roi_img


    def roi_from_map(self, map_, rl_size_x, rl_size_y):
        map_shape_x = map_.img.shape[1]
        map_shape_y = map_.img.shape[0]
        div_x = map_shape_x/rl_size_x
        div_y = map_shape_y/rl_size_y
        drob_x, cel_x = math.modf(div_x)
        drob_y, cel_y = math.modf(div_y)
        if cel_x != 0:
            step_x = rl_size_x + ((map_shape_x - rl_size_x*cel_x)/cel_x)
        else:
            step_x = map_shape_x
        if cel_y != 0:    
            step_y = rl_size_y + ((map_shape_y - rl_size_y*cel_x)/cel_y)
        else:
            step_y = map_shape_y
        borders = []
        for x in range(0, int(map_shape_x), int(step_x)):
            x_min = int(x)
            x_max = int(x+step_x+step_x*0.1)
            if x_max > map_shape_x:
                 x_max = map_shape_x
            if x_max - x_min < step_x/5:
                break 
            for y in range(0, int(map_shape_y), int(step_y)):
                y_min = int(y)
                y_max = int(y+step_y+step_y*0.1)
                if y_max > map_shape_y:
                    y_max = map_shape_y
                if y_max - y_min < step_y/5:
                    break
                borders.append([x_min, x_max, y_min, y_max])
        # print(borders)
        return borders
    
    def create_roi_from_border(self, map_, border):
        x_min = border[0]
        x_max = border[1]
        y_min = border[2]
        y_max = border[3]
        img = map_.img[x_min:x_max, y_min:y_max]
        roi_ = roi(img = img, kp = None, dp = None,
                pixel_size = map_.pixel_size, pixel_x_main_map = x_min, pixel_y_main_map = y_min)    
        return roi_

    def roi_full_map(self, map_):
        # shape = map_.img.shape
        # img = map_.img[:int(0.7*shape[0]),:]
        img = map_.img
        roi_img = roi(img, 0, 0, map_.pixel_size)
        return roi_img

    
    def rescale_for_optimal_sift(self, roi, cadr):
        optimal_scale_for_cadr = cadr.cadr_scale
        roi.img = resize_img(roi.img, optimal_scale_for_cadr)
        roi.pixel_size = roi.pixel_size/Decimal(optimal_scale_for_cadr)
        return roi, cadr
    
    def rescale_cadr(self, cadr, pixel_size):
        optimal_scale_for_cadr = self.optimal_size_x/cadr.shape[1]
        cadr = resize_img(cadr, optimal_scale_for_cadr)
        pixel_size = pixel_size/Decimal(optimal_scale_for_cadr)
        return cadr, optimal_scale_for_cadr, pixel_size

    def find_matches(self, roi, cadr): #roi, cadr
        keypoints_1 = cadr.kp
        descriptors_1 = cadr.dp
        matches = []
        if self.cuda_enabled == False:
            # bf = cv2.BFMatcher(cv2.NORM_L2)
            bf = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
            matches = bf.knnMatch(descriptors_1,roi.dp,k=2)
        else:
            matcherGPU = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_L2)
            gpu_stream = cv2.cuda_Stream()
            desc_gpu_1 = cv2.cuda_GpuMat(descriptors_1)
            desc_gpu_2 = cv2.cuda_GpuMat(roi.dp)
            matches = matcherGPU.knnMatch(desc_gpu_1, desc_gpu_2, k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < self.points_quality*n.distance:
                good.append([m])
        # img3 = cadr.img
        img3 = cv2.drawMatchesKnn(cadr.img, keypoints_1, roi.img, roi.kp, good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)     
        return good, img3

    def find_kp_dp(self, img):
        # clahe = cv2.createCLAHE(clipLimit=30.0, tileGridSize=(8,8))
        clahe = cv2.createCLAHE(clipLimit=80.0, tileGridSize=(16,16))
        img = clahe.apply(img)
        # img = cv2.GaussianBlur(img,(3,3),0)
        surf = cv2.xfeatures2d.SIFT_create(nfeatures = 0,
            nOctaveLayers = self.nOctaveLayers,
            contrastThreshold = self.contrastThreshold,
            edgeThreshold = self.edgeThreshold,
            sigma = self.sigma)
        

        # surf = cv2.xfeatures2d.SURF_create(hessianThreshold = 10,
        #                             nOctaves = 3,
        #                             nOctaveLayers = 1,
        #                             extended = False,
        #                             upright = False)
        

        keypoints_1, descriptors_1 = surf.detectAndCompute(img, None)
        return keypoints_1, descriptors_1, img

    def find_keypoints_transform(self, matches, roi, cadr, img_for_pub):
        try:
            img2 = roi.img
        except:
            img2 = roi.img
        img1 = cadr.img
        kp1 = cadr.kp
        kp2 = roi.kp
        src_pts = np.float32([ kp1[m[0].queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m[0].trainIdx].pt for m in matches]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.USAC_MAGSAC, 0.1)
        matchesMask = [[0,0] for i in range(len(matches))]
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        pts_vis = [[0,0],[0,h-1],[w-1,h-1],[w-1,0]]
        dst = cv2.perspectiveTransform(pts,M)
        isConv, answer = isConvex(dst, img1.shape, self.angle_restriction_homography,
                self.low_scale_restriction_homography, self.high_scale_restriction_homography) 
        x_center, y_center = line_intersection((dst[0][0], dst[2][0]), (dst[1][0], dst[3][0]))
        #draw
        if img_for_pub is not None:
            dst[0][0][0]+=img1.shape[1]
            dst[1][0][0]+=img1.shape[1]
            dst[2][0][0]+=img1.shape[1]
            dst[3][0][0]+=img1.shape[1]
            x_center_img, y_center_img = line_intersection((dst[0][0], dst[2][0]), (dst[1][0], dst[3][0]))
            img_for_pub = cv2.circle(img_for_pub, (int(x_center_img), int(y_center_img)), 10, 255, 5)
            img_for_pub = cv2.polylines(img_for_pub,[np.int32(dst)],True,255,3, cv2.LINE_AA)    
        if isConv is True:
            roll, pitch, yaw = self.get_angles_from_homography(M) 
            return x_center, y_center, roll, pitch, yaw, M, img_for_pub, answer
        else:
            return None, None, None, None, None, None, img_for_pub, answer
    
    def solve_IK(self, x_center, y_center, height, roll, pitch, yaw, roi, map_):
        delta_pitch_pixels = -1*height*np.sin(pitch+self.camera_pitch_angle)/float(roi.pixel_size)
        delta_roll_pixels = height*np.sin(roll)/float(roi.pixel_size)

        x_center = x_center + delta_pitch_pixels*np.sin(yaw) + delta_roll_pixels*np.cos(yaw)
        y_center = y_center - delta_pitch_pixels*np.cos(yaw) + delta_roll_pixels*np.sin(yaw)
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
        
     def calculate_hog(self, image):
        winSize = (64,64)
        blockSize = (16,16)
        blockStride = (8,8)
        cellSize = (8,8)
        nbins = 9
        derivAperture = 1
        winSigma = 4.
        histogramNormType = 0
        L2HysThreshold = 2.0000000000000001e-01
        gammaCorrection = 0
        nlevels = 64
        hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,
                                histogramNormType,L2HysThreshold,gammaCorrection,nlevels)
        #compute(img[, winStride[, padding[, locations]]]) -> descriptors
        winStride = (8,8)
        padding = (8,8)
        locations = ((10,20),)
        hist = hog.compute(image,winStride,padding,locations)
        
    def get_angles_from_homography(self, H):
        #eject the yaw transform
        #[c -s 0 0]
        #[s c  0 0]
        #[0 0  1 0]
        #[0 0  0 1]
        u, _, vh = np.linalg.svd(H[0:2, 0:2])
        R = u @ vh
        yaw = np.arctan2(R[1,0], R[0,0])
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

    def resize_by_scale(self, img, pixel_size, scale):
        img = resize_img(img, scale)
        pixel_size = Decimal(pixel_size) / Decimal(scale)
        return img, pixel_size

    def normalize_images(self, roi, cadr):
        # roi = self.basicLinearTransform(roi, 1.0, 0.5)
        # cadr = self.basicLinearTransform(cadr, 1.5, 0.0)
        sum_roi = (np.sum(roi))/(roi.shape[0]*roi.shape[1])
        sum_cadr = (np.sum(cadr))/(cadr.shape[0]*cadr.shape[1])
        # print(sum_roi, sum_cadr)
        # rel = sum_roi/sum_cadr
        # if rel <= 1:
            # roi = self.basicLinearTransform(roi, 1/rel, 0.0)
        # else:
            # cadr = self.basicLinearTransform(cadr, rel, 0.0)
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