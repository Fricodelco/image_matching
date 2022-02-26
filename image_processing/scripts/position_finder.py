#!/usr/bin/env python3  
from numpy import reshape
import rospy
import cv2
import numpy as np
import rospkg
import gdal
from image_processing import image_processing
from decimal import Decimal
from match_finder import match_finder
from time import time
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import copy
from utils import resize_img, draw_circle_on_map_by_coord

class PositionFinder:
    def __init__(self):
        #load main map
        self.main_map = image_processing('500m/17.tif', 0)
        # self.main_map = image_processing('500m/Anapa_n.TIF', 0)
        self.map_pixel_size = self.main_map.find_pixel_size()
        self.first_cadr = True
        self.height = 500
        #camera params
        self.poi = 84/180.0*np.pi
        self.f = 7.7
        #create matcher object
        self.matcher = match_finder()
        #ros infrustructure
        self.sub_photo = rospy.Subscriber("photo",Image, self.photo_cb)
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/keypoints_matches', Image, queue_size=1)
        print("Position Finder ready")

    def photo_cb(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        cadr = image_processing(img = image)
       
        #find cadr pixel size
        x = Decimal(np.tanh(self.poi/2)*2*self.height)
        cadr.pixel_size = x/Decimal(image.shape[1])
        
        #resize by pixel size
        scale, map_pixel_bigger = self.matcher.find_scale(self.map_pixel_size, cadr.pixel_size)
        if map_pixel_bigger is True:
            cadr.rasterArray, cadr.pixel_size = self.matcher.resize_by_scale(
                                    cadr.rasterArray, cadr.pixel_size, scale)
        else:
            #need copy of full size map for future
            self.main_map.rasterArray, self.main_map.pixel_size = self.matcher.resize_by_scale(
                                self.main_map.rasterArray, self.main_map.pixel_size, scale)
            self.map_pixel_size = self.main_map.pixel_size
        #find match
        if self.first_cadr == True:
            lat, lon = self.find_first_pose_without_roi(cadr)
            # self.first_cadr = False
        else:
            a = 1

    def find_first_pose_without_roi(self, cadr):
        print("here---------------------")
        rolling_window_size_x = cadr.rasterArray.shape[0]*1.5
        rolling_window_size_y = cadr.rasterArray.shape[1]*1.5
        roi_borders = self.matcher.roi_from_map(self.main_map, rolling_window_size_x, rolling_window_size_y)
        percents = []
        for border in roi_borders:
            roi = self.matcher.create_roi_from_border(self.main_map, border)
            percent_of_good,_,_,_,_, = self.find_matches(roi, cadr)
            percents.append(percent_of_good)
        max_value = max(percents)
        max_index = percents.index(max_value)
        print(max_value, max_index)
        if max_value < 0.1:
            return 0,0
        else:
            roi = self.matcher.create_roi_from_border(self.main_map, roi_borders[max_index])
            percent_of_good, kp_1, kp_2, good, img_for_pub = self.find_matches(roi, cadr)
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
            print(percent_of_good)

        # roi = self.matcher.roi_full_map(self.main_map)
        # lat, lon = self.find_match(roi, cadr)
        
        # print(lat, lon)
        # if lat != 0.0 and lon != 0.0:
            
            # print("excelent!")
        # return lat,lon
        return 0,0

    def find_matches(self, roi, cadr):
        cadr_rescaled = copy.deepcopy(cadr)
        roi, cadr_rescaled = self.matcher.rescale_for_optimal_sift(roi, cadr_rescaled)
        # roi.img = cv2.equalizeHist(roi.img)
        # cadr_rescaled.rasterArray = cv2.equalizeHist(cadr_rescaled.rasterArray)
        kp_1, kp_2, good, img_for_pub = self.matcher.find_matches(roi.img, cadr_rescaled.rasterArray)
        percent_of_good = (len(good)/len(kp_1))*100
        # self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
        return percent_of_good, kp_1, kp_2, good, img_for_pub

    def find_match(self, roi, cadr):
        cadr_rescaled = copy.deepcopy(cadr)
        roi, cadr_rescaled = self.matcher.rescale_for_optimal_sift(roi, cadr_rescaled)
        roi.img = cv2.equalizeHist(roi.img)
        cadr_rescaled.rasterArray = cv2.equalizeHist(cadr_rescaled.rasterArray)
        kp_1, kp_2, good, img_for_pub = self.matcher.find_matches(roi.img, cadr_rescaled.rasterArray)
        percent_of_good = (len(good)/len(kp_1))*100
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
        # print(len(good))
        print(percent_of_good)
        # if len(good) > 4:
        # if percent_of_good > self.matcher.self.percent_of_good_value:
        #     x_center, y_center, roll, pitch, yaw, M = self.matcher.find_keypoints_transform(kp_1, kp_2, good, roi.img, cadr_rescaled.rasterArray)
        #     lat, lon = self.matcher.solve_IK(x_center, y_center, roll, pitch, yaw, roi, main_map)
        #     img = resize_img(main_map.rasterArray, 0.2)
        #     pixel_size = main_map.pixel_size/Decimal(0.2)
        #     img = draw_circle_on_map_by_coord(img,
        #                             # (main_map.main_points[0].lat, main_map.main_points[0].lon),
        #                             (lat, lon), pixel_size)
        #     self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

            # cv2.imshow('img', img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            # return lat, lon    
        # else:
        return 0.0 ,0.0



if __name__ == '__main__':
    rospy.init_node('position_finder')
    photo_publisher = PositionFinder()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    




