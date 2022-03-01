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
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import copy
from utils import resize_img, draw_circle_on_map_by_coord_and_angles
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3

class PositionFinder:
    def __init__(self):
        #load main map
        # self.main_map = image_processing('500m/17.tif', 0)
        self.main_map = image_processing('500m/Anapa_g.tif', 0)
        # self.main_map = image_processing('500m/b-g.jpg', 0)
        self.map_pixel_size = self.main_map.find_pixel_size()
        self.first_cadr = True
        self.height = 500
        #camera params
        self.poi = 84/180.0*np.pi
        self.f = 7.7
        #create matcher object
        self.matcher = match_finder()
        #create global variables
        self.last_x = 0
        self.last_y = 0
        self.last_roi = None
        #ros infrustructure
        self.sub_photo = rospy.Subscriber("photo",Image, self.photo_cb)
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/find_transform', Image, queue_size=1)
        self.pub_keypoints_image = rospy.Publisher('/keypoints_matches', Image, queue_size=1)
        self.pub_latlon = rospy.Publisher('/coordinates_by_img', NavSatFix, queue_size=1)
        self.pub_estimated_odom = rospy.Publisher('/odom_by_img', Odometry, queue_size=1)
        print("Position Finder ready")

    def photo_cb(self, data):
        time1 = time()
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
        lat, lon, roll, pitch, yaw = self.find_first_pose_without_roi(cadr)
        print(time()- time1)
        
    def find_first_pose_without_roi(self, cadr):
        if self.first_cadr is True:
            roi = self.matcher.roi_full_map(self.main_map)
            # self.first_cadr = False
        else:
            roi = self.matcher.roi_from_last_xy(self.main_map, float(self.last_x), float(self.last_y), cadr, 3)
            
        percent_of_good, kp_1, kp_2, good, img_for_pub, cadr_rescaled = self.find_matches(roi, cadr)
        # self.pub_keypoints_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
        x_center, y_center, roll, pitch, yaw, M, img = self.matcher.find_keypoints_transform(kp_1, kp_2, good, roi.img, cadr_rescaled.rasterArray)
        if x_center is not None:
            # x_center =  pixel_x + self.height*np.cos(pitch)
            x_center, y_center = self.add_angle_translation(roll, pitch, yaw, roi, x_center, y_center)
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))
            lat, lon, self.last_x, self.last_y = self.matcher.solve_IK(x_center, y_center, roll, pitch, yaw, roi, self.main_map)
            self.first_cadr = False
            img = resize_img(self.main_map.rasterArray, 0.2)
            pixel_size = self.main_map.pixel_size/Decimal(0.2)
            img = draw_circle_on_map_by_coord_and_angles(img,
                                        (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                        (lat, lon), pixel_size, roll, pitch, yaw)
            
            self.pub_keypoints_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))    
            self.generate_and_send_pose(lat, lon, roll, pitch, yaw)
            return lat, lon, roll, pitch, yaw
        return 0,0,0,0,0
    
    def generate_and_send_pose(self, lat, lon, roll, pitch, yaw):
        latlon_msg = NavSatFix()
        latlon_msg.latitude = lat
        latlon_msg.longitude = lon
        latlon_msg.altitude = self.height
        latlon_msg.header.stamp = rospy.Time.now()
        self.pub_latlon.publish(latlon_msg)
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # odom_msg.pose.pose = Pose(Point(0,0,0.), Quaternion(*odom_quat))
        odom_msg.pose.pose.position.x = roll
        odom_msg.pose.pose.position.y = pitch
        odom_msg.pose.pose.position.z = yaw
        self.pub_estimated_odom.publish(odom_msg)

    def add_angle_translation(self, roll, pitch, yaw, roi, x_center, y_center):
        delta_pitch_pixels = self.height*np.sin(pitch)/float(roi.pixel_size)
        delta_roll_pixels = self.height*np.cos(roll)/float(roi.pixel_size)
        x_center = x_center - delta_pitch_pixels*np.cos(yaw)
        y_center = y_center - delta_pitch_pixels*np.sin(yaw)
        x_center = x_center + delta_roll_pixels*np.cos(yaw)
        y_center = y_center + delta_roll_pixels*np.sin(yaw)
        return x_center, y_center    

    def find_matches(self, roi, cadr):
        cadr_rescaled = copy.deepcopy(cadr)
        roi, cadr_rescaled = self.matcher.rescale_for_optimal_sift(roi, cadr_rescaled)
        roi.img = cv2.equalizeHist(roi.img)
        # roi.img = cv2.GaussianBlur(roi.img,(7,7),0)
        cadr_rescaled.rasterArray = cv2.equalizeHist(cadr_rescaled.rasterArray)
        # cadr_rescaled.rasterArray = cv2.GaussianBlur(cadr_rescaled.rasterArray,(3,3),0)
        kp_1, kp_2, good, img_for_pub = self.matcher.find_matches(roi.img, cadr_rescaled.rasterArray)
        percent_of_good = (len(good)/len(kp_1))*100
        # self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
        return percent_of_good, kp_1, kp_2, good, img_for_pub, cadr_rescaled

        # print("here---------------------")
        # rolling_window_size_x = cadr.rasterArray.shape[0]*3.0
        # rolling_window_size_y = cadr.rasterArray.shape[1]*3.0
        # roi_borders = self.matcher.roi_from_map(self.main_map, rolling_window_size_x, rolling_window_size_y)
        # percents = []
        # for border in roi_borders:
        #     roi = self.matcher.create_roi_from_border(self.main_map, border)
        #     percent_of_good,_,_,_,_,_ = self.find_matches(roi, cadr)
        #     percents.append(percent_of_good)
        # max_value = max(percents)
        # max_index = percents.index(max_value)
        # print(max_value, max_index)
        # if max_value < 0.1:
        #     return 0,0
        # else:
        #     roi = self.matcher.create_roi_from_border(self.main_map, roi_borders[max_index])
        #     percent_of_good, kp_1, kp_2, good, img_for_pub, cadr_rescaled = self.find_matches(roi, cadr)
        #     x_center, y_center, roll, pitch, yaw, M, img = self.matcher.find_keypoints_transform(kp_1, kp_2, good, roi.img, cadr_rescaled.rasterArray)
        #     # lat, lon = self.matcher.solve_IK(x_center, y_center, roll, pitch, yaw, roi, self.main_map)
        #     # img = resize_img(self.main_map.rasterArray, 0.2)
        #     # pixel_size = self.main_map.pixel_size/Decimal(0.2)
        #     # img = draw_circle_on_map_by_coord(img,
        #                                 # (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
        #                                 # (lat, lon), pixel_size)
        #     self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))

        #     self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
            # print(percent_of_good)

if __name__ == '__main__':
    rospy.init_node('position_finder')
    photo_publisher = PositionFinder()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    




