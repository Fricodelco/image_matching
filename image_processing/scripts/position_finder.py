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
from sensor_msgs.msg import Image, Imu, CompressedImage, NavSatFix
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import copy
from utils import resize_img, draw_circle_on_map_by_coord_and_angles
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from geodetic_conv import GeodeticConvert
import yaml

class PositionFinder:
    def __init__(self):
        #load main map
        self.config = self.load_params()
        # self.main_map = image_processing('05_03_2022/2020_03_06_kor.TIF', 0)
        # self.main_map = image_processing('500m/Anapa_g.tif', 0)
        self.main_map = image_processing('600m/Anapa2_g.tif', 0)
        self.map_pixel_size = self.main_map.find_pixel_size()
        self.first_cadr = True
        self.height = 150
        #camera params
        self.poi = 84/180.0*np.pi
        self.f = 7.7
        #create matcher object
        self.matcher = match_finder(self.config)
        #create global variables
        self.last_roi = None
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.lat_gps = 0.0
        self.lon_gps = 0.0
        self.last_yaw = 0.0
        self.old_keypoints = None
        self.old_descriptors = None
        self.old_cadr = None
        self.time_between_cadrs = time()
        self.x_meter = 0
        self.y_meter = 0
        self.between_iter = 0
        self.north_filtered = np.zeros(5)
        self.east_filtered = np.zeros(5)
        self.low_pass_time = time()
        #load params
        self.search_scale_for_roi_by_gps = self.config["search_scale_for_roi_by_gps"]
        self.search_scale_for_roi_by_detection = self.config["search_scale_for_roi_by_detection"]
        self.count_of_pictures_for_odometry = self.config["count_of_pictures_for_odometry"]
        self.low_pass_speed = self.config["low_pass_speed"]
        self.low_pass_coordinates = self.config["low_pass_coordinates"]
        self.use_imu = self.config["use_imu"]
        self.use_gps = self.config["use_gps"]
        self.publish_roi_img = self.config["publish_roi_img"]
        self.publish_keypoints_matches_img = self.config["publish_keypoints_matches_img"]
        self.publish_between_img = self.config["publish_between_img"]
        self.publish_calculated_pose_img = self.config["publish_calculated_pose_img"]
        self.publish_tf_img = self.config["publish_tf_img"]
        #ros infrustructure
        self.sub_photo = rospy.Subscriber("photo",Image, self.photo_cb)
        if self.use_imu is True:
            self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        self.sub_gps = rospy.Subscriber("gps", NavSatFix, self.gps_cb)
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/find_transform', Image, queue_size=1)
        self.pub_roi_image = rospy.Publisher('/roi', Image, queue_size=1)
        self.pub_keypoints_image = rospy.Publisher('/keypoints_matches', Image, queue_size=1)
        self.pub_pose_image = rospy.Publisher('/calculated_pose', Image, queue_size=1)        
        self.pub_between_image = rospy.Publisher('/between_image', Image, queue_size=1)
        self.pub_latlon = rospy.Publisher('/coordinates_by_img', NavSatFix, queue_size=1)
        self.pub_estimated_odom = rospy.Publisher('/odom_by_img', Odometry, queue_size=1)
        print("Position Finder ready")

    def photo_cb(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cadr = image_processing(img = image)
        cadr.find_pixel_size_by_height(self.height, self.poi)
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
        self.find_first_pose_without_roi(cadr)
        
        
    def find_first_pose_without_roi(self, cadr):
        #check if the cadr is first
        if self.first_cadr is True:
            # roi = self.matcher.roi_full_map(self.main_map)
            roi = self.matcher.find_map_roi_by_coordinates(self.main_map, cadr, self.lat_gps, self.lon_gps, self.search_scale_for_roi_by_gps)
        else:
            roi = self.matcher.roi_from_last_xy(self.main_map, float(self.x_meter), float(self.y_meter), cadr, self.search_scale_for_roi_by_detection, self.last_yaw)
        
        if self.publish_roi_img is True:
            self.pub_roi_image.publish(self.bridge.cv2_to_imgmsg(roi.img, "8UC1"))
        
        percent_of_good, kp_1, kp_2, good, img_for_pub, cadr_rescaled, descriptors_1, descriptors_2 = self.find_matches(roi, cadr)
        #compare cadrs
        self.between_iter+=1
        north_speed = 0
        east_speed = 0
        yaw_speed = 0
        speed_limit = False
        if self.between_iter > self.count_of_pictures_for_odometry:
            try:
                north_speed, east_speed, speed_limit, yaw_speed = self.compare_cadrs(kp_1, self.old_keypoints, descriptors_1, self.old_descriptors, cadr_rescaled, self.old_cadr)
            except:
                north_speed = 0
                east_speed = 0
            self.old_cadr = cadr_rescaled
            self.old_keypoints = kp_1
            self.old_descriptors = descriptors_1
            self.between_iter = 0
        
        if self.publish_keypoints_matches_img is True:
            self.pub_keypoints_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
        
        if(len(good)>4):
            try:
                x_center, y_center, roll, pitch, yaw, M, img_tf = self.matcher.find_keypoints_transform(kp_1, kp_2, good, roi.img, cadr_rescaled.rasterArray)
            except Exception as e: x_center = None
        else:
            x_center = None
        #show coordinates
        img = resize_img(self.main_map.rasterArray, 0.2)
        pixel_size = self.main_map.pixel_size/Decimal(0.2)
        img = draw_circle_on_map_by_coord_and_angles(img,
                                    (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                    (self.lat_gps, self.lon_gps), pixel_size, (self.imu_yaw), (0,0,255))
        g_c = GeodeticConvert()
        g_c.initialiseReference(self.main_map.main_points[0].lat, self.main_map.main_points[0].lon, 0)
        lat_mezh, lon_mezh, _ = g_c.ned2Geodetic(north=float(-self.y_meter), east=float(self.x_meter), down=0)

        img = draw_circle_on_map_by_coord_and_angles(img,
                                    (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                    (lat_mezh, lon_mezh), pixel_size, (self.last_yaw), (255,0,255))
            
        if x_center is not None and speed_limit is False:
            if self.publish_tf_img is True:
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_tf, "8UC1"))
            
            lat_zero, lon_zero,_ ,_ , x_meter, y_meter = self.matcher.solve_IK(x_center, y_center, self.height, 0, 0, yaw, roi, self.main_map)
            lat, lon, _, _, x_inc, y_inc = self.matcher.solve_IK(x_center, y_center, self.height,
                                        self.imu_roll, self.imu_pitch, yaw, roi, self.main_map)
            
            self.last_yaw = yaw
            
            #check if first cadr
            if self.first_cadr == True:
                self.first_cadr = False
                self.x_meter = float(x_inc)
                self.y_meter = float(y_inc)
            #low pass check for coordinates
            if self.low_pass_pose(float(x_inc), float(y_inc)) is True:
                self.x_meter = float(x_inc)
                self.y_meter = float(y_inc)
                img = draw_circle_on_map_by_coord_and_angles(img,
                                        (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                        (lat, lon), pixel_size, (yaw), (255,0,0))
            
                img = draw_circle_on_map_by_coord_and_angles(img,
                                        (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                        (lat_zero, lon_zero), pixel_size, (yaw), (0,255,0))
                # img = img[int(img.shape[0]*0.2):int(img.shape[0]*0.8), int(img.shape[1]*0.2):int(img.shape[1]*0.8)]
                self.pub_pose_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))    
        #send poses
        if self.first_cadr == False:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.main_map.main_points[0].lat, self.main_map.main_points[0].lon, 0)
            lat, lon, _ = g_c.ned2Geodetic(north=float(-self.y_meter), east=float(self.x_meter), down=0)
            self.generate_and_send_pose(lat, lon, self.imu_roll, self.imu_pitch, self.last_yaw)
            self.generate_and_send_vel(north_speed, east_speed, yaw_speed)
        
        if self.publish_calculated_pose_img is True:
            self.pub_pose_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))    

    
    def compare_cadrs(self, kp_new, kp_old, descriptors_new, descriptors_old, cadr, cadr_old):
        good = self.matcher.find_good_matches(descriptors_new, descriptors_old)
        x_center, y_center, roll, pitch, yaw_cadr, M, img = self.matcher.find_keypoints_transform(
            kp_old, kp_new, good, cadr.rasterArray, cadr_old.rasterArray)
        if self.publish_between_img is True:
            self.pub_between_image.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))
        delta_y = -1*(x_center-cadr.rasterArray.shape[1]/2)*float(cadr.pixel_size)
        delta_x =  (y_center-cadr.rasterArray.shape[0]/2)*float(cadr.pixel_size)
        x_trans = delta_x*np.cos(self.last_yaw)# - delta_y*np.sin(self.last_yaw)
        y_trans = -1*delta_x*np.sin(self.last_yaw)# - delta_y*np.cos(self.last_yaw)
        delta_time = time() - self.time_between_cadrs
        self.time_between_cadrs = time()
        # print(delta_time, yaw_cadr - np.pi/2)
        if delta_time < 2.0 and abs(yaw_cadr - np.pi/2) < 1.0:
            north_speed = y_trans/delta_time
            east_speed = x_trans/delta_time
            speed_limit = False
            if abs(north_speed) > self.low_pass_speed or abs(east_speed) > self.low_pass_speed:
                north_speed = 0
                east_speed = 0
                speed_limit = True
            # print("north speed: ",float('{:.3f}'.format(north_speed)), "east_speed: ", float('{:.3f}'.format(east_speed)), "yaw cadr: ", yaw_cadr)
            # print(speed_limit)
            yaw_speed = -1*(yaw_cadr-np.pi/2)/delta_time
            self.last_yaw -= yaw_cadr-np.pi/2
            self.x_meter += east_speed*delta_time
            self.y_meter += north_speed*delta_time
            return north_speed, east_speed, speed_limit, yaw_speed 
    
    def generate_and_send_vel(self, north_speed, east_speed, yaw_speed):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.twist.twist.linear.x = east_speed
        msg.twist.twist.linear.y = -north_speed
        msg.twist.twist.angular.z = yaw_speed
        odom_quat = tf.transformations.quaternion_from_euler(self.imu_roll, self.imu_pitch, self.last_yaw)
        msg.pose.pose.orientation.x = odom_quat[0]
        msg.pose.pose.orientation.y = odom_quat[1]
        msg.pose.pose.orientation.z = odom_quat[2]
        msg.pose.pose.orientation.w = odom_quat[3]
        self.pub_estimated_odom.publish(msg)

    def low_pass_pose(self, x, y):
        delta_time = time() - self.low_pass_time
        delta_x = abs(self.x_meter - x)
        delta_y = abs(self.y_meter - y)
        vx = delta_x/delta_time
        vy = delta_y/delta_time
        # print("vx: ",float('{:.3f}'.format(vx)), "vy: ", float('{:.3f}'.format(vy)))
        if vx > self.low_pass_coordinates or vy > self.low_pass_coordinates:
            return False
        else:
            self.low_pass_time = time()
            return True

    def generate_and_send_pose(self, lat, lon, roll, pitch, yaw):
        latlon_msg = NavSatFix()
        latlon_msg.latitude = lat
        latlon_msg.longitude = lon
        latlon_msg.altitude = self.height
        latlon_msg.header.stamp = rospy.Time.now()
        latlon_msg.header.frame_id = 'base_link'
        self.pub_latlon.publish(latlon_msg)
        
    def imu_cb(self, data):
        quat = [0,0,0,0]
        quat[0] = data.orientation.x
        quat[1] = data.orientation.y
        quat[2] = data.orientation.z
        quat[3] = data.orientation.w
        self.imu_roll, self.imu_pitch, self.imu_yaw = tf.transformations.euler_from_quaternion(quat)

    def gps_cb(self, data):
        self.lat_gps = data.latitude
        self.lon_gps = data.longitude
        self.height = data.altitude
        # self.height = 150

    def find_matches(self, roi, cadr):
        cadr_rescaled = copy.deepcopy(cadr)
        roi.img, cadr_rescaled.rasterArray = self.matcher.normalize_images(roi.img, cadr_rescaled.rasterArray)
        roi, cadr_rescaled = self.matcher.rescale_for_optimal_sift(roi, cadr_rescaled)
        kp_1, kp_2, good, img_for_pub, descriptors_1, descriptors_2 = self.matcher.find_matches(roi.img, cadr_rescaled.rasterArray)
        try:
            percent_of_good = (len(good)/len(kp_1))*100
        except: percent_of_good = 0
        return percent_of_good, kp_1, kp_2, good, img_for_pub, cadr_rescaled, descriptors_1, descriptors_2

    def load_params(self):
        rospack = rospkg.RosPack()
        data_path = rospack.get_path('image_processing') + '/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
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
    




