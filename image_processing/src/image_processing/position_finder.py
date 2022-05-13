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
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
import copy
from utils import resize_img, draw_circle_on_map_by_coord_and_angles
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from geodetic_conv import GeodeticConvert
import yaml
import os
import actionlib
from copa_msgs.msg import WindSpeedAction, WindSpeedResult, WindSpeedFeedback
from datetime import datetime
import logging
import sys

class PositionFinder:
    def __init__(self):
        #load main map
        # self.main_map = image_processing('05_03_2022/2020_03_06_kor.TIF', 0)
        self.realtime = self.get_realtime()
        self.wind_speed_measure_param = rospy.get_param("wind_speed_measure")
        self.logger = self.create_logger()
        if self.wind_speed_measure_param == False:
            self.main_map = image_processing(filename=rospy.get_param("map_name"))
            self.map_pixel_size = self.main_map.find_pixel_size()
        else:
            self.main_map = image_processing(img = np.zeros((100,100)))
            self.map_pixel_size = 1
        # self.main_map = image_processing('600m/Anapa2_g.tif', 0)
        self.first_cadr = True
        #camera params
        self.poi = 84/180.0*np.pi
        self.f = 7.7
        #create matcher object
        self.matcher = match_finder()
        #create global variables
        self.last_roi = None
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.lat_gps = 0.0
        self.lon_gps = 0.0
        self.last_yaw = 0.0
        self.old_cadr = None
        self.time_between_cadrs = time()
        self.x_meter = 0
        self.y_meter = 0
        self.between_iter = 0
        self.filtered_lat = 0.0
        self.filtered_lon = 0.0
        self.north_filtered = np.zeros(5)
        self.east_filtered = np.zeros(5)
        self.low_pass_time = time()
        self.rois = []
        self.roi_iterator = 100
        self.roi_after_link = None
        self.first_rolling = True
        self.height_init = False
        self.start_height = 0.0
        self.main_cadr = None
        self.wind_velocities_x = np.empty(1)
        self.wind_velocities_y = np.empty(1)
        self.wind_time = None
        self.wind_mes_flag = False
        self.wind_cadr = None
        self.gps_init = False
        self.gps_time = False
        self.gps_trans = None
        self.pose_from_privyazka = False
        #load params
        self.search_scale_for_roi_by_gps = rospy.get_param("search_scale_for_roi_by_gps")
        self.search_scale_for_roi_by_detection = rospy.get_param("search_scale_for_roi_by_detection")
        self.search_scale_for_roi_by_rolling_window = rospy.get_param("search_scale_for_roi_by_rolling_window")
        self.count_of_pictures_for_odometry = rospy.get_param("count_of_pictures_for_odometry")
        self.low_pass_speed = rospy.get_param("low_pass_speed")
        self.low_pass_coordinates = rospy.get_param("low_pass_coordinates")
        self.use_imu = rospy.get_param("use_imu")
        self.use_gps = rospy.get_param("use_gps")
        self.use_baro = rospy.get_param("use_baro")
        self.height = rospy.get_param("height")
        self.publish_roi_img = rospy.get_param("publish_roi_img")
        self.publish_keypoints_matches_img = rospy.get_param("publish_keypoints_matches_img")
        self.publish_between_img = rospy.get_param("publish_between_img")
        self.publish_calculated_pose_img = rospy.get_param("publish_calculated_pose_img")
        self.publish_tf_img = rospy.get_param("publish_tf_img")
        self.time_sleep_before_wind_measure = rospy.get_param("time_sleep_before_wind_measure")
        self.wind_measure_time = rospy.get_param("wind_measure_time")

        #ros infrustructure
        if self.use_imu is True:
            self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        # if self.use_gps is True:
        self.sub_gps = rospy.Subscriber("gps", NavSatFix, self.gps_cb)
        if self.use_baro is True:
            self.sub_baro = rospy.Subscriber("baro", Float64, self.baro_cb)
            self.height_init = False
        else:
            self.height_init = True
        self.sub_photo = rospy.Subscriber("photo",Image, self.photo_cb, queue_size=1)
        self.bridge = CvBridge()
        if self.publish_tf_img is True:
            self.pub_image = rospy.Publisher('/find_transform', Image, queue_size=1)
        if self.publish_roi_img is True:
            self.pub_roi_image = rospy.Publisher('/roi', Image, queue_size=1)
        if self.publish_keypoints_matches_img is True:
            self.pub_keypoints_image = rospy.Publisher('/keypoints_matches', Image, queue_size=1)
        if self.publish_calculated_pose_img is True:
            self.pub_pose_image = rospy.Publisher('/calculated_pose', Image, queue_size=1)
            self.sub_filter_gps_publisher = rospy.Subscriber('/filtered_gps', NavSatFix, self.filter_cb)
            scale = 600/self.main_map.img.shape[0]
            self.map_resized = resize_img(self.main_map.img, scale)
            self.map_resized_pixel_size = self.main_map.pixel_size/Decimal(scale)
        if self.publish_between_img is True:
            self.pub_between_image = rospy.Publisher('/between_image', Image, queue_size=1)
        self.pub_latlon = rospy.Publisher('/coordinates_by_img', NavSatFix, queue_size=1)
        self.pub_pose_from = rospy.Publisher('/pose_from_privyazka', Bool, queue_size=1)
        self.pub_estimated_odom = rospy.Publisher('/odom_by_img', Odometry, queue_size=1)
        self.wind_server = actionlib.SimpleActionServer("mes_wind", WindSpeedAction, self.windCall, False)
        self.wind_server.start()
        # print("position finder ready")
        sys.stdout.write('position finder ready\n')
        self.logger.info("Position Finder ready")

    def photo_cb(self, data):
        try:
            self.log_state()
            if (self.use_baro is True and self.height_init is True) or self.use_baro is False:
                start_time = time()
                if self.realtime == False:
                    image = self.bridge.imgmsg_to_cv2(data, "8UC1")
                else:
                    image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                cadr = image_processing(img = image)
                cadr.find_pixel_size_by_height(self.height, self.poi)
                #IF THE WIND SERVER GOALED
                if self.wind_mes_flag == True:
                    cadr.find_kp_dp_scale(self.matcher)
                    self.find_wind_speed(cadr)
                    print("cadr analize time wind speed: ", time() - start_time)
                    self.logger.info("cadr analize time wind speed: "+str(time() - start_time))
                #REGULAR LOCALIZATION
                else:
                    #resize by pixel size
                    scale, map_pixel_bigger = self.matcher.find_scale(self.map_pixel_size, cadr.pixel_size)
                    if map_pixel_bigger is True:
                        # print(self.map_pixel_size, cadr.pixel_size)
                        # print(cadr.img.shape, scale)
                        cadr.img, cadr.pixel_size = self.matcher.resize_by_scale(
                                                cadr.img, cadr.pixel_size, scale)
                    else:
                        #need copy of full size map for future
                        self.main_map.img, self.main_map.pixel_size = self.matcher.resize_by_scale(
                                            self.main_map.img, self.main_map.pixel_size, scale)
                        self.map_pixel_size = self.main_map.pixel_size
                    #find match
                    cadr.find_kp_dp_scale(self.matcher)
                    self.find_pose(cadr)
                    print("cadr analize time: ", time() - start_time)
                    self.logger.info("cadr analize time: " + str(time() - start_time))
            else:
                print("HEIGHT NOT INTIALIZED")
        except Exception as e:
            self.logger.error(e)  
            print(e)  
            
    def find_pose(self, cadr):
        #check if the cadr is first
        if self.first_cadr is True:
            if self.use_gps == True:
                if self.roi_iterator > 10:
                    self.roi_after_link = self.matcher.find_map_roi_by_coordinates(self.main_map, cadr, self.lat_gps, self.lon_gps, self.search_scale_for_roi_by_gps)
                    if self.roi_after_link.img.shape[0] == 0:
                        return
                    self.roi_iterator = 0
                    self.roi_after_link, cadr = self.matcher.rescale_for_optimal_sift(self.roi_after_link, cadr)
                    self.roi_after_link.kp, self.roi_after_link.dp, self.roi_after_link.img = self.matcher.find_kp_dp(self.roi_after_link.img)        
                else:
                    self.roi_iterator += 1
                if self.roi_after_link.img.shape[0] != 0:
                    self.pose_from_roi(self.roi_after_link, cadr)
                    if self.publish_roi_img is True:
                        self.pub_roi_image.publish(self.bridge.cv2_to_imgmsg(self.roi_after_link.img, "8UC1"))
            else:
                if self.first_rolling == True:
                    rolling_window_size_x = (float(cadr.pixel_size)/float(self.main_map.pixel_size))*cadr.img.shape[1]*self.search_scale_for_roi_by_rolling_window
                    rolling_window_size_y = (float(cadr.pixel_size)/float(self.main_map.pixel_size))*cadr.img.shape[1]*self.search_scale_for_roi_by_rolling_window
                    roi_borders = self.matcher.roi_from_map(self.main_map, rolling_window_size_x, rolling_window_size_y)
                    for border in roi_borders:
                        roi = self.matcher.create_roi_from_border(self.main_map, border)
                        if roi.img.shape[0] > 0 and roi.img.shape[1] > 0:
                            roi, cadr = self.matcher.rescale_for_optimal_sift(roi, cadr)
                            print(roi.img.shape, cadr.img.shape)
                            roi.kp, roi.dp, roi.img = self.matcher.find_kp_dp(roi.img)        
                            if len(roi.kp) > 0:
                                self.rois.append(roi)
                            if self.publish_roi_img is True:
                                self.pub_roi_image.publish(self.bridge.cv2_to_imgmsg(roi.img, "8UC1"))
                    self.first_rolling = False
                else:
                    for roi in self.rois:
                        cadr = cadr
                        pose = self.pose_from_roi(roi, cadr)
                        if self.publish_roi_img is True:
                            self.pub_roi_image.publish(self.bridge.cv2_to_imgmsg(roi.img, "8UC1"))
                        if pose == True:
                            return True               
        else:
            if self.roi_iterator > 10:
                self.roi_after_link = self.matcher.roi_from_last_xy(self.main_map, float(self.x_meter), float(self.y_meter), cadr, self.search_scale_for_roi_by_detection, self.last_yaw)
                self.roi_iterator = 0
                self.roi_after_link, cadr = self.matcher.rescale_for_optimal_sift(self.roi_after_link, cadr)
                self.roi_after_link.kp, self.roi_after_link.dp, self.roi_after_link.img = self.matcher.find_kp_dp(self.roi_after_link.img)        
            else:
                self.roi_iterator += 1
            self.pose_from_roi(self.roi_after_link, cadr)
            if self.publish_roi_img is True:
                self.pub_roi_image.publish(self.bridge.cv2_to_imgmsg(self.roi_after_link.img, "8UC1"))
        
    def pose_from_roi(self, roi, cadr):
        good, img_for_pub = self.matcher.find_matches(roi, cadr)
        north_speed = 0
        east_speed = 0
        yaw_speed = 0
        speed_limit = False
        if time() - self.time_between_cadrs > self.count_of_pictures_for_odometry:
            try:
                north_speed, east_speed, speed_limit, yaw_speed = self.compare_cadrs(cadr, self.old_cadr)
            except:
                north_speed = 0
                east_speed = 0
            self.old_cadr = cadr
        
        if self.publish_keypoints_matches_img is True:
            self.pub_keypoints_image.publish(self.bridge.cv2_to_imgmsg(img_for_pub, "rgb8"))
        answer = "transform exception"
        if(len(good)>10):
            try:
                x_center, y_center, roll, pitch, yaw, M, img_tf, answer = self.matcher.find_keypoints_transform(good, roi, cadr)
            except Exception as e:
                x_center = None
                self.logger.error(e)
        else:
            answer = "not enough keypoints to transform"
            self.logger.info("could not find transform")
            x_center = None
        #show coordinates
        if self.publish_calculated_pose_img is True:
            img = copy.deepcopy(self.map_resized)
            img = draw_circle_on_map_by_coord_and_angles(img,
                                        (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                        (self.lat_gps, self.lon_gps), self.map_resized_pixel_size, (self.imu_yaw), (0,0,255))
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.main_map.main_points[0].lat, self.main_map.main_points[0].lon, 0)
            lat_mezh, lon_mezh, _ = g_c.ned2Geodetic(north=float(-self.y_meter), east=float(self.x_meter), down=0)

            img = draw_circle_on_map_by_coord_and_angles(img,
                                        (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                        (lat_mezh, lon_mezh), self.map_resized_pixel_size, (self.last_yaw), (255,0,255))
        
            img = draw_circle_on_map_by_coord_and_angles(img,
                                        (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                        (self.filtered_lat, self.filtered_lon), self.map_resized_pixel_size, (self.last_yaw), (255,255,255))
            
        self.pose_from_privyazka = False
        
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
                self.pose_from_privyazka = True
                self.x_meter = float(x_inc)
                self.y_meter = float(y_inc)
                g_c = GeodeticConvert()
                g_c.initialiseReference(self.main_map.main_points[0].lat, self.main_map.main_points[0].lon, 0)
                lat, lon, _ = g_c.ned2Geodetic(north=float(-y_inc), east=float(x_inc), down=0)
                self.logger.info("calculated lat: "+str(lat)+
                                " calculated_lon: "+str(lon))
                self.generate_and_send_pose(lat, lon, self.imu_roll, self.imu_pitch, self.last_yaw)
                if self.publish_calculated_pose_img is True:
                    img = draw_circle_on_map_by_coord_and_angles(img,
                                            (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                            (lat, lon), self.map_resized_pixel_size, (yaw), (255,0,0))
                
                    img = draw_circle_on_map_by_coord_and_angles(img,
                                            (self.main_map.main_points[0].lat, self.main_map.main_points[0].lon),
                                            (lat_zero, lon_zero), self.map_resized_pixel_size, (yaw), (0,255,0))
                    # img = img[int(img.shape[0]*0.2):int(img.shape[0]*0.8), int(img.shape[1]*0.2):int(img.shape[1]*0.8)]
                    self.pub_pose_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))    
            else:
                self.logger.info("low pass position")
        print("count of good: "+str(len(good))+
        " roi: "+str(len(roi.kp))+
        " cadr: "+str(len(cadr.kp))+' from privyazka: '+str(self.pose_from_privyazka))
        print(answer)
        self.log_keypoints(len(good), len(roi.kp), len(cadr.kp), self.pose_from_privyazka, answer)
        
        #send poses
        if self.publish_calculated_pose_img is True:
            self.pub_pose_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))    
        if x_center is not None:
            return True
        else:
            return False
    
    def compare_cadrs(self, cadr, cadr_old):
        good, _ = self.matcher.find_matches(cadr, cadr_old)
        x_center, y_center, roll, pitch, yaw_cadr, M, img = self.matcher.find_keypoints_transform(good, cadr, cadr_old)
        if x_center is not None:
            if self.publish_between_img is True:
                if img is not None:
                    self.pub_between_image.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))
            delta_x =  -1*(x_center-cadr.img.shape[1]/2)*float(cadr.pixel_size)
            delta_y =  (y_center-cadr.img.shape[0]/2)*float(cadr.pixel_size)
            x_trans = delta_y*np.sin(self.imu_yaw) - delta_x*np.cos(self.imu_yaw)
            y_trans = delta_y*np.cos(self.imu_yaw) - delta_x*np.sin(self.imu_yaw)
            delta_time = time() - self.time_between_cadrs
            self.time_between_cadrs = time()
            # print(delta_time, yaw_cadr - np.pi/2)
            if delta_time < 2.0 and abs(yaw_cadr) < 1.0:
                north_speed = y_trans/delta_time
                east_speed = x_trans/delta_time
                speed_limit = False
                if abs(north_speed) > self.low_pass_speed or abs(east_speed) > self.low_pass_speed:
                    north_speed = 0
                    east_speed = 0
                    speed_limit = True
                    self.logger.info("low pass speed")
                else:
                    self.logger.info("north_speed: "+str(north_speed)+
                                    " east_speed: "+str(east_speed))
                # print(speed_limit)
                # print("north speed: ",float('{:.3f}'.format(north_speed)), "east_speed: ", float('{:.3f}'.format(east_speed)), "yaw cadr: ", yaw_cadr)
                yaw_speed = -1*(yaw_cadr)/delta_time
                self.generate_and_send_vel(north_speed, east_speed, yaw_speed, self.pose_from_privyazka)
                
                self.last_yaw -= yaw_cadr
                self.x_meter += east_speed*delta_time
                self.y_meter -= north_speed*delta_time
                return north_speed, east_speed, speed_limit, yaw_speed 
            

    def windCall(self, goal):
        self.wind_mes_flag = True
        answer = WindSpeedResult()
        while self.wind_mes_flag == True:
            rospy.sleep(0.1)
        mean_x = np.mean(self.wind_velocities_x)
        mean_y = np.mean(self.wind_velocities_y)
        print("calculated wind speed: ", mean_x, mean_y)
        speed = np.sqrt(mean_x**2 + mean_y**2)
        alpha = np.arctan2(mean_y, mean_x)
        answer.speed = speed
        answer.angle = (alpha*180)/np.pi
        self.wind_velocities_y = np.empty(1)
        self.wind_velocities_x = np.empty(1)
        self.wind_server.set_succeeded(answer)

    def find_wind_speed(self, cadr):
        if self.main_cadr == None:
            self.main_cadr = cadr
            self.time_between_cadrs = time()
            rospy.sleep(self.time_sleep_before_wind_measure)
            self.wind_time = time()
            return None, None
        good, _ = self.matcher.find_matches(cadr, self.main_cadr)
        x_center, y_center, roll, pitch, yaw_cadr, M, img = self.matcher.find_keypoints_transform(good, cadr, self.main_cadr)
        if x_center is None:
            self.main_cadr = cadr
            self.time_between_cadrs = time()
            return None, None 
        delta_x =  -1*(x_center-cadr.img.shape[1]/2)*float(cadr.pixel_size)
        delta_y =  (y_center-cadr.img.shape[0]/2)*float(cadr.pixel_size)
        delta_t = time() - self.time_between_cadrs
        vx = delta_x/delta_t
        vy = delta_y/delta_t
        self.wind_velocities_y = np.append(self.wind_velocities_y, vy)
        self.wind_velocities_x = np.append(self.wind_velocities_x, vx)
        if time() - self.wind_time > self.wind_measure_time:
            self.wind_mes_flag = False
        # x2 = int(cadr.img.shape[1]/2 + 5*vx)
        # y2 = int(cadr.img.shape[0]/2 - 5*vy)
        # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # img = cv2.line(img, (int(cadr.img.shape[1]/2), int(cadr.img.shape[0]/2)), (x2, y2), (255,0,0), 2)
        # self.pub_between_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        
    def generate_and_send_vel(self, north_speed, east_speed, yaw_speed, pose_from_privyazka):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.twist.twist.linear.x = east_speed
        msg.twist.twist.linear.y = north_speed
        msg.twist.twist.angular.z = yaw_speed
        odom_quat = tf.transformations.quaternion_from_euler(self.imu_roll, self.imu_pitch, self.last_yaw)
        msg.pose.pose.orientation.x = odom_quat[0]
        msg.pose.pose.orientation.y = odom_quat[1]
        msg.pose.pose.orientation.z = odom_quat[2]
        msg.pose.pose.orientation.w = odom_quat[3]
        self.pub_estimated_odom.publish(msg)
        msg = Bool()
        msg.data = pose_from_privyazka
        self.pub_pose_from.publish(msg)

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

        # self.roll = data.angular_velocity.y/180.0*np.pi
        # self.pitch = data.angular_velocity.x/180.0*np.pi
        # self.yaw = data.angular_velocity.z/180.0*np.pi


    def gps_cb(self, data):
        # if self.gps_init == False:
        #     self.gps_init = True
        #     self.gps_time = time()
        # if time() - self.gps_time > self.count_of_pictures_for_odometry:
        #     g_c = GeodeticConvert()
        #     g_c.initialiseReference(self.lat_gps, self.lon_gps, 0)
        #     x, y, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
        #     self.gps_trans = np.sqrt(x*x + y*y)
        #     self.gps_time = time()
        #     print(self.gps_trans
        self.lat_gps = data.latitude
        self.lon_gps = data.longitude

    
    def filter_cb(self, data):
        self.filtered_lat = data.latitude
        self.filtered_lon = data.longitude

    def baro_cb(self, data):
        if self.height_init == False:
            self.height_init = True
            if self.realtime == True:
                self.start_height = data.data
        self.height = data.data - self.start_height
    
    def create_logger(self):
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        logname = home+'/copa5/logs/pos_finder_'+str(now)+'.log'
        # print("LOGNAME")
        # print(logname)
        logging.basicConfig(filename=logname,
                                filemode='w',
                                format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                                datefmt='%H:%M:%S',
                                level=logging.DEBUG,
                                force=True)
        logger = logging.getLogger('position_finder')
        logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(logname)
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(fmt='POSFINDER:[%(asctime)s: %(levelname)s] %(message)s'))
        return logger

    def log_state(self):
        self.logger.info("     /////ITERATION/////     ")
        self.logger.info("first_cadr: "+str(self.first_cadr)+
        " lat_gps: "+str(self.lat_gps)+
        " lon_gps: "+str(self.lon_gps)+
        " height_init: "+str(self.height_init)+
        " start_height: "+str(self.start_height)+
        " wind_mes_flag: "+str(self.wind_mes_flag)+
        " height: "+str(self.height)) 

    def log_keypoints(self, good, kp_roi, kp_cadr, pose_from_privyazka, answer):
        self.logger.info("count of good: "+str(good)+
        " roi: "+str(kp_roi)+
        " cadr: "+str(kp_cadr)+" pose from link: "+str(pose_from_privyazka))
        self.logger.info(answer)
        
    def get_realtime(self):
        realtime = None
        while(realtime is None):
            try:
                realtime = rospy.get_param("realtime")
            except:
                realtime = None
            rospy.sleep(0.1)
        return realtime

if __name__ == '__main__':
    rospy.init_node('position_finder')
    photo_publisher = PositionFinder()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    # print("position finder dead")
    sys.stdout.write('position finder dead\n')
    




