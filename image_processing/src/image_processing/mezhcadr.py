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
from std_msgs.msg import Float32, Bool
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

class MezhCadr:
    def __init__(self):
        self.load_params()
        self.first_cadr_bool = True
        #camera params
        self.poi = 84/180.0*np.pi
        self.f = 7.7
        #create matcher object
        self.matcher = match_finder()
        #create global variables
        self.height = 0.0
        self.height_init = False
        self.lat_gps = None
        self.lon_gps = None
        self.gps_init = False
        self.imu_roll = None
        self.imu_pitch = None
        self.imu_yaw = None
        self.main_cadr = None
        self.time_between_cadrs = None
        self.cadr_counter = 0
        self.x_pose = 0.0
        self.y_pose = 0.0
        self.time_of_work = None

        self.use_baro = rospy.get_param("use_baro")
        self.working_time = rospy.get_param("working_time")
        self.realtime = rospy.get_param("realtime")
        self.count_of_pictures_for_odometry = rospy.get_param("count_of_pictures_for_odometry")
        self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        self.sub_gps = rospy.Subscriber("gps", NavSatFix, self.gps_cb)
        if self.use_baro is True:
            self.sub_baro = rospy.Subscriber("baro", Float32, self.baro_cb)
        else:
            self.height = rospy.get_param("height")
    
        self.sub_photo = rospy.Subscriber("photo",Image, self.photo_cb, queue_size=1)
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/find_transform', Image, queue_size=1)
        self.pub_latlon = rospy.Publisher('/filtered_gps', NavSatFix, queue_size=1)
        self.pub_pose_from = rospy.Publisher('/pose_from_privyazka', Bool, queue_size=1)
        self.pub_estimated_odom = rospy.Publisher('/odom_by_img', Odometry, queue_size=1)
        sys.stdout.write('mezhcadr ready\n')
        
    def photo_cb(self, data):
        if self.time_of_work is None:
            self.time_of_work = time()
        try:
            tow = time() - self.time_of_work
            if ((self.use_baro is True and self.height_init is True) or self.use_baro is False) and (self.gps_init is True) and (tow < self.working_time):
                start_time = time()
                if self.realtime == False:
                    image = self.bridge.imgmsg_to_cv2(data, "8UC1")
                else:
                    image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                if(self.first_cadr_bool is True):
                    self.first_cadr_bool = False
                    self.main_cadr = image_processing(img = image)  
                    self.main_cadr.find_pixel_size_by_height(self.height, self.poi)
                    self.main_cadr.find_kp_dp_scale(self.matcher)
                    return
                if self.cadr_counter > self.count_of_pictures_for_odometry:
                    cadr = image_processing(img = image)
                    cadr.find_pixel_size_by_height(self.height, self.poi)
                    cadr.find_kp_dp_scale(self.matcher)
                    self.find_wind_speed(cadr)
                    self.main_cadr = cadr
                    self.cadr_counter = 0
                else:
                    self.cadr_counter += 1
                print("cadr analize time: ", time() - start_time)
        except Exception as e:
            print(e) 
            
    def compare_cadrs(self, cadr, cadr_old):
        good, _ = self.matcher.find_matches(cadr, cadr_old)
        x_center, y_center, roll, pitch, yaw_cadr, M, img = self.matcher.find_keypoints_transform(good, cadr, cadr_old)
        if x_center is not None:
            if self.publish_between_img is True:
                if img is not None:
                    self.pub_between_image.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))
            delta_y = -1*(x_center-cadr.img.shape[1]/2)*float(cadr.pixel_size)
            delta_x =  (y_center-cadr.img.shape[0]/2)*float(cadr.pixel_size)
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

    def find_wind_speed(self, cadr):
        good, _ = self.matcher.find_matches(cadr, self.main_cadr)
        x_center, y_center, roll, pitch, yaw_cadr, M, img = self.matcher.find_keypoints_transform(good, cadr, self.main_cadr)
        if x_center is None:
            self.main_cadr = cadr
            self.time_between_cadrs = time()
            return None, None 
        delta_x =  -1*(x_center-cadr.img.shape[1]/2)*float(cadr.pixel_size)
        delta_y =  (y_center-cadr.img.shape[0]/2)*float(cadr.pixel_size)
        self.imu_yaw = self.imu_yaw - yaw_cadr + np.pi/2
        x_trans = delta_y*np.sin(self.imu_yaw) - delta_x*np.cos(self.imu_yaw)
        y_trans = delta_y*np.cos(self.imu_yaw) - delta_x*np.sin(self.imu_yaw)
        self.x_pose += x_trans
        self.y_pose += y_trans
        g_c = GeodeticConvert()
        g_c.initialiseReference(self.lat_gps, self.lon_gps, 0)
        lat_mezh, lon_mezh, _ = g_c.ned2Geodetic(north=float(self.y_pose), east=float(self.x_pose), down=0)
        self.generate_and_send_pose(lat_mezh, lon_mezh, self.imu_roll, self.imu_pitch, self.imu_yaw)
        # delta_t = time() - self.time_between_cadrs
        # vx = delta_x/delta_t
        # vy = delta_y/delta_t
        # self.wind_velocities_y = np.append(self.wind_velocities_y, vy)
        # self.wind_velocities_x = np.append(self.wind_velocities_x, vx)
        # if time() - self.wind_time > self.wind_measure_time:
        #     self.wind_mes_flag = False
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
        msg.twist.twist.linear.y = -north_speed
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
        print("IMU START", self.imu_roll, self.imu_pitch, self.imu_yaw)

    def gps_cb(self, data):
        self.lat_gps = data.latitude
        self.lon_gps = data.longitude
        self.gps_init = True
        print("GPS ", self.lat_gps, self.lon_gps)
    
    def filter_cb(self, data):
        self.filtered_lat = data.latitude
        self.filtered_lon = data.longitude

    def baro_cb(self, data):
        if self.height_init == False:
            self.height_init = True
        self.height = data.data
        
    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config_mezhcadr.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        for key in params:
            rospy.set_param(key, params[key])       

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
    photo_publisher = MezhCadr()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    # print("position finder dead")
    sys.stdout.write('position finder dead\n')
    




