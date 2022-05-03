#!/usr/bin/env python3  
import rospy
import numpy as np
import rospkg
from decimal import Decimal
from time import time
from sensor_msgs.msg import Image, Imu, CompressedImage, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from geodetic_conv import GeodeticConvert
import logging

class AntiSpoof:
    def __init__(self):
        self.realtime = self.get_realtime()
        # self.logger = self.create_logger()
        #create global variables
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.lat_gps = 0.0
        self.lon_gps = 0.0
        self.lat_pose = 0.0
        self.lon_pose = 0.0

        self.odom_init = False
        self.odom_time = False
        self.odom_east = np.empty(shape=(0))
        self.odom_north = np.empty(shape=(0))

        self.pose_init = False
        self.pose_time = False
        self.pose_east = np.empty(shape=(0))
        self.pose_north = np.empty(shape=(0))


        self.gps_init = False
        self.gps_time = False
        self.gps_east = np.empty(shape=(0))
        self.gps_north = np.empty(shape=(0))
        self.pose_from_privyazka = False
        #load params
        self.count_of_pictures_for_odometry = rospy.get_param("count_of_pictures_for_odometry")
        
        #ros infrustructure
        self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        # if self.use_gps is True:
        self.sub_gps = rospy.Subscriber("gps", NavSatFix, self.gps_cb)
        self.sub_latlon = rospy.Subscriber('/coordinates_by_img', NavSatFix, self.img_coord_cb)
        self.sub_pose_from = rospy.Subscriber('/pose_from_privyazka', Bool, self.sub_pose_from_priv)
        self.sub_estimated_odom = rospy.Subscriber('/odom_by_img', Odometry, self.sub_odom)
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)

    def timer_callback(self, data):
        gps_east = np.mean(self.gps_east)
        gps_north = np.mean(self.gps_north)
        odom_east = np.mean(self.odom_east)
        odom_north = np.mean(self.odom_north)
        pose_east = np.mean(self.pose_east)
        pose_north = np.mean(self.pose_north)
        if self.gps_east.shape[0] > self.odom_east.shape[0]:
            self.gps_east = self.gps_east[:(self.odom_east.shape[0])]
            self.gps_north = self.gps_north[:(self.odom_north.shape[0])]
        else:
            self.odom_east = self.gps_east[:(self.gps_east.shape[0])]
            self.odom_north = self.gps_north[:(self.gps_north.shape[0])]
        correlation_odom_east = np.corrcoef(self.gps_east, self.odom_east)
        correlation_odom_north = np.corrcoef(self.gps_north, self.odom_north)
        print("correlation: ", correlation_odom_north, correlation_odom_east)
        # print("gps: ", gps_east, gps_north)
        # print("odom: ", odom_east, odom_north)
        # print("pose: ", pose_east, pose_north)
        self.gps_east = np.empty(shape=(0))
        self.gps_north = np.empty(shape=(0))
        self.odom_east = np.empty(shape=(0))
        self.odom_north = np.empty(shape=(0))
        self.pose_east = np.empty(shape=(0))
        self.pose_north = np.empty(shape=(0))
        


    def img_coord_cb(self, data):
        if self.pose_init == False:
            self.pose_init = True
            self.pose_time = time()
        if time() - self.pose_time > self.count_of_pictures_for_odometry:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.lat_pose, self.lon_pose, 0)
            north, east, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
            self.pose_north = np.append(self.pose_north, np.array([north]), axis = 0)
            self.pose_east = np.append(self.pose_east, np.array([east]), axis = 0)
            self.lat_pose = data.latitude
            self.lon_pose = data.longitude
            self.pose_time = time()

    def sub_pose_from_priv(self, data):
        a = 1
        
    def sub_odom(self, data):
        if self.odom_init == False:
            self.odom_init = True
            self.odom_time = time()
        delta = time() - self.odom_time
        east_speed = data.twist.twist.linear.x
        north_speed = data.twist.twist.linear.y         
        self.odom_east = np.append(self.odom_east, np.array([east_speed*delta]), axis = 0)
        self.odom_north =  np.append(self.odom_north, np.array([north_speed*delta]), axis = 0)
        self.odom_time = time()

    def imu_cb(self, data):
        quat = [0,0,0,0]
        quat[0] = data.orientation.x
        quat[1] = data.orientation.y
        quat[2] = data.orientation.z
        quat[3] = data.orientation.w
        self.imu_roll, self.imu_pitch, self.imu_yaw = tf.transformations.euler_from_quaternion(quat)

    def gps_cb(self, data):
        if self.gps_init == False:
            self.gps_init = True
            self.gps_time = time()
        if time() - self.gps_time > self.count_of_pictures_for_odometry:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.lat_gps, self.lon_gps, 0)
            north, east, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
            self.gps_north = np.append(self.gps_north, np.array([north]), axis = 0)
            self.gps_east = np.append(self.gps_east, np.array([east]), axis = 0)
            self.lat_gps = data.latitude
            self.lon_gps = data.longitude
            self.gps_time = time()

    
    def filter_cb(self, data):
        self.filtered_lat = data.latitude
        self.filtered_lon = data.longitude

    def baro_cb(self, data):
        if self.height_init == False:
            self.height_init = True
            if self.realtime == True:
                self.start_height = data.data
        self.height = data.data - self.start_height
    
    
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
    rospy.init_node('anti_spoofer')
    photo_publisher = AntiSpoof()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    




