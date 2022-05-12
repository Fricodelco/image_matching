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
from coparos.msg import DroneInfo
class AntiSpoof:
    def __init__(self):
        self.realtime = self.get_realtime()
        # self.logger = self.create_logger()
        #create global variables
        self.rolling_filter_window = 10
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.lat_gps = 0.0
        self.lon_gps = 0.0
        self.lat_pose = 0.0
        self.lon_pose = 0.0

        self.odom_init = False
        self.odom_time = False
        self.odom_east_last = 0
        self.odom_north_last = 0
        self.odom_east = np.zeros(shape=(self.rolling_filter_window))
        self.odom_north = np.zeros(shape=(self.rolling_filter_window))

        self.pose_init = False
        self.pose_time = False
        self.pose_east_last = 0
        self.pose_north_last = 0
        self.pose_east = np.zeros(shape=(self.rolling_filter_window))
        self.pose_north = np.zeros(shape=(self.rolling_filter_window))
        self.pose_east_realtime = np.zeros(shape=(self.rolling_filter_window))
        self.pose_north_realtime = np.zeros(shape=(self.rolling_filter_window))


        self.gps_init = False
        self.gps_time = False
        self.gps_east_last = 0
        self.gps_north_last = 0
        self.gps_east = np.zeros(shape=(self.rolling_filter_window))
        self.gps_north = np.zeros(shape=(self.rolling_filter_window))

        self.noisy_lat_gps = 0.0
        self.noisy_lon_gps = 0.0
        self.noisy_gps_init = False
        self.noisy_gps_time = False
        self.noisy_gps_east_last = 0
        self.noisy_gps_north_last = 0
        
        self.noisy_gps_east = np.zeros(shape=(self.rolling_filter_window))
        self.noisy_gps_north = np.zeros(shape=(self.rolling_filter_window))
      
        self.pose_from_privyazka = False
        
        self.synchro_time = 1
        self.check_VC_time = 2
        self.nsat = 0
        self.nsat_initialized = False
        self.minimum_nsat = 9

        self.delta_north_VC = 10
        self.delta_east_VC = 10
        self.delta_north_odom = 10
        self.delta_east_odom = 10

        self.iter = 0
        #load params
        self.count_of_pictures_for_odometry = rospy.get_param("count_of_pictures_for_odometry")
        
        #ros infrustructure
        self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        # if self.use_gps is True:
        self.sub_gps = rospy.Subscriber("gps", NavSatFix, self.gps_cb)
        self.sub_gps = rospy.Subscriber("/noisy_gps", NavSatFix, self.noisy_gps_cb)
        self.sub_latlon = rospy.Subscriber('/coordinates_by_img', NavSatFix, self.img_coord_cb)
        self.sub_pose_from = rospy.Subscriber('/pose_from_privyazka', Bool, self.sub_pose_from_priv)
        self.sub_estimated_odom = rospy.Subscriber('/odom_by_img', Odometry, self.sub_odom)
        self.sub_droneinfo = rospy.Subscriber('/droneInfo', DroneInfo, self.droneinfo_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/self.synchro_time), self.timer_callback)

    def timer_callback(self, data):
        self.update_data()
        video_controller_enabled = self.check_VC()
        flag = 0
        if self.iter < self.rolling_filter_window:
            return
        if self.nsat_initialized is True:
            if video_controller_enabled is True:
                if self.nsat < self.minimum_nsat:
                    flag = 3
                else:
                    if self.check_gps_VC() is True:
                        if self.check_gps_odom() is True:
                            flag = 1
                        else:
                            flag = 5
                    else:
                        flag = 5
            else:
                if self.nsat < self.minimum_nsat:
                    flag = 4
                else:
                    if self.check_gps_odom() is True:
                        flag = 2
                    else:
                        flag = 6
        print("flag: ", flag)
    
    def update_data(self):
        self.iter +=1
        self.pose_north = self.pose_north[1:self.rolling_filter_window]
        self.pose_east = self.pose_east[1:self.rolling_filter_window]
        self.pose_north = np.append(self.pose_north, np.array([self.pose_north_last]), axis = 0)
        self.pose_east = np.append(self.pose_east, np.array([self.pose_east_last]), axis = 0)
        
        self.odom_north = self.odom_north[1:self.rolling_filter_window]
        self.odom_north =  np.append(self.odom_north, np.array([self.odom_north_last]), axis = 0)
        self.odom_east = self.odom_east[1:self.rolling_filter_window]
        self.odom_east = np.append(self.odom_east, np.array([self.odom_east_last]), axis = 0)
        
        self.gps_north = self.gps_north[1:self.rolling_filter_window]
        self.gps_north = np.append(self.gps_north, np.array([self.gps_north_last]), axis = 0)
        self.gps_east = self.gps_east[1:self.rolling_filter_window]
        self.gps_east = np.append(self.gps_east, np.array([self.gps_east_last]), axis = 0)
            
        self.noisy_gps_north = self.noisy_gps_north[1:self.rolling_filter_window]
        self.noisy_gps_north = np.append(self.noisy_gps_north, np.array([self.noisy_gps_east_last]), axis = 0)
        self.noisy_gps_east = self.noisy_gps_east[1:self.rolling_filter_window]
        self.noisy_gps_east = np.append(self.noisy_gps_east, np.array([self.noisy_gps_north_last]), axis = 0)

    def check_gps_VC(self):
        # delta_north = abs(np.mean(self.noisy_gps_north - self.pose_north))
        # delta_east = abs(np.mean(self.noisy_gps_east - self.pose_east))
        delta_north = abs(np.mean(self.gps_north - self.pose_north))
        delta_east = abs(np.mean(self.gps_east - self.pose_east))
        
        # print(delta_north)
        print("VC delta_north: ", delta_north, " delta_east: ", delta_east)
        if delta_north < self.delta_north_VC:
            if delta_east < self.delta_east_VC:
                return True
        return False
        
    
    def check_gps_odom(self):
        # delta_north = float(abs(np.mean(self.noisy_gps_north - self.odom_north)))
        # delta_east = float(abs(np.mean(self.noisy_gps_east - self.odom_east)))
        delta_north = float(abs(np.mean(self.gps_north - self.odom_north)))
        delta_east = float(abs(np.mean(self.gps_east - self.odom_east)))
        
        # print(delta_north)
        print("ODOM delta_north: ", delta_north, " delta_east: ", delta_east)
        if delta_north < self.delta_north_odom:
            if delta_east < self.delta_east_odom:
                return True        
        return False

    def check_VC(self): 
        if time() - self.pose_time < self.check_VC_time:
            for i in range(0, self.rolling_filter_window-1):
                delta_north = self.pose_north_realtime[i] - self.pose_north_realtime[i+1]
                if abs(delta_north) > 30:
                    return False
                delta_east = self.pose_north_realtime[i] - self.pose_north_realtime[i+1]
                if abs(delta_east) > 30:
                    return False
        else:
            return False
        return True

    def droneinfo_cb(self, data):
        self.nsat_initialized = True
        self.nsat = data.GPS_NUMBER_OF_SATELLITES

    def img_coord_cb(self, data):
        if self.pose_init == False:
            self.pose_init = True
            self.pose_time = time()
            self.lat_pose = data.latitude
            self.lon_pose = data.longitude
            return
        if time() - self.pose_time > self.synchro_time:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.lat_pose, self.lon_pose, 0)
            north, east, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
            self.pose_north_last = north
            self.pose_east_last = east
            self.lat_pose = data.latitude
            self.lon_pose = data.longitude
            self.pose_time = time()
        else:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.lat_pose, self.lon_pose, 0)
            north, east, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
            self.pose_north_realtime = self.pose_north_realtime[1:self.rolling_filter_window]
            self.pose_east_realtime = self.pose_east_realtime[1:self.rolling_filter_window]
            self.pose_north_realtime = np.append(self.pose_north_realtime, np.array([north]), axis = 0)
            self.pose_east_realtime = np.append(self.pose_east_realtime, np.array([east]), axis = 0)
        
    def sub_pose_from_priv(self, data):
        a = 1
        
    def sub_odom(self, data):
        if self.odom_init == False:
            self.odom_init = True
            self.odom_time = time()
            return
        delta = time() - self.odom_time
        if delta > self.synchro_time:
            east_speed = data.twist.twist.linear.x
            north_speed = data.twist.twist.linear.y         
            self.odom_north_last =  north_speed*delta
            self.odom_east_last = east_speed*delta
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
            self.lat_gps = data.latitude
            self.lon_gps = data.longitude
            return
        if time() - self.gps_time > self.synchro_time:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.lat_gps, self.lon_gps, 0)
            north, east, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
            self.gps_north_last = north
            self.gps_east_last = east
            self.lat_gps = data.latitude
            self.lon_gps = data.longitude
            self.gps_time = time()

    def noisy_gps_cb(self, data):
        if self.noisy_gps_init == False:
            self.noisy_gps_init = True
            self.noisy_gps_time = time()
            self.noisy_lat_gps = data.latitude
            self.noisy_lon_gps = data.longitude
            return
        if time() - self.noisy_gps_time > self.synchro_time:
            g_c = GeodeticConvert()
            g_c.initialiseReference(self.noisy_lat_gps, self.noisy_lon_gps, 0)
            north, east, _ = g_c.geodetic2Ned(data.latitude, data.longitude, 0)
            self.noisy_gps_north_last = north
            self.noisy_gps_east_last = east
            self.noisy_lat_gps = data.latitude
            self.noisy_lon_gps = data.longitude
            self.noisy_gps_time = time()

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
    




