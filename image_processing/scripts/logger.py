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
from sensor_msgs.msg import Image, CompressedImage, NavSatFix, Imu 
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import copy
from utils import resize_img, draw_circle_on_map_by_coord_and_angles
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
import csv
import rospkg
from datetime import datetime
import os
import yaml
class Logger:
    def __init__(self):
        #load main map
        
        # rospack = rospkg.RosPack()
        params = self.load_params()
        self.realtime = params["realtime"]
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        self.data_path = home+'/copa5/created_csv/log'+str(now)+'.csv'
        self.sub_latlon = rospy.Subscriber('/filtered_gps', NavSatFix, self.latlon_cb, queue_size=1)
        # self.sub_estimated_odom = rospy.Subscriber('/odom_by_img', Odometry, self.odom_cb, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_cb, queue_size=1)
        self.pose_from_privyazka = False
        self.sub_pose_from_privyazka = rospy.Subscriber('/pose_from_privyazka', Bool, self.pose_from_cb, queue_size=1)
        self.empty_file = True
        if self.realtime is False:
            self.sub_time = rospy.Subscriber('/csv_time', String, self.csv_time_cb, queue_size=1)
            self.time_csv = None
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.header = ["time", "lat", "lon", "alt", "roll", "pitch", "head", "ub", "nsat"]
        self.rows = []
        self.time = time()
        self.first_msg = True
        self.my_date = None

    def pose_from_cb(self, data):
        self.pose_from_privyazka = data.data

    def latlon_cb(self, data):
        if self.first_msg is True:
            self.time = time()
            self.first_msg = False
            self.my_date = datetime.now()
        else:
            lat = data.latitude
            lon = data.longitude
            alt = data.altitude
            if self.realtime is True:
                now = datetime.now()
                # delta = now - self.my_date
                delta = now.strftime("%H:%M:%S.%f")[:-4]
                # delta = str(delta)[:-4]
            else:
                delta = self.time_csv
            if self.pose_from_privyazka is True:
                nsat = 1
            else:
                nsat = 2
            row = {"time":str(delta),
                "lat":float('{:.6f}'.format(lat)),
                "lon":float('{:.6f}'.format(lon)),
                "alt":float('{:.3f}'.format(alt)), 
                "roll":float('{:.3f}'.format(self.roll)),
                "pitch":float('{:.3f}'.format(self.pitch)), 
                "head":float('{:.3f}'.format(self.yaw)),
                "ub":0, "nsat":str(nsat)}
            self.save_data(row)
            # self.rows.append(row)
            

    def odom_cb(self, data):
        self.roll = data.pose.pose.position.x
        self.pitch = data.pose.pose.position.y
        self.yaw = data.pose.pose.position.z

    def imu_cb(self, data):
        quat = [0,0,0,0]
        quat[0] = data.orientation.x
        quat[1] = data.orientation.y
        quat[2] = data.orientation.z
        quat[3] = data.orientation.w
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(quat)
        self.roll = (self.roll*180)/np.pi
        self.pitch = (self.pitch*180)/np.pi
        self.yaw = (self.yaw*180)/np.pi

    def save_data(self, row):
        if self.empty_file:
            myFile = open(self.data_path, 'w')
        else:
            myFile = open(self.data_path, 'a+')
        with myFile:
            writer = csv.DictWriter(myFile, fieldnames=self.header, delimiter = ";")
            if self.empty_file:
                writer.writeheader()
            writer.writerow(row)
        if self.empty_file:
            self.empty_file = False
        
    
    # def save_data(self):
    #     myFile = open(self.data_path, 'w')
    #     with myFile:
    #         # writer = csv.writer(myFile)
    #         writer = csv.DictWriter(myFile, fieldnames=self.header, delimiter = ";")
    #         writer.writeheader()
    #         writer.writerows(self.rows)
    #         # writer.writerows(self.data)
    
    def csv_time_cb(self, data):
        self.time_csv = data.data

    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    


if __name__ == '__main__':
    rospy.init_node('logger')
    Logger = Logger()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    # Logger.save_data()
    




