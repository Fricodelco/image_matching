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
import sys
from copa_msgs.msg import ImageImu
class Image_Logger:
    def __init__(self):
        self.realtime = self.get_realtime()
        print("REALTIME PARAM", self.realtime)
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        self.data_path = home+'/copa5/video/'
        self._name = self.data_path+'created_video_'+str(now)+'.mkv'
        self._fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._time = None
        self.iterator = 1
        self.first_msg = True
        self.sub_video = rospy.Subscriber('/photo', ImageImu, self.image_cb, queue_size = 1)
        self.bridge = CvBridge()
        
        self.data_path_csv = home+'/copa5/created_csv/log_postanalize_visual'+str(now)+'.csv'
        self.empty_file = True       
        self.first_msg = True
        self.sub_latlon = rospy.Subscriber('/gps', NavSatFix, self.latlon_cb, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_cb, queue_size=1)
        self.sub_baro = rospy.Subscriber('/baro', Float64, self.baro_cb, queue_size=1)
        self.sub_droneinfo = rospy.Subscriber('/droneInfo', DroneInfo, self.droneinfo_cb, queue_size=1)
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.height = 0
        self.nsat = 0
        self.bat = 0
        self.header = ["time", "lat", "lon", "alt", "roll", "pitch", "head", "ub", "nsat"]
        self.rows = []
        self.time = time()
        self.my_date = None

    def image_cb(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data.img,'bgr8')
            video_gray = False
        except:
            img = self.bridge.imgmsg_to_cv2(data.img,'8UC1')
            video_gray = True
        if self.first_msg is True:
            if video_gray is True:
                self._out = cv2.VideoWriter(self._name, self._fourcc, 5.0, (img.shape[1],img.shape[0]), 0)
            else:
                self._out = cv2.VideoWriter(self._name, self._fourcc, 5.0, (img.shape[1],img.shape[0]))
            self._time = time()
            self.first_msg = False
        else:
            self._out.write(img)
        if self.first_msg is False:
            delta = now.strftime("%H:%M:%S.%f")[:-4]
            nsat = 0
            row = {"time":str(delta),
                "lat":float('{:.6f}'.format(self.lat)),
                "lon":float('{:.6f}'.format(self.lon)),
                "alt":float('{:.3f}'.format(self.height)), 
                "roll":float('{:.3f}'.format(self.roll)),
                "pitch":float('{:.3f}'.format(self.pitch)), 
                "head":float('{:.3f}'.format(self.yaw)),
                "ub":str(float('{:.1f}'.format(self.bat))), 
                "nsat":str(self.nsat)}
            self.save_data(row)
        
        
    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    
    def droneinfo_cb(self, data):
        self.nsat = data.GPS_NUMBER_OF_SATELLITES
        self.bat = data.VBAT

    def latlon_cb(self, data):
        if self.first_msg is True:
            self.time = time()
            self.first_msg = False
            self.my_date = datetime.now()
        self.lat = data.latitude
        self.lon = data.longitude
        self.alt = data.altitude
            
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
            myFile = open(self.data_path_csv, 'w')
        else:
            myFile = open(self.data_path_csv, 'a+')
        with myFile:
            writer = csv.DictWriter(myFile, fieldnames=self.header, delimiter = ";")
            if self.empty_file:
                writer.writeheader()
            writer.writerow(row)
        if self.empty_file:
            self.empty_file = False
        
    def baro_cb(self, data):
        self.height = data.data

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
    rospy.init_node('logger')
    logger = Image_Logger()
    if logger.realtime == True:
        rospy.spin()
        logger._out.release()
        sys.stdout.write('image logger dead\n')
        




