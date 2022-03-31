#!/usr/bin/env python3  
import rospy
import cv2
import numpy as np
from time import time
from std_msgs.msg import String, Bool, Float32
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu,  NavSatFix
import csv
from datetime import datetime
import os
import yaml
class Logger:
    def __init__(self):
        params = self.load_params()
        self.realtime = params["realtime"]
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        self.data_path = home+'/copa5/created_csv/plata_log'+str(now)+'.csv'
        self.empty_file = True       
        self.sub_latlon = rospy.Subscriber('/gps', NavSatFix, self.latlon_cb, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_cb, queue_size=1)
        self.sub_baro = rospy.Subscriber('/baro', Float32, self.baro_cb, queue_size=1)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.height = 0
        self.header = ["time", "lat", "lon", "alt", "roll", "pitch", "head", "ub", "nsat"]
        self.rows = []
        self.time = time()
        self.first_msg = True
        self.my_date = None

    def latlon_cb(self, data):
        if self.first_msg is True:
            self.time = time()
            self.first_msg = False
            self.my_date = datetime.now()
        else:
            lat = data.latitude
            lon = data.longitude
            alt = data.altitude
            now = datetime.now()
            # delta = now - self.my_date
            delta = now.strftime("%H:%M:%S.%f")[:-4]
            # delta = str(delta)[:-4]
            nsat = 0
            row = {"time":str(delta),
                "lat":float('{:.6f}'.format(lat)),
                "lon":float('{:.6f}'.format(lon)),
                "alt":float('{:.3f}'.format(self.height)), 
                "roll":float('{:.3f}'.format(self.roll)),
                "pitch":float('{:.3f}'.format(self.pitch)), 
                "head":float('{:.3f}'.format(self.yaw)),
                "ub":0, "nsat":str(nsat)}
            self.save_data(row)
            # self.rows.append(row)
            
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
            self.empty_file = False
        else:
            myFile = open(self.data_path, 'a+')
        with myFile:
            writer = csv.DictWriter(myFile, fieldnames=self.header, delimiter = ";")
            if self.empty_file:
                writer.writeheader()
            writer.writerow(row)
        
    # def save_data(self):
    #     myFile = open(self.data_path, 'w')
    #     with myFile:
    #         # writer = csv.writer(myFile)
    #         writer = csv.DictWriter(myFile, fieldnames=self.header, delimiter = ";")
    #         writer.writeheader()
    #         writer.writerows(self.rows)
            # writer.writerows(self.data)
    
    def baro_cb(self, data):
        self.height = data.data

    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    
if __name__ == '__main__':
    rospy.init_node('logger')
    logger = Logger()
    rate = rospy.Rate(10.0)
    if logger.realtime is True:
        while not rospy.is_shutdown():
            rate.sleep()
        # logger.save_data()
        




