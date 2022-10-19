#!/usr/bin/env python3  
import rospy
import cv2
import numpy as np
from time import time
from std_msgs.msg import String, Bool, Float64
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu,  NavSatFix
from coparos.msg import DroneInfo
import csv
from datetime import datetime
import os
import yaml
import subprocess, shlex, psutil
from time import sleep, time
class Logger:
    def __init__(self):
        self.realtime = self.get_realtime()
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M:%S")
        self.data_path = home+'/copa5/bags/bag'+str(now)
        self.empty_file = True       
        self.first_msg = True
        self.sub_baro = rospy.Subscriber('/baro_relative', Float64, self.baro_cb, queue_size=1)
        self.height = 0.0
        self.start_height = rospy.get_param("start_height")
        self.take_off = False
        while abs(self.height) < self.start_height:
            rospy.sleep(0.2)        
        self.take_off = True
        self.start_time = time()        
        command = "rosbag record -O "+self.data_path + " /photo /gps /baro_relative /photo_test /imu /calculated_pose"
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)
        
    def check_height(self):
        if abs(self.height) <= self.start_height and time() - self.start_time > 120:
            self.kill_bag()          
        elif self.start_height == 0.0 and time() - self.start_time > 120:
            self.kill_bag()

    
    def baro_cb(self, data):
        self.height = data.data
        if self.take_off is True:
            self.check_height()

    def kill_bag(self, data):
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)
        rosbag_proc.send_signal(subprocess.signal.SIGINT)

    def get_realtime(self):
        realtime = None
        while(realtime is None):
            try:
                realtime = rospy.get_param("realtime")
            except:
                realtime = None
            rospy.sleep(0.1)
        return realtime

    
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
            # writer.writerows(self.data)
    
    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    
if __name__ == '__main__':
    rospy.init_node('bag_recorder')
    logger = Logger()
    rate = rospy.Rate(10.0)
    # if logger.realtime is True:
    rospy.spin()




