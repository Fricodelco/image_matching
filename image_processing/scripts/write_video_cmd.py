#!/usr/bin/env python3  
from numpy import reshape
import rospy
import cv2
import numpy as np
import rospkg
import gdal
from decimal import Decimal
from time import time
from sensor_msgs.msg import Image, CompressedImage, NavSatFix, Imu 
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float64
from cv_bridge import CvBridge
import copy
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
import csv
import rospkg
from datetime import datetime
import os
import yaml
import sys
from copa_msgs.msg import ImageImu
import logging

class Image_Logger:
    def __init__(self):
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        self.data_path = home+'/copa5/video/'
        self._name = self.data_path+'cmd_video.mkv'
        self._fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._time = None
        self.iterator = 1
        self.sub_video = rospy.Subscriber('/photo', ImageImu, self.image_cb, queue_size = 1)
        self.bridge = CvBridge()
        self.first_msg = True
        
    def image_cb(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data.img,'bgr8')
            video_gray = False
        except:
            img = self.bridge.imgmsg_to_cv2(data.img,'8UC1')
            video_gray = True
        if self.first_msg is True:
            print("start_filming")
            if video_gray is True:
                self._out = cv2.VideoWriter(self._name, self._fourcc, 5.0, (img.shape[1],img.shape[0]), 0)
            else:
                self._out = cv2.VideoWriter(self._name, self._fourcc, 5.0, (img.shape[1],img.shape[0]))
            self.first_msg = False
        else:
            self._out.write(img)
        
if __name__ == '__main__':
    rospy.init_node('cmd_video')
    logger = Image_Logger()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    print("stop_filming")
    logger._out.release()
    sys.stdout.write('image logger dead\n')
    



