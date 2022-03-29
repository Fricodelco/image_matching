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
class Image_Logger:
    def __init__(self):
        params = self.load_params()
        self.realtime = params["realtime"]
        home = os.getenv("HOME")
        self.data_path = home+'/copa5/video/'
        self._name = self.data_path+'created_video.mp4'
        self._fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        self._out = cv2.VideoWriter(self._name, self._fourcc, 5.0, (1920,1080))
        self.sub_video = rospy.Subscriber('/photo', Image, self.image_cb, queue_size = 1)
        self.bridge = CvBridge()
        

    def image_cb(self, data):
        img = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self._out.write(img)

    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    


if __name__ == '__main__':
    rospy.init_node('logger')
    logger = Image_Logger()
    if logger.realtime == True:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
        logger._out.release()

        # logger.save_data()
    




