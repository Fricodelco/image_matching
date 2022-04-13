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
        self.realtime = rospy.get_param("realtime")
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        self.data_path = home+'/copa5/video/'
        self._name = self.data_path+'created_video_'+str(now)+'.mp4'
        # self._fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        self._time = None
        self.iterator = 1
        # self._fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self._out = None
        self._out = cv2.VideoWriter(self._name, self._fourcc, 5.0, (1920,1080))
        self.first_msg = True
        self.sub_video = rospy.Subscriber('/photo', Image, self.image_cb, queue_size = 1)
        self.bridge = CvBridge()
        
    def image_cb(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data,'bgr8')
        except:
            img = self.bridge.imgmsg_to_cv2(data,'8UC1')
        # print(img.shape)
        if self.first_msg is True:
        #     self._out = cv2.VideoWriter(self._name, self._fourcc, 4.0, (img.shape[1],img.shape[0]))
            self._time = time()
            self.first_msg = False
        self._out.write(img)
        if time()-self._time > 60:
            os.system("sync")
        #     print("start saving video part")
        #     self._out.release()
        #     self._out = cv2.VideoWriter(self.data_path+'created_video'+str(self.iterator)+'.mp4',
        #                     self._fourcc, 4.0, (img.shape[1],img.shape[0]))
        #     cap = cv2.VideoCapture(self.data_path+'created_video'+str(self.iterator)+'.mp4')
        #     self.iterator+=1
        #     self._time = time()
        #     print("end saving video part")

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
        # print("image logger dead")
        sys.stdout.write('image logger dead\n')
        # logger.save_data()
    




