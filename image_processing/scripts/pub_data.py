#!/usr/bin/env python3 
from datetime import datetime
import rospy
import os
import cv_bridge
import csv
from sensor_msgs.msg import Imu, NavSatFix
import tf
from math import asin, pi
from sensor_msgs.msg import Image,  CompressedImage # Image is the message type
from cv_bridge import CvBridge
import cv2
import rospkg
from time import time

class ImageCsvRos():
    def __init__(self):
        self.csv_data = []
        data_path = rospack.get_path('image_processing') + '/data'
        self.count = 1
        # self.csv_file_path = data_path + '/05_03_2022/flight_1/003-acl_222.csv'
        self.csv_file_path = data_path + '/500m/video_log_5.csv'
        #self.cap = cv2.VideoCapture(0)
        # self.cap = cv2.VideoCapture(data_path + '/05_03_2022/flight_1/C0003_3.mp4')
        self.cap = cv2.VideoCapture(data_path + '/500m/500.mp4')  
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        self.rate_photo =  1 / fps
        self.rate_data = 5
        self.bridge = CvBridge()
        self.imu_publisher = rospy.Publisher('/imu', Imu, queue_size=10)
        self.gps_publisher = rospy.Publisher('/gps', NavSatFix, queue_size=10)
        self.pub = rospy.Publisher('/photo', Image, queue_size=10)
        self.pub_compressed_image = rospy.Publisher('/compressed_photo', CompressedImage, queue_size=1)
    
        self.time_1 = 0
        self.time_2 = 0
        self.__parse_csv_data()

    def __parse_csv_data(self) -> None:
        with open(self.csv_file_path) as csvfile:
            spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
            for i,line in enumerate(spamreader):
                if i !=0:
                    time = self.__parse_time_to_sec(line[0])
                    data = list(map(float ,line[1:]))
                    self.csv_data.append([time]+data)    

    def __parse_time_to_sec(self, date: str) -> float:
        split_date = list(map(float,date.split(':')))
        sec = split_date[0]*3600 + split_date[1]*60+split_date[2]
        return sec 

    def publish_gps_imu_data(self):
        try:
            imu_msg = Imu()
            roll, pitch, yaw = self.csv_data[self.count][4], self.csv_data[self.count][5], self.csv_data[self.count][6]
            roll = roll/180.0 * pi
            pitch = pitch/180.0 * pi
            yaw = yaw/180.0 * pi
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            imu_msg.header.stamp = rospy.Time.now()

            gps_msg = NavSatFix()
            gps_msg.latitude = self.csv_data[self.count][1]
            gps_msg.longitude = self.csv_data[self.count][2]
            gps_msg.altitude = self.csv_data[self.count][3]
            gps_msg.header.stamp = rospy.Time.now()

            self.imu_publisher.publish(imu_msg)
            self.gps_publisher.publish(gps_msg)
            self.count += 1

        except:
            self.time_2 = time()

    def publish_images(self, event):
        ret, frame = self.cap.read()
        if ret == True:
            rospy.loginfo(self.time_1 - self.time_2)
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # self.pub_compressed_image.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
        else:
            self.time_1 = time()
            rospy.loginfo(self.time_1 - self.time_2)
 

if __name__ == '__main__':
    rospy.init_node('photo_publisher')
    rospack = rospkg.RosPack()
    # pkg_path = rospack.get_path('csv_data_pkg')
    asinc_image = ImageCsvRos()
    # asinc_image.rate_data = asinc_image.solution_rate_data()
    rospy.Timer(rospy.Duration(asinc_image.rate_photo), asinc_image.publish_images)
    while not rospy.is_shutdown():
        try:
            rospy.sleep(asinc_image.csv_data[asinc_image.count][0]- asinc_image.csv_data[asinc_image.count - 1][0])
            asinc_image.publish_gps_imu_data()
        except:
            rospy.loginfo("end file")
