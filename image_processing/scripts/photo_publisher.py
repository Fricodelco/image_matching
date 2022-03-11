#!/usr/bin/env python3  
import rospy
import glob
import rospkg
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from csv_data_pkg.publish_csv import CsvRosHendler
from time import time
import os
import threading
class PhotoPublisher:
    def __init__(self, photo):
        rospack = rospkg.RosPack()
        data_path = rospack.get_path('image_processing') + '/data'

        if photo is True:
            self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
            self.photos_paths = glob.glob(data_path+"/photos"+"/*.jpg")
            self.photos_paths.sort()
        else:
            # self.cap = cv2.VideoCapture(data_path+'/05_03_2022/flight_1/C_18fps.mp4')
            self.cap = cv2.VideoCapture(data_path+'/05_03_2022/flight_2/C_18fps.mp4')
            # self.cap = cv2.VideoCapture(data_path+'/500m'+'/500.mp4')
            # self.cap = cv2.VideoCapture(data_path+'/600m'+'/600.mp4')
            fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            # total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            # estemated_time = 133
            print(fps)
            self.rate = fps
        self.pub_image = rospy.Publisher('/photo', Image, queue_size=1)
        self.pub_compressed_image = rospy.Publisher('/compressed_photo', CompressedImage, queue_size=1)
        self.bridge = CvBridge()
        self.iterator = 0

    def timer_callback(self, timer):
        if self.iterator < len(self.photos_paths):
            image = cv2.imread(self.photos_paths[self.iterator])
            self.iterator+=1
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "rgb"))
            self.pub_compressed_image.publish(self.bridge.cv2_to_compressed_imgmsg(image))

    def video_publisher(self):
        ret, frame = self.cap.read()
        # print(frame.shape)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.pub_compressed_image.publish(self.bridge.cv2_to_compressed_imgmsg(frame))


if __name__ == '__main__':
    rospy.init_node('photo_publisher')
    photo_publisher = PhotoPublisher(photo = False) 
    rate = rospy.Rate(photo_publisher.rate)
    # rate = rospy.Rate(30)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('csv_data_pkg')
    # csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', '003-acl_222.csv'))
    csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', '001-acl - 2222.csv'))
    # csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', 'video_log_5.csv'))
    # csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', '600m.csv'))
    thread = threading.Thread(target=csvRosHandler.start_publihs_gps_imu, daemon=True)
    thread.start()
    i = 0
    time_old = time()
    while not rospy.is_shutdown():
        photo_publisher.video_publisher()
        # try:
            # time_delta = csvRosHandler.csv_data[i][0] - csvRosHandler.csv_data[i-1][0]
            # if(time()-time_old > time_delta):
                # print(time_delta)
                # time_old = time()
                # csvRosHandler.publish_gps_imu_data(i)
                # i+=1
        # except: a = 1
        rate.sleep()
    thread.join()