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
import yaml
from copa_msgs.msg import ImageImu
from sensor_msgs.msg import Imu

class PhotoPublisher:
    def __init__(self, photo):
        enable = self.get_realtime()
        self.done = False
        if enable is True:
            self.done = None            
            return None
        name = rospy.get_param("video_name")
        home = os.getenv("HOME")
        self.rate_multi = rospy.get_param("rate_multiplier")
        self.data_path = home+'/copa5/video/'+name
        if photo is True:
            self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
            self.photos_paths = glob.glob(self.data_path+"/photos"+"/*.jpg")
            self.photos_paths.sort()
        else:
            # self.cap = cv2.VideoCapture(data_path+'/05_03_2022/flight_1/C_18fps.mp4')
            # self.cap = cv2.VideoCapture(data_path+'/05_03_2022/flight_2/C_18fps.mp4')
            file_exists = os.path.exists(self.data_path+'.mp4')
            if file_exists is True:
                # self.cap = cv2.VideoCapture(self.data_path+'.mp4', cv2.CAP_FFMPEG)
                self.cap = cv2.VideoCapture('filesrc location=' + self.data_path+'.mp4'+' ! qtdemux ! queue ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw,format=BGRx ! queue ! videoconvert ! queue ! video/x-raw, format=BGR ! appsink', cv2.CAP_GSTREAMER)
                fps = int(self.cap.get(cv2.CAP_PROP_FPS))
                if fps == 0:
                    self.cap = cv2.VideoCapture(self.data_path+'.mp4', cv2.CAP_FFMPEG)
            else:
                file_exists = os.path.exists(self.data_path+'.MP4')
                if file_exists is True:
                    self.cap = cv2.VideoCapture('filesrc location=' + self.data_path+'.MP4'+' ! qtdemux ! queue ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw,format=BGRx ! queue ! videoconvert ! queue ! video/x-raw, format=BGR ! appsink', cv2.CAP_GSTREAMER)
                    fps = int(self.cap.get(cv2.CAP_PROP_FPS))
                    if fps == 0:
                        self.cap = cv2.VideoCapture(self.data_path+'.MP4', cv2.CAP_FFMPEG)
                else:
                    file_exists = os.path.exists(self.data_path+'.mkv')
                    if file_exists is True:
                        self.cap = cv2.VideoCapture('filesrc location=' + self.data_path+'.mkv'+' ! qtdemux ! queue ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw,format=BGRx ! queue ! videoconvert ! queue ! video/x-raw, format=BGR ! appsink', cv2.CAP_GSTREAMER)
                        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
                        if fps == 0:
                            self.cap = cv2.VideoCapture(self.data_path+'.mkv', cv2.CAP_FFMPEG)
                    else:
                        # print("NO VIDEO FILE")
                        self.done = None            
                        return None
            # self.cap.set(cv2.CAP_GSTREAMER)
            # self.cap = cv2.VideoCapture('filesrc location=/home/jetson/copa5/video/001-acl-2222.mp4 ! qtdemux ! queue ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw,format=BGRx ! queue ! videoconvert ! queue ! video/x-raw, format=BGR ! appsink', cv2.CAP_GSTREAMER)
            fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            self.rate = fps*self.rate_multi
            self.iterator = 0
        self.pub_image = rospy.Publisher('/photo', ImageImu, queue_size=1)
        self.pub_image_rqt = rospy.Publisher('/photo_for_rqt', Image, queue_size=1)
        self.imu_msg = Imu()
        self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        self.bridge = CvBridge()
        self.iterator = 0

    def imu_cb(self, data):
        self.imu_msg = data

    def get_realtime(self):
        realtime = None
        while(realtime is None):
            try:
                realtime = rospy.get_param("realtime")
            except:
                realtime = None
            rospy.sleep(0.1)
        return realtime

    def timer_callback(self, timer):
        if self.iterator < len(self.photos_paths):
            image = cv2.imread(self.photos_paths[self.iterator])
            self.iterator+=1
            # self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "rgb"))

    def video_publisher(self):
        # try:
            ret, frame = self.cap.read()
            if self.iterator > self.rate/5:
                # frame = frame[:,:,2]
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                msg_img = Image()
                msg_img = self.bridge.cv2_to_imgmsg(frame, "8UC1")
                msg_img_imu = ImageImu()
                msg_img_imu.img = msg_img
                msg_img_imu.imu = self.imu_msg
                self.pub_image.publish(msg_img_imu)
                self.pub_image_rqt.publish(msg_img)
                self.iterator = 0
            self.iterator+=1
            return True
        # except Exception as e:
            # return False
        
    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    

if __name__ == '__main__':
    rospy.init_node('photo_publisher')
    # rospy.sleep(15)
    photo_publisher = PhotoPublisher(photo = False)
    # rospy.sleep(10)
    if rospy.get_param("wind_speed_measure" == True):
        rospy.sleep(2)
    if photo_publisher.done is not None:
        rate = rospy.Rate(photo_publisher.rate)
        path = photo_publisher.data_path
        file_exists = os.path.exists(path+'.csv')
        try:
            if file_exists is True:
                csvRosHandler = CsvRosHendler(path+'.csv', photo_publisher.rate_multi)
                thread = threading.Thread(target=csvRosHandler.start_publihs_gps_imu, daemon=True)
                thread.start()
            else:
                print("NO CSV FILE")
        except Exception as e:
            print("CSV FILE BROKEN: ", e)
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path('csv_data_pkg')
        # csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', '003-acl_222.csv'))
        # csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', '001-acl - 2222.csv'))
        i = 0
        time_old = time()
        rospy.loginfo("PHOTO")
        while not rospy.is_shutdown():
            answer = photo_publisher.video_publisher()
            if answer == False:
                break
            rate.sleep()
        rospy.loginfo("VIDEO ENDED")
        thread.join()
        
