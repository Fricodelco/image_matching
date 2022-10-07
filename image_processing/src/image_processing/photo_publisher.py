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
import numpy as np
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
        
        path = str(home)+'/copa5/ws/src/image_matching/image_processing/camera_params/params.yml'
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode('K').mat()
        dist_matrix = cv_file.getNode('D').mat()
        cv_file.release()
        self.mtx = camera_matrix
        self.dist = dist_matrix


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
                frame = self.undistort(frame, balance=1.0, scale=0.625)
                # h,  w = frame.shape[:2]
                # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
                # frame = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
                # x, y, w, h = roi
                # print(frame.shape)
                # frame = frame[y:y+h, x:x+w]
                # print(frame.shape)
                # frame = cv2.undistort(frame, self.mtx, self.dist, None, None)
                # frame = self.undistort(frame)
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
    
    def undistort(self, img, balance=0.0, dim2=None, dim3=None, scale = 0.5):
        # DIM=(1920, 1080)
        # K=np.array([[982.5159865510459, 0.0, 942.15315016315], [0.0, 987.6987576270565, 541.8676162420575], [0.0, 0.0, 1.0]])
        # D=np.array([[-0.03681123778339426], [0.07699469409655353], [-0.15661955407342928], [0.09486768984401851]])
        DIM=(1920, 1080)
        K=np.array([[980.112887385314, 0.0, 941.3416228048669], [0.0, 984.9820720751916, 539.6096400309403], [0.0, 0.0, 1.0]])
        D=np.array([[-0.027361240466794334], [0.02977144485814498], [-0.06669059605260248], [0.038761129156165954]])
        # h,w = img.shape[:2]
        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        # undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # return undistorted_img
        
        dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
        assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if not dim2:
            dim2 = dim1
        if not dim3:
            dim3 = dim1
        scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_AREA, borderMode=cv2.BORDER_CONSTANT)
        start_dim_0 = int((dim1[0]*(1-scale))/2)
        start_dim_1 = int((dim1[1]*(1-scale))/2)
        target_dim_0 = int(dim1[0]*scale)
        target_dim_1 = int(dim1[1]*scale)
        frame = undistorted_img[start_dim_1:start_dim_1+target_dim_1, start_dim_0:start_dim_0+target_dim_0]
        return frame

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
        
