#!/usr/bin/env python3  
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from time import time
from copa_msgs.msg import ImageImu
from sensor_msgs.msg import Imu
import os
from datetime import datetime
import logging
from std_msgs.msg import Float64
import numpy as np
from std_msgs.msg import Bool
# define a video capture object
# pipeline = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1920, height=1080, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
# pipeline = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1920, height=1080, format=I420 ! videoconvert h! video/x-raw, format=BGR ! appsink'

# pipeline = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=I420, appsink max-buffers=1 drop=true'
# vid = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=10,
    flip_method=1,
):
    return (
        # "nvarguscamerasrc sensor-id=%d wbmode=0 gainrange='0 16' ispdigitalgainrange='0 16' exposuretimerange='5000000 5000000' aelock=true !"
        "nvarguscamerasrc sensor-id=%d tnr-mode='1' tnr-strength='1' awblock='false' wbmode='5' scene-mode='3' exposuretimerange='13000 500000' aelock='false' orientation='2' !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method='2' ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink "
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            display_width,
            display_height,
        )
    )


# def show_camera():
#     window_title = "CSI Camera"

#     # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
#     print(gstreamer_pipeline(flip_method=0))
#     video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2, framerate=10), cv2.CAP_GSTREAMER)
#     # video_capture = cv2.VideoCapture(0)
#     # video_capture.set(3,1280)
#     # video_capture.set(4,1024)
#     if video_capture.isOpened():
#         try:
#             window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
#             while True:
#                 ret_val, frame = video_capture.read()
#                 # Check to see if the user closed the window
#                 # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
#                 # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
#                 if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
#                     cv2.imshow(window_title, frame)
#                 else:
#                     break 
#                 keyCode = cv2.waitKey(10) & 0xFF
#                 # Stop the program on the ESC key or 'q'
#                 if keyCode == 27 or keyCode == ord('q'):
#                     break
#         finally:
#             video_capture.release()
#             cv2.destroyAllWindows()
#     else:
#         print("Error: Unable to open camera")


# if __name__ == "__main__":
#     show_camera()


class PhotoPublisher:
    def __init__(self):
        enable = self.get_realtime()
        # enable = True
        self.done = False
        if enable is False:
            self.done = None            
            return None
        self.height = 0.0
        self.logger = self.create_logger()
        self.logger.info("initializing pipeline...")
        self.video_capture = None
        pipeline = gstreamer_pipeline(flip_method=0, framerate=10)
        self.logger.info("used pipeline: "+str(pipeline))
        enabled = False
        while enabled == False:
            rospy.sleep(1)
            enabled = self.init_pipeline(pipeline)
        
            
        self.init_pipeline(pipeline)
        self.imu_msg = Imu()
        self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        self.pub_image = rospy.Publisher('/photo', ImageImu, queue_size=1)
        self.sub_baro = rospy.Subscriber("baro_relative", Float64, self.baro_cb)
        self.pub_image_for_test = rospy.Publisher('/photo_test', Image, queue_size=1)
        self.pub_camera_alive = rospy.Publisher('/camera_alive', Bool, queue_size=1)
        self.bridge = CvBridge()
        self.iterator = 0
        start_height = rospy.get_param("start_height")
        self.cam_alive_msg = Bool()
        self.cam_alive_msg.data = True
        while abs(self.height) < start_height:
            self.pub_camera_alive.publish(self.cam_alive_msg)
            rospy.sleep(0.2)
        self.logger.info("csi start")

    def baro_cb(self, data):
        self.height = data.data

    def init_pipeline(self, pipeline):
        try:
            self.video_capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            self.logger.info("pipeline initialized!")
            return self.video_capture.isOpened()
        except Exception as e:
            self.logger.info("PIPELINE INIT ERROR: " + str(e))
            return False

    def imu_cb(self,data):
        self.logger.info("get imu!")
        self.imu_msg = data

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


    def get_realtime(self):
        realtime = None
        while(realtime is None):
            try:
                realtime = rospy.get_param("realtime")
            except:
                realtime = None
            rospy.sleep(0.1)
        return realtime


    def video_publisher(self):
        try:
            ret_val, frame = self.video_capture.read()
            frame = self.undistort(frame, balance=1.0, scale=0.625)    
            self.logger.info("succesfully read image")
            msg_img = Image()
            msg_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg_img_imu = ImageImu()
            msg_img_imu.img = msg_img
            msg_img_imu.imu = self.imu_msg
            self.pub_image.publish(msg_img_imu)
            self.pub_image_for_test.publish(msg_img)           
            self.pub_camera_alive.publish(self.cam_alive_msg)
            if frame.shape[0] > 0:
                self.logger.info("send photo!")
            self.iterator = 0
            return True
        except Exception as e:
            self.logger.error("ERROR: "+str(e))
            rospy.logerr(e)
            return False
    
    def create_logger(self):
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M:%S")
        logname = home+'/copa5/logs/csi_camera_'+str(now)+'.log'
        logging.basicConfig(filename=logname,
                                filemode='w',
                                format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                                datefmt='%H:%M:%S',
                                level=logging.DEBUG,
                                force=True)
        logger = logging.getLogger('photo_publisher')
        logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(logname)
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(fmt='POSFINDER:[%(asctime)s: %(levelname)s] %(message)s'))
        return logger

if __name__ == '__main__':
    rospy.init_node('photo_publisher')
    # rospy.sleep(15)
    photo_publisher = PhotoPublisher()
    # rospy.sleep(10)
    if photo_publisher.done is not None:
        print("here")
        rate = rospy.Rate(10)
        rospy.loginfo("PHOTO")
        while not rospy.is_shutdown():
            answer = photo_publisher.video_publisher()
            if answer == False:
                rospy.sleep(1)
                pipeline = gstreamer_pipeline(flip_method=2, framerate=10)
                photo_publisher.init_pipeline(pipeline)
            rate.sleep()
        rospy.loginfo("VIDEO ENDED")
