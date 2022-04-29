#!/usr/bin/env python3  
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from time import time

  
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
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
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

        self.video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2, framerate=10), cv2.CAP_GSTREAMER)
        self.pub_image = rospy.Publisher('/photo', Image, queue_size=1)
        self.bridge = CvBridge()
        self.iterator = 0

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
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.iterator = 0
            return True
        except Exception as e:
            rospy.loginfo(e)
            return False

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
                break
            rate.sleep()
        rospy.loginfo("VIDEO ENDED")
