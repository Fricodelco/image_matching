#!/usr/bin/env python3  
import rospy
import glob
import rospkg
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class PhotoPublisher:
    def __init__(self, photo):
        rospack = rospkg.RosPack()
        data_path = rospack.get_path('image_processing') + '/data'
        if photo is True:
            self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
            self.photos_paths = glob.glob(data_path+"/photos"+"/*.jpg")
            self.photos_paths.sort()
        else:
            self.cap = cv2.VideoCapture(data_path+'/500m'+'/500.mp4')
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
        print(frame.shape)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.pub_compressed_image.publish(self.bridge.cv2_to_compressed_imgmsg(frame))


if __name__ == '__main__':
    rospy.init_node('photo_publisher')
    photo_publisher = PhotoPublisher(photo = False) 
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        photo_publisher.video_publisher()
        rate.sleep()