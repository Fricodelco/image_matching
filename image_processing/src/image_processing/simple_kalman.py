#!/usr/bin/env python3  
import rospy
import numpy as np
import rospkg
from decimal import Decimal
from time import time
from sensor_msgs.msg import Image, Imu, CompressedImage, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
import tf
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from geodetic_conv import GeodeticConvert
import logging
from coparos.msg import DroneInfo
class SimpleKalman:
    def __init__(self):
        self.realtime = self.get_realtime()
        #create global variables
        self.first_filter = True

        self.odom_init = False
        self.odom_time = time()
        
        self.pose_init = False
        self.pose_time = False
        self.lat_init = 0
        self.lon_init = 0
        self.pose_east_last = 0
        self.pose_north_last = 0
        
        self.alpha = 0.1
        self.beta = 0.1
        self.stab_x = 0.0
        self.stab_vx = 0.0
        self.stab_y = 0.0
        self.stab_vy = 0.0
        self.g_c = GeodeticConvert()
        
        #load params
        self.count_of_pictures_for_odometry = rospy.get_param("count_of_pictures_for_odometry")
        
        #ros infrustructure
        self.sub_latlon = rospy.Subscriber('/coordinates_by_img', NavSatFix, self.img_coord_cb)
        self.sub_estimated_odom = rospy.Subscriber('/odom_by_img', Odometry, self.sub_odom)
        self.pub_latlon = rospy.Publisher('/filtered_gps', NavSatFix, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def timer_callback(self, data):
        if self.pose_init == True:
            if self.first_filter == True:
                self.stab_x = self.pose_east_last
                self.stab_y = self.pose_north_last
                self.stab_vx = 0.0
                self.stab_vy = 0.0
                self.first_filter = False
                return
            print("x: ", self.stab_x, self.pose_east_last)
            self.stab_vx = (1.0 - self.alpha)*self.stab_vx + self.alpha*(self.pose_east_last - self.stab_x)
            self.stab_x = self.beta*self.pose_east_last + (1.0 - self.beta)*(self.stab_x + self.stab_vx)
            self.stab_vy = (1.0 - self.alpha)*self.stab_vy + self.alpha*(self.pose_north_last - self.stab_y)
            self.stab_y = self.beta*self.pose_north_last + (1.0 - self.beta)*(self.stab_y + self.stab_vy)
            lat_, lon_, _ = self.g_c.ned2Geodetic(north=float(self.stab_y), east=float(self.stab_x), down=0)
            # lat_, lon_, _ = self.g_c.ned2Geodetic(north=float(self.), east=float(self.stab_y), down=0)
            msg = NavSatFix()
            msg.latitude = lat_
            msg.longitude = lon_
            self.pub_latlon.publish(msg)

    def img_coord_cb(self, data):
        if self.pose_init == False:
            self.pose_init = True
            self.pose_time = time()
            self.lat_init = data.latitude
            self.lon_init = data.longitude
            self.pose_east_last = 0.0
            self.pose_north_last = 0.0
            self.g_c.initialiseReference(self.lat_init, self.lon_init, 0)
            print(self.lat_init, self.lon_init)
            return
        north, east, _ = self.g_c.geodetic2Ned(data.latitude, data.longitude, 0)
        self.pose_north_last = north
        self.pose_east_last = east
        print("priv: ", self.pose_east_last)
        
        
    def sub_odom(self, data):
        if self.pose_init == False:
            return
        if self.odom_init == False:
            self.odom_init = True
            self.odom_time = time()
            return
        delta = time() - self.odom_time
        east_speed = data.twist.twist.linear.x
        north_speed = data.twist.twist.linear.y    
        self.pose_north_last +=  north_speed*delta
        self.pose_east_last += east_speed*delta
        self.odom_time = time()
        print("odom: ", self.pose_east_last)    
    
    def get_realtime(self):
        realtime = None
        while(realtime is None):
            try:
                realtime = rospy.get_param("realtime")
            except:
                realtime = None
            rospy.sleep(0.1)
        return realtime

if __name__ == '__main__':
    rospy.init_node('simple_kalman')
    photo_publisher = SimpleKalman()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    




