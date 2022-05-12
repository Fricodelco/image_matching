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

class GeodeticParam():
    def __init__(self) -> None:
        self.a = 6378137
        self.b = 6356752.3142
        self.e_2 = 1 - self.b**2/self.a**2

    def getCurrentRadius(self, lat, lon, alt) -> float:
        N = self.a/(np.sqrt(1-self.e_2*np.sin(lat)**2))

        x = (N + alt)*np.cos(lat)*np.cos(lon)
        y = (N + alt)*np.cos(lat)*np.sin(lon)

        z = (self.b**2/self.a**2*N + alt)*np.sin(lat)

        return np.sqrt(x**2 + y**2 + z**2)

class GpsNoizer():
    def __init__(self) -> None:
        self.last_lon = 0.0
        self.noisy_lon = 0.0
        self.last_lat = 0.0
        self.noisy_lat = 0.0
        self.first_msg = False
        
        self.sub_gps = rospy.Subscriber("gps", NavSatFix, self.gps_data_cb)
        self.publisher = rospy.Publisher('/noisy_gps', NavSatFix, queue_size=10)
        self.geo_param = GeodeticParam()

        self.rate = 1/5
        self.new_msg = False

        self.msg = NavSatFix()

        self.accuracy = [5, 5, 5]

        self.w1 = 0.5
        self.w2 = 0.9
        self.w3 = 1
        self.accuracy_blowout_k = [2, 2, 0]
        self.acuuracy_max_blowout_k = [3, 3, 0]

    def gps_data_cb(self, msg) ->None:
        if self.first_msg == False:
            self.first_msg = True
            self.last_lat = msg.latitude
            self.last_lon = msg.longitude
            self.noisy_lat = msg.latitude
            self.noisy_lon = msg.longitude
            return

        g_c = GeodeticConvert()
        g_c.initialiseReference(self.last_lat, self.last_lon, 0)
        north, east, _ = g_c.geodetic2Ned(msg.latitude, msg.longitude, 0)
        self.last_lat = msg.latitude
        self.last_lon = msg.longitude

        g_c.initialiseReference(self.noisy_lat, self.noisy_lon, 0)
        self.noisy_lat, self.noisy_lon, _ = g_c.ned2Geodetic(north=north, east=east, down=0)
        self.noisy_lat, self.noisy_lon, _ = self.probabilisticModel(self.noisy_lat, self.noisy_lon, msg.altitude)
        new_msg = NavSatFix()
        new_msg.header = msg.header
        new_msg.latitude = self.noisy_lat
        new_msg.longitude = self.noisy_lon
        new_msg.altitude = msg.altitude
        g_c.initialiseReference(0, 0, 0)
        north, east, _ = g_c.geodetic2Ned(self.last_lat-self.noisy_lat, self.last_lon-self.noisy_lon, 0)
        
        print("lat: ", north)
        print("lon: ", east)
        self.publisher.publish(new_msg)


    def probabilisticModel(self, latitude, longitude, altitude) -> list:
        err = [0, 0, 0]
        accuracy_grad = np.array(self.accuracy[:2])/self.geo_param.getCurrentRadius(lat = latitude, lon = longitude, alt = altitude)*180/np.pi
        accuracy_grad = accuracy_grad.tolist()
        accuracy_grad.append(self.accuracy[2])

        for i, _ in enumerate(err):
            w = np.random.rand()
            if w < self.w1:
                err[i] = np.random.normal(0, accuracy_grad[i]/3)
            elif w < self.w2:
                err[i] = 2*self.accuracy_blowout_k[i]*accuracy_grad[i]*np.random.rand()-self.accuracy_blowout_k[i]*accuracy_grad[i]
            else:
                err[i] = 2*self.acuuracy_max_blowout_k[i]*accuracy_grad[i]*np.random.rand()-self.acuuracy_max_blowout_k[i]*accuracy_grad[i]
        
        latitude += err[0]
        longitude += err[1]
        altitude += err[2]

        return [latitude + err[0], longitude + err[1], altitude + err[2]]


if __name__ == '__main__':
    rospy.init_node('gps_noiser')
    photo_publisher = GpsNoizer()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
    




