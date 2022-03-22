#!/usr/bin/python3

import rospy
import time
import statistics
from math import atan2, sqrt, sin, cos
import numpy as np
from sklearn.linear_model import LinearRegression

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class GpsFilter():
    def __init__(self, filter_rate) -> None:
        self.F = np.eye(4)
        self.H = np.eye(4)
        self.R = np.eye(4)
        self.Q = np.eye(4)
        self.P = np.eye(4)

        self.filter_rate = filter_rate

        self.sigma_pos_x = 0.001
        self.sigma_pos_y = 0.001
        self.sigma_vel_x = 0.001
        self.sigma_vel_y = 0.001

        self.sigma_pos = sqrt(self.sigma_pos_x**2+self.sigma_pos_y**2)
        self.sigma_vel = sqrt(self.sigma_vel_x**2+self.sigma_vel_y**2)

        self.F[0,2] = 1/filter_rate
        self.F[1,3] = 1/filter_rate

        self.R[0,0] = self.sigma_pos**2
        self.R[1,1] = self.sigma_pos**2
        self.R[2,2] = self.sigma_vel**2
        self.R[3,3] = self.sigma_vel**2

        self.Q[0,0] = self.sigma_pos_x**2
        self.Q[1,1] = self.sigma_pos_y**2
        self.Q[2,2] = self.sigma_vel_x**2
        self.Q[3,3] = self.sigma_vel_y**2

        self.Q[0,2] = self.sigma_pos_x*self.sigma_vel_x
        self.Q[1,3] = self.sigma_pos_y*self.sigma_vel_y

        self.working_flag = False
        self.filter_gps_publisher = rospy.Publisher('/filtered_gps', NavSatFix, queue_size=10)

        rospy.Subscriber('/coordinates_by_img', NavSatFix, self.updateMeasurementPosition)
        rospy.Subscriber('/odom_by_img', Odometry, self.updateMeasurementVelocity)

    def updateMeasurementPosition(self, msg: NavSatFix):
        if self.working_flag:
            self.measurement[0] = msg.latitude
            self.measurement[1] = msg.longitude
        else:
            self.startFilter(msg.latitude, msg.longitude)

    def updateMeasurementVelocity(self, msg: Odometry):
        if self.working_flag:
            if msg.twist.twist.linear.y !=0:
                self.measurement[2] = msg.twist.twist.linear.y/6367444.65*100
            if msg.twist.twist.linear.x !=0:
                self.measurement[3] = msg.twist.twist.linear.x/6367444.65*100

    def startFilter(self, lat: float, lon: float) -> None:
        x = np.array([lat, lon, 0, 0])
        self.x = x
        self.measurement = np.array([lat, lon, 0, 0])
        self.working_flag = True
        rospy.Timer(rospy.Duration(1/self.filter_rate), self.iterOfKalmanFilter)
    
    def iterOfKalmanFilter(self, e):
        x_t = self.F@self.x
        Pt_t_1 = self.F@self.P@self.F.transpose()+self.Q
        print(self.measurement)

        y_t = self.measurement - self.H@x_t
        S = self.H@Pt_t_1@self.H.transpose() + self.R
        K = Pt_t_1@self.H.transpose()@np.linalg.inv(S)

        self.x = x_t + K@y_t
        self.P = (np.eye(4) - K@self.H)@Pt_t_1

        msg = NavSatFix()
        msg.latitude = self.x[0]
        msg.longitude = self.x[1]
        msg.header.stamp = rospy.Time.now()
        self.filter_gps_publisher.publish(msg)


class GpsLowPassFilter():
    def __init__(self, betta) -> None:
        self.lat_prev = None
        self.lon_prev = None
        self.lat = None
        self.lon = None
        self.last_update = None

        self.median_list_lat = []
        self.median_list_lon = []
        self.median_front = 15

        self.max_dif_dif_lat = 0.002
        self.max_dif_dif_lon = 0.002

        self.worked_flag = False

    def getBetta(self, newLat: float, newLon: float, dt: float) -> list:
        if dt == 0:
            return [1, 1]
        else:
            dif_lat = (newLat - self.lat)/dt
            dif_lon = (newLon - self.lon)/dt

            dif_prev_lat = (self.lat-self.lat_prev)/dt
            dif_prev_lon = (self.lon-self.lon_prev)/dt

            dif_dif_lat = abs(dif_lat-dif_prev_lat)/dt
            dif_dif_lon = abs(dif_lon-dif_prev_lon)/dt


            if dif_dif_lat > self.max_dif_dif_lat:
                betta1 = 0.05
            if dif_dif_lat > 0.5*self.max_dif_dif_lat:
                betta1 = 0.3
            else:
                betta1 = 0.7


            if dif_dif_lon > self.max_dif_dif_lon:
                betta2 = 0.05
            if dif_dif_lon > 0.5*self.max_dif_dif_lon:
                betta2 = 0.3
            else:
                betta2 = 0.7
            
        return [betta1, betta2]

    def medianFilter(self, lat:float, lon:float):
        self.median_list_lat.pop(0)
        self.median_list_lat.append(lat)
        new_lat = statistics.median(self.median_list_lat)

        self.median_list_lon.pop(0)
        self.median_list_lon.append(lon)
        new_lon = statistics.median(self.median_list_lon)

        return new_lat, new_lon

    def updateMeasurement(self, lat: float, lon:float, t: float):
        [betta1, betta2] = self.getBetta(lat, lon, t-self.last_update)
        self.lat_prev = self.lat
        self.lon_prev = self.lon
        lat_t = betta1*lat + (1-betta1)*self.lat
        lon_t = betta2*lon + (1-betta2)*self.lon

        lat_m, lon_m = self.medianFilter(lat, lon)
        self.lat = (lat_t+lat_m)/2
        self.lon = (lon_t+lon_m)/2

        self.last_update = t

    def startFilter(self, lat: float, lon:float, time: float) -> None:
        self.lat = lat
        self.lon = lon
        self.lat_prev = lat
        self.lon_prev = lon
        self.median_list_lat = [lat]*self.median_front
        self.median_list_lon = [lon]*self.median_front
        self.last_update = time
        self.worked_flag = True

    def getFilteredData(self) -> list:
        return [self.lat, self.lon]


# filter = GpsLowPassFilter(0.35)

# def gps_callback(msg: NavSatFix):
#     if filter.worked_flag:
#         filter.updateMeasurementPosition(msg.latitude, msg.longitude, time.time())
#     else:
#         filter.startFilter(msg.latitude, msg.longitude, time.time())
    
#     new_gps_msg = NavSatFix()
#     new_gps_msg.header.stamp = rospy.Time.now()
#     data = filter.getFilteredData()
#     new_gps_msg.latitude = data[0]
#     new_gps_msg.longitude = data[1]
#     filter_gps_publisher.publish(new_gps_msg)

class BaseRegressionFilter():
    def __init__(self) -> None:
        self.statr_time = None
        self.predict_front = 15

        self.time_array = []
        self.lat_array = []
        self.lon_array = []

        self.filter_gps_publisher = rospy.Publisher('/filtered_gps', NavSatFix, queue_size=10)
        rospy.Subscriber('/coordinates_by_img', NavSatFix, self.updateMeasurementPosition)

    def updateMeasurementPosition(self, msg: NavSatFix):
        if len(self.time_array) == 0:
            self.statr_time = time.time()
            self.time_array.append(0)
            self.lat_array.append(msg.latitude)
            self.lon_array.append(msg.longitude)
            new_lat = msg.latitude
            new_lon = msg.longitude

        elif len(self.time_array) == 1:
            self.time_array.append(time.time()-self.statr_time)
            self.lat_array.append(msg.latitude)
            self.lon_array.append(msg.longitude)
            new_lat = msg.latitude
            new_lon = msg.longitude

        else:
            if len(self.time_array)+1 >self.predict_front:
                self.time_array.pop(0)
                self.lat_array.pop(0)
                self.lon_array.pop(0)
            self.time_array.append(time.time()-self.statr_time)
            self.lat_array.append(msg.latitude)
            self.lon_array.append(msg.longitude)

            model_lat =  LinearRegression().fit(np.array(self.time_array).reshape((-1,1)), np.array(self.lat_array))
            model_lon =  LinearRegression().fit(np.array(self.time_array).reshape((-1,1)), np.array(self.lon_array))

            new_lat =  model_lat.intercept_ + model_lat.coef_ * self.time_array[-1]
            new_lon = model_lon.intercept_ + model_lon.coef_ * self.time_array[-1]

        new_msg = NavSatFix()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.latitude = new_lat
        new_msg.longitude = new_lon
        new_msg.altitude = msg.altitude
        self.filter_gps_publisher.publish(new_msg)
           

if __name__=='__main__':
    rospy.init_node('filter_gps_node')

    BaseRegressionFilter()
    rospy.spin()
