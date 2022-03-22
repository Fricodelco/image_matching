#!/usr/bin/python3

import rospy
import numpy as np
import time
import statistics

from sensor_msgs.msg import NavSatFix

# class GpsFilter():
#     def __init__(self, filter_rate) -> None:
#         self.F = np.eye(2)
#         self.H = np.array([[1, 0,],
#                            [0, 1]])
#         self.R = np.eye(2)
#         self.Q = np.eye(2)
#         self.P = np.eye(2)

#         self.filter_rate = filter_rate

#         self.sigma_pos_x = 0.00002
#         self.sigma_pos_y = 0.00002

#         self.sigma_pos = 0.00002

#         self.R[0,0] = self.sigma_pos**2
#         self.R[1,1] = self.sigma_pos**2

#         self.Q[0,0] = self.sigma_pos_x**2
#         self.Q[1,1] = self.sigma_pos_y**2

#         self.worked_flag = False

#     def startFilter(self, lat: float, lon: float) -> None:
#         x = np.array([lat, lon])
#         self.x = x
#         self.measurement = np.array([lat, lon])
#         rospy.Timer(rospy.Duration(1/self.filter_rate), self.iterOfKalmanFilter)

#     def updateMeasurement(self, lat: float, lon:float, dt = None) -> None:
#         self.measurement = np.array([lat, lon])
    
#     def iterOfKalmanFilter(self, e):
#         x_t = self.F@self.x
#         Pt_t_1 = self.F@self.P@self.F.transpose()+self.Q

#         y_t = self.measurement - self.H@x_t
#         S = self.H@Pt_t_1@self.H.transpose() + self.R
#         K = Pt_t_1@self.H.transpose()@np.linalg.inv(S)

#         self.x = x_t + K@y_t
#         self.P = (np.eye(2) - K@self.H)@Pt_t_1

#     def getFilteredData(self) -> list:
        # return list(self.x)

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


filter_gps_publisher = rospy.Publisher('/filtered_gps', NavSatFix, queue_size=10)

# filter = GpsFilter(2)
filter = GpsLowPassFilter(0.35)

def gps_callback(msg: NavSatFix):
    if filter.worked_flag:
        filter.updateMeasurement(msg.latitude, msg.longitude, time.time())
    else:
        filter.startFilter(msg.latitude, msg.longitude, time.time())
    
    new_gps_msg = NavSatFix()
    new_gps_msg.header.stamp = rospy.Time.now()
    data = filter.getFilteredData()
    new_gps_msg.latitude = data[0]
    new_gps_msg.longitude = data[1]
    filter_gps_publisher.publish(new_gps_msg)


if __name__=='__main__':
    rospy.init_node('filter_gps_node')

    rospy.Subscriber('/gps', NavSatFix, gps_callback)
    rospy.spin()
