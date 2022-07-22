#!/usr/bin/env python3 
import rospy
import rospkg
import csv
import os
import tf
from sensor_msgs.msg import Imu, NavSatFix
from numpy import pi
from std_msgs.msg import Float64, String
from coparos.msg import DroneInfo

class CsvRosHendler():
    def __init__(self, csv_file_path, rate_multi: str) -> None:
        #ros
        self.rate_multi = rate_multi
        self.csv_data = []
        self.time_stamps = []
        self.csv_file_path = csv_file_path
        self.imu_publisher = rospy.Publisher('/imu', Imu, queue_size=10)
        self.gps_publisher = rospy.Publisher('/gps', NavSatFix, queue_size=10)
        self.baro_publisher = rospy.Publisher('/baro_relative', Float64, queue_size=10)
        self.str_publisher = rospy.Publisher('/csv_time', String, queue_size=10)
        self.drone_info_publisher = rospy.Publisher('/droneInfo', DroneInfo, queue_size=10)
        self.__parse_csv_data()
    
    def __parse_csv_data(self) -> None:
        with open(self.csv_file_path, newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
            for i,line in enumerate(spamreader):
                if i !=0:
                    time = self.__parse_time_to_sec(line[0])
                    data = list(map(float ,line[1:]))
                    self.csv_data.append([time]+data)
                    self.time_stamps.append(line[0])

    def __parse_time_to_sec(self, date: str) -> float:
        split_date = list(map(float,date.split(':')))
        sec = split_date[0]*3600 + split_date[1]*60+split_date[2]
        return sec
    
    def start_publihs_gps_imu(self)->None:
        for i, data in enumerate(self.csv_data):
            if i !=0:
                rospy.sleep((data[0]-self.csv_data[i-1][0])/self.rate_multi)
            self.publish_gps_imu_data(i)
            if rospy.is_shutdown():
                break
        print("CSV END")

    def publish_gps_imu_data(self, index: int)->None:
        imu_msg = Imu()
        roll, pitch, yaw = self.csv_data[index][4], self.csv_data[index][5], self.csv_data[index][6]
         
        roll = roll/180.0*pi
        pitch = pitch/180.0*pi
        yaw = yaw/180.0*pi
        quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        imu_msg.header.stamp = rospy.Time.now()

        gps_msg = NavSatFix()
        gps_msg.latitude = self.csv_data[index][1]
        gps_msg.longitude = self.csv_data[index][2]
        gps_msg.altitude = self.csv_data[index][3]
        gps_msg.header.stamp = rospy.Time.now()
        
        baro_msg = Float64()
        baro_msg.data = self.csv_data[index][3]

        str_msg = String()
        str_msg.data = str(self.time_stamps[index])

        info_msg = DroneInfo()
        info_msg.GPS_NUMBER_OF_SATELLITES = int(self.csv_data[index][8])
        self.drone_info_publisher.publish(info_msg)
        self.imu_publisher.publish(imu_msg)
        self.gps_publisher.publish(gps_msg)
        self.baro_publisher.publish(baro_msg)
        self.str_publisher.publish(str_msg)


if __name__=="__main__":
    rospy.init_node('publish_csv_node')
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('csv_data_pkg')
    csvRosHandler = CsvRosHendler(os.path.join(pkg_path, 'data', 'log.csv'))
    csvRosHandler.start_publihs_gps_imu()