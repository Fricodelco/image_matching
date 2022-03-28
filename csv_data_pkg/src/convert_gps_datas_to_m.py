#!/usr/bin/python3

import rospy
import rospkg
from geodetic_conv import GeodeticConvert
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
import csv
import os
from datetime import datetime
import time
import atexit

converter_gps = GeodeticConvert()
converter_gps_filtred = GeodeticConvert()
position_publisher = rospy.Publisher('/odometry', Pose, queue_size=10)
filter_position_publisher = rospy.Publisher('/filtered_odometry', Pose, queue_size=10)

write_csv_flag = True

start_pose = []
start_pose_filtred = []

rosp = rospkg.RosPack()
path = rosp.get_path('csv_data_pkg')
if write_csv_flag:
    csv_file = open(os.path.join(path, 'data', 'filter_data.csv'), 'w')
    csv_writer = csv.writer(csv_file, delimiter=';')

    header = ['time','lat','lon','alt','roll','pitch','head','ub','nsat']
    csv_writer.writerow(header)

def gps_callback(msg: NavSatFix) -> None:
    global start_pose
    new_pose = Pose()
    if converter_gps.have_reference_:
        [x,y,z] = converter_gps.geodetic2Ecef(msg.latitude, msg.longitude, msg.altitude)
        x = x - start_pose[0]
        y = y - start_pose[1]
        z = z - start_pose[2]
    else:
        converter_gps.initialiseReference(msg.latitude, msg.longitude, msg.altitude)
        start_pose = converter_gps.geodetic2Ecef(msg.latitude, msg.longitude, msg.altitude)
        x = 0.0
        y = 0.0
        z = 0.0

    new_pose.position.x = x
    new_pose.position.y = y
    new_pose.position.z = z
    position_publisher.publish(new_pose)

def filtered_gps_callback(msg: NavSatFix) -> None:
    global start_pose_filtred
    if write_csv_flag:
        global csv_writer
        csv_writer.writerow([datetime.fromtimestamp(time.time()).strftime('%H:%M:%S.%f')[:-4],msg.latitude, msg.longitude, msg.altitude, 0,0,0,0,0])

    new_pose = Pose()
    if converter_gps_filtred.have_reference_:
        [x,y,z] = converter_gps_filtred.geodetic2Ecef(msg.latitude, msg.longitude, msg.altitude)
        x = x - start_pose_filtred[0]
        y = y - start_pose_filtred[1]
        z = z - start_pose_filtred[2]
    else:
        converter_gps_filtred.initialiseReference(msg.latitude, msg.longitude, msg.altitude)
        start_pose_filtred = converter_gps_filtred.geodetic2Ecef(msg.latitude, msg.longitude, msg.altitude)
        x = 0.0
        y = 0.0
        z = 0.0

    new_pose.position.x = x
    new_pose.position.y = y
    new_pose.position.z = z
    filter_position_publisher.publish(new_pose)


if __name__=="__main__":
    rospy.init_node('publish_gps_m_node')
    rospy.Subscriber('/coordinates_by_img', NavSatFix, gps_callback)
    rospy.Subscriber('/filtered_gps', NavSatFix, filtered_gps_callback)
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
    if write_csv_flag:
        csv_file.close()

