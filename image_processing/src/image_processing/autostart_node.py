#!/usr/bin/env python3
import roslaunch
import rospy
from std_msgs.msg import Float64, String
from time import time

class Start:
    def __init__(self,baseAltitude, path):
        print(baseAltitude)
        self.start_height = self.load_param()
        self.baseAltitude = baseAltitude
        self.status = False
        self.cb_status = False
        self.started = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path])
        self.sub_altitude = rospy.Subscriber("/baro",Float64, self.gps_cb, queue_size=10)
        self.log_pub = rospy.Publisher("/logging_topic", String, queue_size=1)
        self.baro_relative_pub = rospy.Publisher("/baro_relative", Float64, queue_size=2)
        self.init_time = time()

    def gps_cb(self,data):
        msg = Float64()
        msg.data = data.data - self.baseAltitude
        self.baro_relative_pub.publish(msg)
        if abs(data.data - self.baseAltitude) > self.start_height and self.started is False:
            self.init_time = time()
            self.setStatus(True)
            self.started = True
        elif abs(data.data - self.baseAltitude) <= self.start_height and self.started is True and time() - self.init_time > 30:
            self.setStatus(False)
    
    def systemStart(self):
        self.launch.start()
    
    def systemStop(self):
        self.launch.shutdown()
    
    def setStatus(self,state):
        self.cb_status = state
    
    def checkStatus(self):
        if self.status != self.cb_status:
            if self.cb_status == True:
                self.systemStart()
            else:
                self.systemStop()
            self.status = self.cb_status
    
    def load_param():
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
            for key in params:
                if key == 'start_height':     
            return params[key] 

    def find_path_to_config():
        name = 'config.yaml'
        path = '/media/jetson'
        for file in os.listdir(path):
            print("Looking for extended config in: ", path+'/'+file)
            if os.path.exists(path+'/'+file+'/config.yaml') is True:
                return path+file+'/config.yaml'
        return None  
#"/home/haier/catkin_ws/src/testapi/launch/test_node.launch"


if __name__ == "__main__":
    
    rospy.init_node('AutoStartNode', anonymous=True)
    baro = rospy.wait_for_message("/baro", Float64)
    node = Start(baro.data, "/home/jetson/copa5/ws/src/image_matching/image_processing/launch/find_pose.launch")
    rospy.loginfo("AutoStart enabled")
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        node.checkStatus()
        rate.sleep()
# 3 seconds later