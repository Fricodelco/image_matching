#!/usr/bin/env python3  
import rospy
import cv2
import numpy as np
from time import time
from std_msgs.msg import String, Bool, Float64
import tf
from coparos.msg import DroneInfo
import csv
from datetime import datetime
import os
import yaml
class Logger:
    def __init__(self):
        self.realtime = self.get_realtime()
        home = os.getenv("HOME")
        now = datetime.now()
        now = now.strftime("%d:%m:%Y,%H:%M")
        self.data_path = home+'/copa5/logs/command_log'+str(now)+'.csv'
        self.empty_file = True       
        self.first_msg = True
        self.sub_string = rospy.Subscriber('/logging_topic', String, self.command_cb, queue_size=1)
        self.header = ["time", "command"]
        self.time = time()
        self.my_date = None

    def command_cb(self, data):
        self.command = data.data
        delta = now.strftime("%H:%M:%S.%f")[:-4]
        row = {"time":str(delta),
                "command":self.command}
        self.save_data(row)

    def get_realtime(self):
        realtime = None
        while(realtime is None):
            try:
                realtime = rospy.get_param("realtime")
            except:
                realtime = None
            rospy.sleep(0.1)
        return realtime
            
    def save_data(self, row):
        if self.empty_file:
            myFile = open(self.data_path, 'w')
        else:
            myFile = open(self.data_path, 'a+')
        with myFile:
            writer = csv.DictWriter(myFile, fieldnames=self.header, delimiter = ";")
            if self.empty_file:
                writer.writeheader()
            writer.writerow(row)
        if self.empty_file:
            self.empty_file = False
    
    def load_params(self):
        home = os.getenv("HOME")
        data_path = home+'/copa5/config/config.yaml'
        with open(data_path) as file:
            params = yaml.full_load(file)
        return params
    
if __name__ == '__main__':
    rospy.init_node('logger')
    logger = Logger()
    rate = rospy.Rate(10.0)
    if logger.realtime is True:
        rospy.spin()




