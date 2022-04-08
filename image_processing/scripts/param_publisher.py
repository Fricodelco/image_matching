#!/usr/bin/env python3  
import rospy
import os
import yaml

def load_params():
    home = os.getenv("HOME")
    data_path = home+'/copa5/config/config.yaml'
    with open(data_path) as file:
        params = yaml.full_load(file)
    return params


if __name__ == '__main__':
    rospy.init_node('book_action_client_py')
    params = load_params()
    for key in params:
        rospy.set_param(key, params[key])    
    