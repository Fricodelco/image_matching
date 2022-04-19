#!/usr/bin/env python3  
import rospy
import actionlib
from copa_msgs.msg import WindSpeedAction, WindSpeedResult, WindSpeedFeedback, WindSpeedGoal
import os
def action_feedback(fb):
    print(fb)

def action_client():
    client = actionlib.SimpleActionClient('mes_wind', WindSpeedAction)
    client.wait_for_server()
    goal = WindSpeedGoal(True)
    client.send_goal(goal, feedback_cb=action_feedback)
    client.wait_for_result()
    return client.get_result()
    
def get_realtime():
    realtime = None
    while(realtime is None):
        try:
            realtime = rospy.get_param("realtime")
        except:
            realtime = None
        rospy.sleep(0.1)
    return realtime

if __name__ == '__main__':
    realtime = get_realtime()
    rospy.init_node('book_action_client_py')
    if rospy.get_param("wind_speed_measure") == True:
        result = action_client()
        home = os.getenv("HOME")
        path = home+'/copa5/created_csv/wind_speed.txt'
        with open(path, 'w') as file:
            file.write('speed: '+ str(result.speed)+'\n')
            angle = (result.angle/3.14)*180
            file.write('angle: '+ str(angle))
        print("Result:", result.speed, result.angle)
    