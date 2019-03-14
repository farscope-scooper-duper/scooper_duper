#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *

def motion_execute_c_loop_callback(data):
    rospy.loginfo("Motion executor received data from t_EE_pose")
    rospy.loginfo(data)


def spoofer():
    #To-arm publishing left out for the time being
    rospy.init_node('motion_execute', anonymous=True)

    rospy.Subscriber("t_EE_pose", Transform , motion_execute_c_loop_callback)
    rate = rospy.Rate(0.5)
        
    rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
