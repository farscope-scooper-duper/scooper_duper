#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *

def vacuum_c_loop_callback(data):
    rospy.loginfo("Vacuum control received data from suction_state")
    rospy.loginfo(data)


def spoofer():
    rospy.init_node('vacuum', anonymous=True)

    rospy.Subscriber('suction_state', Bool, vacuum_c_loop_callback)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
