#!/usr/bin/env python

import rospy
import signal
import sys
from std_msgs.msg import String,Bool,Int8,Float64
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
suction_state = False
def vacuum_c_loop_callback(data):
    global suction_state
    suction_state = data.data
    rospy.loginfo("Vacuum control received data from suction_state")
    rospy.loginfo(data)


def spoofer():
    global suction_state
    rospy.init_node('vacuum', anonymous=True)

    pressure_sensor_pub = rospy.Publisher('pressure_sensor', Float64, queue_size=10)

    rospy.Subscriber('suction_state', Bool, vacuum_c_loop_callback)
    rate = rospy.Rate(0.5)
    
    #reading = 1000.54
    reading = 0
    while not rospy.is_shutdown():
        
        if (suction_state):
            reading = 850
        else:
            reading = 1010
        pressure_sensor_pub.publish(reading) 
        rospy.loginfo("Pressure reading: {}".format(reading))
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
