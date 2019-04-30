#!/usr/bin/env python

import rospy
import signal
import sys
from std_msgs.msg import Float64

def spoofer():
    rospy.init_node('pressure_sensor_spoof', anonymous=True)
    pressure_sensor_pub = rospy.Publisher('pressure_sensor', Float64, queue_size=10)

    rate = rospy.Rate(1.1)
    #reading = 1000.54
    reading = 0
    while not rospy.is_shutdown():
        pressure_sensor_pub.publish(reading)
        rospy.loginfo("Pressure reading: {}".format(reading))
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
