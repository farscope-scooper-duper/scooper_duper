#!/usr/bin/env python

import rospy
import signal
import sys
from std_msgs.msg import String,Bool,Int8

def signal_handler(sig, frame):
    print("Exit called")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def gripper_c_loop_callback(data):
    pass
    #rospy.loginfo("Gripper received data from finger pos")
    #rospy.loginfo(data)

def spoofer():
    rospy.init_node('gripper', anonymous=True)
    grip_sensor_pub = rospy.Publisher('grip_sensor', Int8, queue_size=10)

    rospy.Subscriber('finger_pos', Bool, gripper_c_loop_callback)

    rate = rospy.Rate(1.1)

    state = 1
    grip_sensor_pub.publish(state)

    while not rospy.is_shutdown():
        state = input("Set grip state: ")
        grip_sensor_pub.publish(state)

        rospy.loginfo("Grip sensor state output to grip_sensor")
 
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
