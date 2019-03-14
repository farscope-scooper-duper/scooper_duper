#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8

def gripper_c_loop_callback(data):
    rospy.loginfo("Gripper received data from finger pos")
    rospy.loginfo(data)

def spoofer():
    rospy.init_node('gripper', anonymous=True)
    grip_sensor_pub = rospy.Publisher('grip_sensor', Int8, queue_size=10)

    rospy.Subscriber('finger_pos', Bool, gripper_c_loop_callback)

    rate = rospy.Rate(0.5)

    state = 0

    while not rospy.is_shutdown():
        if (state==0):
          grip_sensor_pub.publish(55)
        elif (state == 1):
          grip_sensor_pub.publish(155)
        elif (state == 2):
          grip_sensor_pub.publish(255)
        elif (state ==3):
          grip_sensor_pub.publish(100)
        
        state = state +1
        if state >= 4:
          state = 0
        rospy.loginfo("Grip sensor state output to grip_sensor")
 
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
