#!/usr/bin/env python

import signal
import sys
import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *

def signal_handler(sig, frame):
    print("Exit called")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def c_loop_vision_callback(data):
	rospy.loginfo("Control loop recieved data from topic item_in_view");
	rospy.loginfo(data);

def c_loop_gripsensor_callback(data):
	rospy.loginfo("Control loop recieved from topic grip_sensor");
	rospy.loginfo(data);

def spoofer():
    rospy.init_node('control_loop', anonymous=True)
    finger_pos_pub = rospy.Publisher('finger_pos', Bool, queue_size=10)
    suction_state_pub = rospy.Publisher('suction_state', Bool, queue_size=10)
    EE_pose_pub = rospy.Publisher('t_EE_pose', Transform, queue_size=10)
    target_item_pub = rospy.Publisher('target_item', String, queue_size=10)
    target_item_pub.publish('sharpie_accent_tank_style_highlighters')

    rospy.Subscriber("item_in_view", Bool , c_loop_vision_callback)
    rospy.Subscriber("grip_sensor", Int8 , c_loop_gripsensor_callback)
    rate = rospy.Rate(0.5)

    state = 0
    counter = 0
    while not rospy.is_shutdown():
        if(counter < 20):
            counter = counter + 1
            target_item_pub.publish('sharpie_accent_tank_style_highlighters')
        else:
            target_item_pub.publish('adventures_of_huckleberry_finn_book')
        if (state==0):
          finger_pos_pub.publish(False)
          suction_state_pub.publish(False)
        elif (state == 1):
          finger_pos_pub.publish(True)
          suction_state_pub.publish(False)
        elif (state == 2):
          finger_pos_pub.publish(False)
          suction_state_pub.publish(True)
        elif (state == 3):
          finger_pos_pub.publish(True)
          suction_state_pub.publish(True)
        
        state = state +1
        if state >= 4:
          state = 0
        rospy.loginfo("Gripper and vacuum toggled");
        target_pose = Transform()
        target_pose.translation = Vector3(0,0,0)
        target_pose.rotation = Quaternion(0,0,0,0)
        EE_pose_pub.publish(target_pose)
        rospy.loginfo("End effector goal pose");
 
				
				
        
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
