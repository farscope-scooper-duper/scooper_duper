#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
#from scooper-duper.msg import ItemList
def c_loop_vision_callback(data):
	rospy.loginfo("Control loop recieved data from topic items_in_view");
	rospy.loginfo(data);
def c_loop_gripsensor_callback(data):
	rospy.loginfo("Control loop recieved from topic grip_sensor");
	rospy.loginfo(data);

def spoofer():
    rospy.init_node('control_loop', anonymous=True)
    finger_pos_pub = rospy.Publisher('finger_pos', Bool, queue_size=10)
    suction_state_pub = rospy.Publisher('suction_state', Bool, queue_size=10)
    EE_pose_pub = rospy.Publisher('t_EE_pose', Transform, queue_size=10)

    #rospy.Subscriber("items_in_view", ItemList , c_loop_vision_callback)
    rospy.Subscriber("grip_sensor", Int8 , c_loop_gripsensor_callback)
    rate = rospy.Rate(0.5) # 10hz

    state = 0

    while not rospy.is_shutdown():
        if (state==0):
          finger_pos_pub.publish(False)
          suction_state_pub.publish(False)
        elif (state == 1):
          finger_pos_pub.publish(True)
          suction_state_pub.publish(False)
        elif (state == 2):
          finger_pos_pub.publish(False)
          suction_state_pub.publish(True)
        elif (state ==3):
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
