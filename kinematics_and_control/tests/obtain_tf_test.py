#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from motion_executor import motion_executor
rospy.init_node('positon_test',anonymous=True)
m = motion_executor()
rospy.sleep(1.0)
m.print_pose()
